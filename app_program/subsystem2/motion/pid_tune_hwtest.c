/*
 * pid_tune_hwtest.c - closed-loop wheel speed PID tuning (SENSOR_TEST=pid_tune).
 *
 * Runs the vehicle's own wheel_speed_control (one PID per wheel, output =
 * PID_BASE + correction, clamped to 0..PID_MAX) at the vehicle's update rate
 * (odometry every PID_PERIOD_MS), with wheel speeds taken from the encoders.
 * Use it to pick PID_KP / PID_KI / PID_KD / PID_BASE for motion_config in
 * app_main.c.
 *
 * Sequence (one GO press per step):
 *   1  step response, wheels IN THE AIR, for PID_AIR_MS
 *   2  forward  PID_DIST_MM on the floor
 *   3  reverse  PID_DIST_MM on the floor (back along the same line)
 *   4  forward arc left  PID_DIST_MM: wheels at PID_SPEED -/+ PID_TURN, as a
 *      line-steering VELOCITY command drives them (PID_TURN=0 skips 4-5)
 *   5  forward arc right
 *
 * Each step prints a trace (t, speed and PWM per wheel; PID_TRACE=0 hides it)
 * and a summary per wheel: rise time to 90 %, overshoot, steady-state speed
 * error, ripple and the PWM it settled at, then tuning hints.
 *
 * Safety: nothing moves until GO (GP20) is pressed; GO during a move stops the
 * motors and ends the test. Moves also stop on timeout, a stalled wheel, a
 * wheel turning the wrong way, or an encoder fault.
 */
#include "../../app.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "motor_driver.h"
#include "wheel_speed_control.h"
#include "pid_tune_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 14

/* Gains are floats; the makefile passes them as written (2, 2.5, 0.8 ...). */
#ifndef PID_KP
#define PID_KP 2.0
#endif
#ifndef PID_KI
#define PID_KI 0.0
#endif
#ifndef PID_KD
#define PID_KD 0.0
#endif
#ifndef PID_SPEED
#define PID_SPEED 15U         /* target, cm/s (vehicle limit is 20) */
#endif
#ifndef PID_BASE
#define PID_BASE 20U          /* PWM % before correction (feed-forward) */
#endif
#ifndef PID_MAX
#define PID_MAX 60U           /* PWM % ceiling */
#endif
#ifndef PID_PERIOD_MS
#define PID_PERIOD_MS 50U     /* = odometry publish period in app_main.c */
#endif
#ifndef PID_DIST_MM
#define PID_DIST_MM 400U
#endif
#ifndef PID_AIR_MS
#define PID_AIR_MS 3000U
#endif
#ifndef PID_TRACE
#define PID_TRACE 1U
#endif
#ifndef PID_TURN
#define PID_TURN 4U           /* arc steps: wheels at SPEED -/+ TURN; 0 skips */
#endif
#ifndef PID_FF
#define PID_FF 0.0            /* feed-forward %PWM per cm/s around PID_BASE at PID_SPEED */
#endif
_Static_assert((PID_FF) >= 0.0 && (PID_FF) <= 5.0, "need 0 <= PID_FF <= 5");

_Static_assert(PID_SPEED >= 3U && PID_SPEED <= 40U, "need 3 <= PID_SPEED <= 40 cm/s");
_Static_assert(PID_BASE >= 1U && PID_BASE <= PID_MAX && PID_MAX <= 80U,
	       "need 1 <= PID_BASE <= PID_MAX <= 80");
_Static_assert(PID_PERIOD_MS >= 10U && PID_PERIOD_MS <= 200U, "need 10 <= PID_PERIOD_MS <= 200");
_Static_assert(PID_DIST_MM >= 100U && PID_DIST_MM <= 2000U, "need 100 <= PID_DIST_MM <= 2000");
_Static_assert(PID_AIR_MS >= 1000U && PID_AIR_MS <= 10000U, "need 1000 <= PID_AIR_MS <= 10000");
_Static_assert(PID_TRACE <= 1U, "PID_TRACE must be 0 or 1");
_Static_assert(PID_TURN < PID_SPEED && PID_SPEED + PID_TURN <= 40U,
	       "need PID_TURN < PID_SPEED and PID_SPEED + PID_TURN <= 40");

/* Same calibration as the vehicle (app_main.c) and motor_moves_hwtest.c. */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define TRACK_MM           110.0f
#define LEFT_SIGN          (-1)    /* left encoder counts down going forward */
#define RIGHT_SIGN         1
#define LEFT_MOTOR_FWD     1       /* motion_task.c MOTION_LEFT_POLARITY */
#define RIGHT_MOTOR_FWD    (-1)    /* motion_task.c MOTION_RIGHT_POLARITY */

#define GO_PIN             20U
#define TAG                "[HWTEST][PID] "
#define PI_F               3.14159265f
#define POLL_MS            10U     /* GO button / safety checks */
#define START_DELAY_MS     3000U
#define SETTLE_MS          600U
#define STALL_MS           1000U
#define STALL_MM           2.0f
#define WRONG_WAY_MM       20.0f
#define MAX_SAMPLES        400U    /* 20 s at 50 ms; later samples not analysed */
#define STEADY_MIN_MS      500U    /* never count the first 0.5 s as steady */

typedef struct {
	const char *name;
	const char *setup;
	int8_t dir;             /* +1 forward, -1 reverse */
	bool timed;             /* true: PID_AIR_MS; false: PID_DIST_MM */
	int8_t turn;            /* +1 arc left, -1 arc right (PID_TURN), 0 straight */
} step_t;

static const step_t s_steps[] = {
	{ "step response (in the air)",
	  "LIFT the robot onto a block so BOTH wheels spin freely in the air", 1, true, 0 },
	{ "forward on the floor",
	  "put the robot ON THE FLOOR, nose along a tape line", 1, false, 0 },
	{ "reverse on the floor",
	  "leave it where it stopped; it backs up along the same line", -1, false, 0 },
	{ "forward arc LEFT (line steering)",
	  "re-square on the tape; needs room ahead and to the LEFT", 1, false, 1 },
	{ "forward arc RIGHT (line steering)",
	  "re-square on the tape; needs room ahead and to the RIGHT", 1, false, -1 },
};
#define STEP_COUNT (sizeof(s_steps) / sizeof(s_steps[0]))

/* Wheel target for a step: left = speed - turn, right = speed + turn, as the
 * motion task does for a VELOCITY command with angular_velocity = turn. */
static float wheel_target(const step_t *st, bool left)
{
	float turn = (float)st->turn * (float)PID_TURN;

	return (float)PID_SPEED + (left ? -turn : turn);
}

typedef enum { RUN_OK = 0, RUN_ABORT_GO, RUN_TIMEOUT, RUN_STALL_L, RUN_STALL_R,
	       RUN_WRONG_L, RUN_WRONG_R, RUN_ENCODER, RUN_PID } run_status_t;

typedef struct {
	float l, r;             /* measured speed, cm/s, in the commanded direction */
	uint8_t pwm_l, pwm_r;
} sample_t;

static sample_t s_samples[MAX_SAMPLES];
static bool s_go_last_pressed;

static float absf(float v) { return v < 0.0f ? -v : v; }

static void fail_forever(const char *what, ER error)
{
	(void)motor_brake_all();
	(void)motor_emergency_stop();
	for (;;) {
		tm_printf((UB*)TAG "%s failed: %d (motors stopped)\n", what, (int)error);
		tk_dly_tsk(5000U);
	}
}

static bool go_pressed_raw(void)
{
	return gpio_get_val(GO_PIN) == 0U;           /* active low */
}

static bool go_edge(void)
{
	bool now = go_pressed_raw();
	bool edge = false;

	if (now != s_go_last_pressed) {
		tk_dly_tsk(POLL_MS * 2U);
		if (go_pressed_raw() == now) {
			edge = now && !s_go_last_pressed;
			s_go_last_pressed = now;
		}
	}
	return edge;
}

static void wait_for_go(void)
{
	while (!go_edge())
		tk_dly_tsk(POLL_MS);
	while (go_pressed_raw())
		tk_dly_tsk(POLL_MS);
	s_go_last_pressed = false;
}

/* Wheel travel in mm since boot, forward-positive. */
static bool read_mm(float *l, float *r)
{
	encoder_snapshot_t s;
	const float mm_per_count = WHEEL_CIRC_MM / COUNTS_PER_REV;

	if (encoder_source_snapshot(&s) < E_OK || !s.valid) return false;
	*l = (float)(LEFT_SIGN * s.left_count) * mm_per_count;
	*r = (float)(RIGHT_SIGN * s.right_count) * mm_per_count;
	return true;
}

static uint32_t now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

static void drive(int dir, uint8_t pwm_l, uint8_t pwm_r)
{
	/* E_BUSY = reversal brake interval; the next sample retries. */
	(void)motor_set_speed(MOTOR_1, (int16_t)(dir * (int)pwm_l * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(dir * (int)pwm_r * RIGHT_MOTOR_FWD));
}

/* "-12.3" (tm_printf has no %f). */
static const char *f1(char *buf, float v)
{
	int t = (int)__builtin_lroundf(v * 10.0f);
	unsigned a = (unsigned)(t < 0 ? -t : t);
	unsigned whole = a / 10U, i = 0U, n = 0U;
	char digits[10];

	if (t < 0) buf[i++] = '-';
	do { digits[n++] = (char)('0' + whole % 10U); whole /= 10U; } while (whole && n < 9U);
	while (n) buf[i++] = digits[--n];
	buf[i++] = '.';
	buf[i++] = (char)('0' + a % 10U);
	buf[i] = '\0';
	return buf;
}

/* "1.25" for gains. */
static const char *f2(char *buf, float v)
{
	int t = (int)__builtin_lroundf(v * 100.0f);
	unsigned a = (unsigned)(t < 0 ? -t : t);
	unsigned i = 0U;
	char tmp[16];

	f1(tmp, (float)(a / 100U));             /* "N.0" */
	if (t < 0) buf[i++] = '-';
	for (unsigned k = 0U; tmp[k] != '.'; k++) buf[i++] = tmp[k];
	buf[i++] = '.';
	buf[i++] = (char)('0' + (a / 10U) % 10U);
	buf[i++] = (char)('0' + a % 10U);
	buf[i] = '\0';
	return buf;
}

typedef struct {
	run_status_t status;
	unsigned n;             /* samples stored */
	uint32_t run_ms;
	float l_mm, r_mm;       /* travel at brake, forward-positive */
	float l_fin, r_fin;     /* after coasting */
} run_result_t;

static run_status_t run_step(const step_t *st, wheel_speed_control_t *pid, run_result_t *res)
{
	const float target = (float)PID_SPEED;
	const uint32_t timeout_ms = st->timed ? PID_AIR_MS + 1000U :
		2000U + (uint32_t)(2.0f * (float)PID_DIST_MM / (target * 10.0f) * 1000.0f);
	float l0, r0, l, r, pl, pr, prog_l = 0.0f, prog_r = 0.0f;
	uint32_t t0, t, next, last_sample, prog_l_ms = 0U, prog_r_ms = 0U;
	wheel_speed_output_t out = { PID_BASE, PID_BASE };
	run_status_t status = RUN_OK;
	char b1[16], b2[16];

	*res = (run_result_t){ 0 };
	if (!wheel_speed_control_reset(pid) ||
	    !wheel_speed_control_set_targets(pid, wheel_target(st, true),
					     wheel_target(st, false)))
		return RUN_PID;
	if (!read_mm(&l0, &r0)) return RUN_ENCODER;
	pl = l = l0;
	pr = r = r0;
	t0 = now_ms();
	last_sample = t0;
	next = t0 + PID_PERIOD_MS;
	drive(st->dir, out.left_pwm_percent, out.right_pwm_percent);
	if (PID_TRACE)
		tm_printf((UB*)TAG "  t_ms  L cm/s pwm   R cm/s pwm\n");

	for (;;) {
		float dl, dr;

		tk_dly_tsk(POLL_MS);
		t = now_ms();
		if (!read_mm(&l, &r)) { status = RUN_ENCODER; break; }
		dl = l - l0;
		dr = r - r0;
		res->run_ms = t - t0;

		if (go_pressed_raw()) { status = RUN_ABORT_GO; break; }
		if (res->run_ms > timeout_ms) { status = RUN_TIMEOUT; break; }
		if (dl * (float)st->dir < -WRONG_WAY_MM) { status = RUN_WRONG_L; break; }
		if (dr * (float)st->dir < -WRONG_WAY_MM) { status = RUN_WRONG_R; break; }
		if (absf(dl - prog_l) >= STALL_MM) { prog_l = dl; prog_l_ms = res->run_ms; }
		if (absf(dr - prog_r) >= STALL_MM) { prog_r = dr; prog_r_ms = res->run_ms; }
		if (res->run_ms - prog_l_ms > STALL_MS) { status = RUN_STALL_L; break; }
		if (res->run_ms - prog_r_ms > STALL_MS) { status = RUN_STALL_R; break; }

		if (st->timed) {
			if (res->run_ms >= PID_AIR_MS) break;
		} else if ((absf(dl) + absf(dr)) * 0.5f >= (float)PID_DIST_MM) {
			break;
		}

		if ((int32_t)(t - next) < 0) continue;
		next += PID_PERIOD_MS;

		/* Speed over the last period, as the vehicle's motion_feedback does,
		 * as a magnitude in the commanded direction (the PID wants >= 0). */
		{
			float dt = (float)(t - last_sample) / 1000.0f;
			float sl = (l - pl) * (float)st->dir / 10.0f / dt;
			float sr = (r - pr) * (float)st->dir / 10.0f / dt;

			pl = l;
			pr = r;
			last_sample = t;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (!wheel_speed_control_update(pid, sl, sr, dt, &out)) {
				status = RUN_PID;
				break;
			}
			if (res->n < MAX_SAMPLES)
				s_samples[res->n++] = (sample_t){ sl, sr, out.left_pwm_percent,
								  out.right_pwm_percent };
			if (PID_TRACE)
				tm_printf((UB*)TAG "  %5u  %6s %3u   %6s %3u\n", (unsigned)res->run_ms,
					  f1(b1, sl), (unsigned)out.left_pwm_percent,
					  f1(b2, sr), (unsigned)out.right_pwm_percent);
		}
		drive(st->dir, out.left_pwm_percent, out.right_pwm_percent);
	}
	(void)motor_brake_all();
	res->l_mm = l - l0;
	res->r_mm = r - r0;
	tk_dly_tsk(SETTLE_MS);
	if (read_mm(&l, &r)) {
		res->l_fin = l - l0;
		res->r_fin = r - r0;
	} else {
		res->l_fin = res->l_mm;
		res->r_fin = res->r_mm;
	}
	res->status = status;
	return status;
}

typedef struct {
	bool risen;
	uint32_t rise_ms;
	float peak, mean, lo, hi, pwm;
	bool saturated;
} wheel_stats_t;

static wheel_stats_t wheel_stats(unsigned n, bool left, float target)
{
	unsigned start = n / 2U, steady_n = 0U, i;
	wheel_stats_t w = { false, 0U, 0.0f, 0.0f, 1e9f, -1e9f, 0.0f, false };

	if (start * PID_PERIOD_MS < STEADY_MIN_MS) start = STEADY_MIN_MS / PID_PERIOD_MS;
	for (i = 0U; i < n; i++) {
		float v = left ? s_samples[i].l : s_samples[i].r;
		uint8_t p = left ? s_samples[i].pwm_l : s_samples[i].pwm_r;

		if (!w.risen && v >= 0.9f * target) { w.risen = true; w.rise_ms = (i + 1U) * PID_PERIOD_MS; }
		if (v > w.peak) w.peak = v;
		if (i >= start) {
			w.mean += v;
			w.pwm += (float)p;
			if (v < w.lo) w.lo = v;
			if (v > w.hi) w.hi = v;
			if (p >= PID_MAX) w.saturated = true;
			steady_n++;
		}
	}
	if (steady_n != 0U) {
		w.mean /= (float)steady_n;
		w.pwm /= (float)steady_n;
	} else {
		w.lo = w.hi = 0.0f;
	}
	return w;
}

static void print_wheel(const char *name, const wheel_stats_t *w, float target)
{
	char b0[16], b1[16], b2[16], b3[16], b4[16], b5[16];

	if (w->risen)
		tm_printf((UB*)TAG "  %s: target %s  rise %u ms  overshoot %s%%  steady %s cm/s "
			  "(%s%% err)  ripple %s cm/s  pwm %s%%\n", name, f1(b0, target),
			  (unsigned)w->rise_ms,
			  f1(b1, (w->peak - target) / target * 100.0f), f1(b2, w->mean),
			  f1(b3, (w->mean - target) / target * 100.0f), f1(b4, w->hi - w->lo),
			  f1(b5, w->pwm));
	else
		tm_printf((UB*)TAG "  %s: target %s  NEVER reached 90%% of it; steady %s cm/s  "
			  "pwm %s%%\n", name, f1(b0, target), f1(b2, w->mean), f1(b5, w->pwm));
}

static void report(const step_t *st, const run_result_t *res)
{
	const float target = (float)PID_SPEED;
	const float tl = wheel_target(st, true), tr = wheel_target(st, false);
	char b1[16], b2[16], b3[16], b4[16];
	wheel_stats_t wl, wr;
	float err_l, err_r, ripple, overshoot, os_l, os_r, lr_pct;
	bool hinted = false;

	tm_printf((UB*)TAG "  ran %u ms; travel at brake L %s mm  R %s mm; after coast L %s  R %s\n",
		  (unsigned)res->run_ms, f1(b1, res->l_mm), f1(b2, res->r_mm),
		  f1(b3, res->l_fin), f1(b4, res->r_fin));
	if (res->n * PID_PERIOD_MS < STEADY_MIN_MS + 2U * PID_PERIOD_MS) {
		tm_printf((UB*)TAG "  too short to analyse\n");
		return;
	}
	wl = wheel_stats(res->n, true, tl);
	wr = wheel_stats(res->n, false, tr);
	print_wheel("LEFT ", &wl, tl);
	print_wheel("RIGHT", &wr, tr);

	if (st->turn == 0) {
		lr_pct = (wl.mean - wr.mean) / target * 100.0f;
		tm_printf((UB*)TAG "  steady L-R difference %s%% of target\n", f1(b1, lr_pct));
		if (!st->timed) {
			float h = (res->r_fin - res->l_fin) / TRACK_MM * (180.0f / PI_F);
			tm_printf((UB*)TAG "  encoder heading drift %s%s deg -- measure the real drift too\n",
				  h > 0.05f ? "L" : (h < -0.05f ? "R" : ""), f1(b1, absf(h)));
		}
	} else {
		/* Arc: wheel speed ratio and the curve it drove, from the encoders. */
		float h = (res->r_fin - res->l_fin) / TRACK_MM * (180.0f / PI_F);
		float d = res->r_fin - res->l_fin;
		float radius = absf(d) > 1.0f ?
			absf(TRACK_MM * 0.5f * (res->l_fin + res->r_fin) / d) : 0.0f;
		float inner = st->turn > 0 ? wl.mean : wr.mean;
		float outer = st->turn > 0 ? wr.mean : wl.mean;

		tm_printf((UB*)TAG "  inner/outer speed %s%% (commanded %s%%)\n",
			  f1(b1, outer > 0.0f ? inner / outer * 100.0f : 0.0f),
			  f1(b2, (float)(PID_SPEED - PID_TURN) / (float)(PID_SPEED + PID_TURN) * 100.0f));
		tm_printf((UB*)TAG "  turned %s%s deg, radius ~%s mm (robot centre) -- compare with "
			  "the mat\n", h > 0.05f ? "L" : (h < -0.05f ? "R" : ""), f1(b3, absf(h)),
			  f1(b4, radius));
		/* Settled PWM per wheel = what each speed really needs, whatever the
		 * feed-forward: their spread over the speed spread is the slope. */
		{
			float pwm_in = st->turn > 0 ? wl.pwm : wr.pwm;
			float pwm_out = st->turn > 0 ? wr.pwm : wl.pwm;
			float v_in = st->turn > 0 ? wl.mean : wr.mean;
			float v_out = st->turn > 0 ? wr.mean : wl.mean;

			if (v_out - v_in > 1.0f)
				tm_printf((UB*)TAG "  settled PWM inner %s%% outer %s%% -> feed-forward "
					  "slope ~%s %%/(cm/s) (vehicle uses 1.4)\n", f1(b1, pwm_in),
					  f1(b2, pwm_out),
					  f1(b3, (pwm_out - pwm_in) / (v_out - v_in)));
		}
	}

	/* Hints, worst first; each wheel is judged against its own target. */
	err_l = absf(wl.mean - tl) / tl;
	err_r = absf(wr.mean - tr) / tr;
	ripple = (wl.hi - wl.lo > wr.hi - wr.lo ? wl.hi - wl.lo : wr.hi - wr.lo) / target;
	os_l = (wl.peak - tl) / tl;
	os_r = (wr.peak - tr) / tr;
	overshoot = os_l > os_r ? os_l : os_r;
	if (wl.saturated || wr.saturated) {
		tm_printf((UB*)TAG "  HINT: PWM hit PID_MAX=%u -- lower PID_SPEED or raise PID_MAX\n",
			  (unsigned)PID_MAX);
		hinted = true;
	}
	if (ripple > 0.30f) {
		tm_printf((UB*)TAG "  HINT: oscillating at steady speed (ripple %s%% of target) -- "
			  "lower PID_KP%s\n", f1(b1, ripple * 100.0f),
			  (float)(PID_KI) > 0.0f ? " or PID_KI" : "");
		hinted = true;
	} else if (overshoot > 0.20f) {
		tm_printf((UB*)TAG "  HINT: overshoots %s%% on start-up -- lower PID_KP a little, "
			  "or set PID_BASE nearer the steady pwm\n", f1(b1, overshoot * 100.0f));
		hinted = true;
	}
	if (err_l > 0.03f || err_r > 0.03f) {
		if ((float)(PID_KI) <= 0.0f)
			tm_printf((UB*)TAG "  HINT: steady-state error with PID_KI=0 -- add integral, "
				  "try PID_KI=1\n");
		else
			tm_printf((UB*)TAG "  HINT: steady-state error remains -- raise PID_KI, "
				  "or run longer to let it settle\n");
		hinted = true;
	}
	if ((wl.risen && wl.rise_ms > 800U) || (wr.risen && wr.rise_ms > 800U) ||
	    !wl.risen || !wr.risen) {
		tm_printf((UB*)TAG "  HINT: slow to reach speed -- raise PID_KP, or set PID_BASE "
			  "near the steady pwm\n");
		hinted = true;
	}
	if (st->turn == 0) {
		unsigned base = (unsigned)__builtin_lroundf((wl.pwm + wr.pwm) * 0.5f);
		if (base != PID_BASE && base >= 1U && base <= PID_MAX)
			tm_printf((UB*)TAG "  TIP: wheels settled near %u%% PWM; PID_BASE=%u starts "
				  "closer to that (feed-forward)\n", base, base);
	}
	if (!hinted)
		tm_printf((UB*)TAG "  looks well tuned at this speed\n");
}

static const char *status_text(run_status_t s)
{
	switch (s) {
	case RUN_ABORT_GO: return "stopped by GO button";
	case RUN_TIMEOUT:  return "timeout: distance not reached (too slow?)";
	case RUN_STALL_L:  return "LEFT wheel stalled: raise PID_BASE or PID_KP";
	case RUN_STALL_R:  return "RIGHT wheel stalled: raise PID_BASE or PID_KP";
	case RUN_WRONG_L:  return "LEFT wheel turned the WRONG WAY: motor or encoder sign";
	case RUN_WRONG_R:  return "RIGHT wheel turned the WRONG WAY: motor or encoder sign";
	case RUN_ENCODER:  return "encoder INVALID";
	case RUN_PID:      return "PID update rejected its inputs";
	default:           return "ok";
	}
}

static void report_encoder_fault(void)
{
	encoder_fault_t f;

	if (encoder_source_fault(&f) < E_OK || f.reason == ENCODER_FAULT_NONE) return;
	tm_printf((UB*)TAG "  ENCODER FAULT: %s wheel, reason %u (1 both channels jumped, over "
		  "the limit; 2 lost edge; 3 bad level), %u glitches and %u both-channel jumps "
		  "tolerated before it\n", f.wheel == 1U ? "LEFT" : "RIGHT", (unsigned)f.reason,
		  (unsigned)f.glitches, (unsigned)f.ambiguous);
}

void pid_tune_hwtest_run(void)
{
	const wheel_speed_config_t config = {
		(float)(PID_KP), (float)(PID_KI), (float)(PID_KD),
		(float)PID_SPEED, (uint8_t)PID_BASE, (uint8_t)PID_MAX,
		(float)(PID_FF), (float)PID_SPEED
	};
	wheel_speed_control_t pid;
	char b1[16], b2[16], b3[16];
	unsigned i;
	ER er;

	er = gpio_set_pin(GO_PIN, GPIO_MODE_IN);
	if (er < E_OK) fail_forever("GO pin", er);
	clr_w(GPIO(GO_PIN), GPIO_PDE);
	set_w(GPIO(GO_PIN), GPIO_PUE);
	er = encoder_source_register();
	if (er < E_OK) fail_forever("encoder registration", er);
	er = gpio_dispatcher_start();
	if (er < E_OK) fail_forever("GPIO dispatcher", er);
	er = motor_driver_init();
	if (er < E_OK) fail_forever("motor driver init", er);
	if (!wheel_speed_control_init(&pid, &config)) fail_forever("PID config (gains >= 0?)", E_PAR);
	s_go_last_pressed = go_pressed_raw();

	tm_printf((UB*)TAG "wheel speed PID tuning: KP %s KI %s KD %s, target %u cm/s, "
		  "base %u%% max %u%%, every %u ms, arc turn %u cm/s\n", f2(b1, config.kp),
		  f2(b2, config.ki), f2(b3, config.kd), (unsigned)PID_SPEED, (unsigned)PID_BASE,
		  (unsigned)PID_MAX, (unsigned)PID_PERIOD_MS, (unsigned)PID_TURN);
	tm_printf((UB*)TAG "each GO press runs the next step; GO during a move = STOP\n");

	for (i = 0U; i < STEP_COUNT; i++) {
		const step_t *st = &s_steps[i];
		run_result_t res;
		run_status_t status;

		if (st->turn != 0 && PID_TURN == 0U) continue;

		tm_printf((UB*)TAG "SETUP: %s\n", st->setup);
		tm_printf((UB*)TAG "---- step %u/%u: %s -- press GO ----\n", i + 1U,
			  (unsigned)STEP_COUNT, st->name);
		wait_for_go();
		tm_printf((UB*)TAG "moving in %u s, hands clear\n", START_DELAY_MS / 1000U);
		tk_dly_tsk(START_DELAY_MS);
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);

		status = run_step(st, &pid, &res);
		(void)motor_emergency_stop();
		report(st, &res);
		if (status != RUN_OK) {
			tm_printf((UB*)TAG "MOVE ENDED: %s -- test stopped, reset to retry\n",
				  status_text(status));
			if (status == RUN_ENCODER) report_encoder_fault();
			for (;;) tk_dly_tsk(5000U);
		}
	}
	tm_printf((UB*)TAG "==== done; motors stopped. Copy the gains you settle on into "
		  "motion_config in app_main.c. ====\n");
	for (;;) tk_dly_tsk(5000U);
}

#endif /* APP_SENSOR_TEST == 14 */
