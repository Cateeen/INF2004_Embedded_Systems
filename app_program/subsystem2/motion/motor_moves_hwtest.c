/*
 * motor_moves_hwtest.c - basic manoeuvres and wheel balance (SENSOR_TEST=motor_moves).
 *
 * Drives the motors OPEN LOOP (fixed PWM, no speed PID) and uses the wheel
 * encoders only to measure what each wheel really did. That answers the
 * question "do both wheels go the same speed for the same PWM?" and gives a
 * per-wheel trim to correct it.
 *
 * Sequence (one GO press per step unless noted):
 *   1-4  balance 25/40/55/70 %  wheels IN THE AIR, one press runs all four
 *   5    forward  MOVE_DIST_MM
 *   6    reverse  MOVE_DIST_MM
 *   7    spin left 90 in place
 *   8    spin right 90 in place
 *   9    spin 180 in place
 *   10   forward + right arc, 90 deg
 *   11   forward + left arc, 90 deg
 * MOVE_SKIP_BALANCE=1 skips steps 1-4 and starts at the floor moves.
 *
 * Per-wheel command = dir * PWM * (wheel share %) * (MOVE_TRIM_x %).
 * Heading (positive = LEFT) = (right_mm - left_mm) / MOVE_TRACK_MM.
 *
 * Safety: nothing moves until GO (GP20) is pressed; GO during a move stops the
 * motors and ends the test. Moves also stop on timeout, a stalled wheel or a
 * wheel turning the wrong way.
 */
#include "../../app.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "motor_driver.h"
#include "motor_moves_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 13

#ifndef MOVE_PWM
#define MOVE_PWM 50U          /* % for the floor moves */
#endif
#ifndef MOVE_TRIM_L
#define MOVE_TRIM_L 100U      /* % scale on the left wheel command */
#endif
#ifndef MOVE_TRIM_R
#define MOVE_TRIM_R 100U      /* % scale on the right wheel command */
#endif
#ifndef MOVE_DIST_MM
#define MOVE_DIST_MM 500U     /* forward / reverse distance */
#endif
#ifndef MOVE_TRACK_MM
#define MOVE_TRACK_MM 110U    /* wheel centre to centre */
#endif
#ifndef MOVE_ARC_INNER
#define MOVE_ARC_INNER 50U    /* inner wheel share (% of outer) on the arcs */
#endif
#ifndef MOVE_SKIP_BALANCE
#define MOVE_SKIP_BALANCE 0U  /* 1 = start at the floor moves (step 5) */
#endif

_Static_assert(MOVE_PWM >= 15U && MOVE_PWM <= 80U, "need 15 <= MOVE_PWM <= 80");
_Static_assert(MOVE_TRIM_L >= 50U && MOVE_TRIM_L <= 100U, "need 50 <= MOVE_TRIM_L <= 100");
_Static_assert(MOVE_TRIM_R >= 50U && MOVE_TRIM_R <= 100U, "need 50 <= MOVE_TRIM_R <= 100");
_Static_assert(MOVE_DIST_MM >= 100U && MOVE_DIST_MM <= 2000U, "need 100 <= MOVE_DIST_MM <= 2000");
_Static_assert(MOVE_TRACK_MM >= 50U && MOVE_TRACK_MM <= 300U, "MOVE_TRACK_MM out of range");
_Static_assert(MOVE_ARC_INNER <= 90U, "MOVE_ARC_INNER must be <= 90");
_Static_assert(MOVE_SKIP_BALANCE <= 1U, "MOVE_SKIP_BALANCE must be 0 or 1");

/* Same calibration as the vehicle (app_main.c) and spin_heading_hwtest.c. */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define LEFT_SIGN          (-1)    /* left encoder counts down going forward */
#define RIGHT_SIGN         1
#define LEFT_MOTOR_FWD     1       /* motion_task.c MOTION_LEFT_POLARITY */
#define RIGHT_MOTOR_FWD    (-1)    /* motion_task.c MOTION_RIGHT_POLARITY */

#define GO_PIN             20U
#define TAG                "[HWTEST][MOVE] "
#define PI_F               3.14159265f
#define LOOP_MS            10U
#define START_DELAY_MS     3000U
#define CHAIN_PAUSE_MS     800U    /* between chained balance levels */
#define SETTLE_MS          600U    /* after braking, before the coast reading */
#define ACCEL_MS           400U    /* skipped before measuring steady speed */
#define MIN_STEADY_MS      200U
#define TIMEOUT_BASE_MS    3000U
#define TIMEOUT_MS_PER_MM  8U      /* ~125 mm/s worst case */
#define TIMEOUT_MS_PER_DEG 50U
#define STALL_MS           1000U   /* commanded wheel moved < STALL_MM in this */
#define STALL_MM           2.0f
#define WRONG_WAY_MM       20.0f
#define SLOW_ZONE_DEG      15.0f   /* spins only: slow down near the target */
#define SLOW_SCALE         0.7f
#define SLOW_MIN_PWM       20
#define DEAD_MM_S          20.0f   /* below this a wheel "did not turn" */

typedef enum { END_TIMED, END_DISTANCE, END_HEADING } move_end_t;

typedef struct {
	const char *name;
	const char *setup;      /* printed before GO; NULL = none */
	int8_t left_dir;        /* +1 forward, -1 reverse */
	int8_t right_dir;
	uint8_t left_share;     /* % of the base PWM */
	uint8_t right_share;
	uint8_t base_pwm;       /* 0 = MOVE_PWM */
	move_end_t end;
	int16_t target;         /* ms, mm, or deg (+ = left) */
	bool chain;             /* runs straight after the previous step, no GO */
} move_t;

typedef enum { MOVE_OK = 0, MOVE_ABORT_GO, MOVE_TIMEOUT, MOVE_STALL_L,
	       MOVE_STALL_R, MOVE_WRONG_L, MOVE_WRONG_R, MOVE_IO } move_status_t;

typedef struct {
	float l_mm, r_mm;        /* travel when the brake was applied */
	float l_fin, r_fin;      /* travel after SETTLE_MS (includes coasting) */
	float l_speed, r_speed;  /* steady-state mm/s */
	int16_t l_cmd, r_cmd;    /* forward-positive command actually sent */
	uint32_t run_ms;
	bool have_speed;
} move_result_t;

static const move_t s_moves[] = {
	{ "balance 25%", "LIFT the robot onto a block so BOTH wheels spin freely in the air",
	  1, 1, 100, 100, 25, END_TIMED, 2000, false },
	{ "balance 40%", NULL, 1, 1, 100, 100, 40, END_TIMED, 2000, true },
	{ "balance 55%", NULL, 1, 1, 100, 100, 55, END_TIMED, 2000, true },
	{ "balance 70%", NULL, 1, 1, 100, 100, 70, END_TIMED, 2000, true },
	{ "forward", "put the robot ON THE FLOOR, nose along a tape line, ~1 m clear ahead",
	  1, 1, 100, 100, 0, END_DISTANCE, (int16_t)MOVE_DIST_MM, false },
	{ "reverse", "leave it where it stopped; it backs up along the same line",
	  -1, -1, 100, 100, 0, END_DISTANCE, (int16_t)MOVE_DIST_MM, false },
	{ "spin left 90", "square the robot on a tape cross (mark L90 / R90 / 180)",
	  -1, 1, 100, 100, 0, END_HEADING, 90, false },
	{ "spin right 90", "do NOT touch it; it should come back to the start line",
	  1, -1, 100, 100, 0, END_HEADING, -90, false },
	{ "spin 180", "re-square on the tape cross if it drifted",
	  -1, 1, 100, 100, 0, END_HEADING, 180, false },
	{ "forward + right arc", "re-square; needs ~0.5 m clear ahead and to the RIGHT",
	  1, 1, 100, (uint8_t)MOVE_ARC_INNER, 0, END_HEADING, -90, false },
	{ "forward + left arc", "re-square; needs ~0.5 m clear ahead and to the LEFT",
	  1, 1, (uint8_t)MOVE_ARC_INNER, 100, 0, END_HEADING, 90, false },
};
#define MOVE_COUNT (sizeof(s_moves) / sizeof(s_moves[0]))
#define BALANCE_STEPS 4U   /* the in-air sweep at the start of s_moves */

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

/* True once per press (debounced: two samples apart). */
static bool go_edge(void)
{
	bool now = go_pressed_raw();
	bool edge = false;

	if (now != s_go_last_pressed) {
		tk_dly_tsk(LOOP_MS * 2U);
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
		tk_dly_tsk(LOOP_MS);
	while (go_pressed_raw())            /* a held button would abort the move */
		tk_dly_tsk(LOOP_MS);
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

static float heading_deg(float dl, float dr)
{
	return (dr - dl) / (float)MOVE_TRACK_MM * (180.0f / PI_F);
}

static uint32_t elapsed_ms(const SYSTIM *t0)
{
	SYSTIM now;

	tk_get_otm(&now);
	return now.lo - t0->lo;
}

/* Forward-positive command for one wheel. */
static int16_t wheel_cmd(int dir, unsigned share, unsigned pwm, unsigned trim, float scale)
{
	float v = (float)pwm * (float)share / 100.0f * (float)trim / 100.0f * scale;
	int mag = (int)(v + 0.5f);

	if (scale < 1.0f && mag > 0 && mag < SLOW_MIN_PWM) mag = SLOW_MIN_PWM;
	return (int16_t)(dir * mag);
}

static void drive(int16_t left, int16_t right)
{
	/* E_BUSY = reversal brake interval; the next loop retries. */
	(void)motor_set_speed(MOTOR_1, (int16_t)(left * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(right * RIGHT_MOTOR_FWD));
}

static uint32_t timeout_for(const move_t *m)
{
	if (m->end == END_TIMED) return (uint32_t)m->target + 1000U;
	if (m->end == END_DISTANCE)
		return TIMEOUT_BASE_MS + TIMEOUT_MS_PER_MM * (uint32_t)m->target;
	return TIMEOUT_BASE_MS + TIMEOUT_MS_PER_DEG *
	       (uint32_t)(m->target < 0 ? -m->target : m->target);
}

static move_status_t run_move(const move_t *m, move_result_t *res)
{
	const unsigned pwm = m->base_pwm != 0U ? m->base_pwm : MOVE_PWM;
	const bool spin = m->left_dir != m->right_dir;
	const uint32_t timeout_ms = timeout_for(m);
	float l0, r0, l, r, dl = 0.0f, dr = 0.0f;
	float steady_l = 0.0f, steady_r = 0.0f, prog_l = 0.0f, prog_r = 0.0f;
	uint32_t elapsed = 0U, steady_ms = 0U, prog_l_ms = 0U, prog_r_ms = 0U;
	bool steady = false;
	move_status_t status = MOVE_OK;
	SYSTIM t0;

	*res = (move_result_t){ 0 };
	res->l_cmd = wheel_cmd(m->left_dir, m->left_share, pwm, MOVE_TRIM_L, 1.0f);
	res->r_cmd = wheel_cmd(m->right_dir, m->right_share, pwm, MOVE_TRIM_R, 1.0f);
	if (!read_mm(&l0, &r0)) return MOVE_IO;
	tk_get_otm(&t0);

	for (;;) {
		float scale = 1.0f;

		if (!read_mm(&l, &r)) { status = MOVE_IO; break; }
		dl = l - l0;
		dr = r - r0;
		elapsed = elapsed_ms(&t0);
		if (!steady && elapsed >= ACCEL_MS) {
			steady = true;
			steady_l = dl;
			steady_r = dr;
			steady_ms = elapsed;
		}

		if (m->end == END_TIMED) {
			if (elapsed >= (uint32_t)m->target) break;
		} else if (m->end == END_DISTANCE) {
			if ((absf(dl) + absf(dr)) * 0.5f >= (float)m->target) break;
		} else {
			float sign = m->target > 0 ? 1.0f : -1.0f;
			float remaining = ((float)m->target - heading_deg(dl, dr)) * sign;

			if (remaining <= 0.0f) break;
			if (spin && remaining < SLOW_ZONE_DEG) scale = SLOW_SCALE;
		}

		if (go_pressed_raw()) { status = MOVE_ABORT_GO; break; }
		if (elapsed > timeout_ms) { status = MOVE_TIMEOUT; break; }
		if (dl * (float)m->left_dir < -WRONG_WAY_MM) { status = MOVE_WRONG_L; break; }
		if (dr * (float)m->right_dir < -WRONG_WAY_MM) { status = MOVE_WRONG_R; break; }

		/* Per-wheel stall; skipped on the air balance (dead band is a result
		 * there, not a fault) and for a wheel commanded to 0. */
		if (absf(dl - prog_l) >= STALL_MM) { prog_l = dl; prog_l_ms = elapsed; }
		if (absf(dr - prog_r) >= STALL_MM) { prog_r = dr; prog_r_ms = elapsed; }
		if (m->end != END_TIMED) {
			if (res->l_cmd != 0 && elapsed - prog_l_ms > STALL_MS) { status = MOVE_STALL_L; break; }
			if (res->r_cmd != 0 && elapsed - prog_r_ms > STALL_MS) { status = MOVE_STALL_R; break; }
		}

		drive(wheel_cmd(m->left_dir, m->left_share, pwm, MOVE_TRIM_L, scale),
		      wheel_cmd(m->right_dir, m->right_share, pwm, MOVE_TRIM_R, scale));
		tk_dly_tsk(LOOP_MS);
	}
	(void)motor_brake_all();

	res->l_mm = dl;
	res->r_mm = dr;
	res->run_ms = elapsed;
	res->have_speed = steady && elapsed >= steady_ms + MIN_STEADY_MS;
	if (res->have_speed) {
		float secs = (float)(elapsed - steady_ms) / 1000.0f;
		res->l_speed = (dl - steady_l) / secs;
		res->r_speed = (dr - steady_r) / secs;
	} else {
		res->l_speed = res->r_speed = 0.0f;
	}

	tk_dly_tsk(SETTLE_MS);
	if (read_mm(&l, &r)) {
		res->l_fin = l - l0;
		res->r_fin = r - r0;
	} else {
		res->l_fin = dl;
		res->r_fin = dr;
		if (status == MOVE_OK) status = MOVE_IO;
	}
	return status;
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

/* Heading as L12.3 / R12.3. */
static const char *fdeg(char *buf, float deg)
{
	char tmp[16];

	buf[0] = deg > 0.05f ? 'L' : (deg < -0.05f ? 'R' : 'C');
	f1(tmp, absf(deg));
	for (unsigned i = 0U; ; i++) {
		buf[i + 1U] = tmp[i];
		if (tmp[i] == '\0') break;
	}
	return buf;
}

/* Compares two magnitudes and prints which wheel is faster, by how much, and
 * the trim that would equalise them. */
static void report_balance(float l, float r, const char *what)
{
	char b1[16], b2[16], b3[16];
	float al = absf(l), ar = absf(r), diff_pct, slow, fast;
	unsigned trim_l = MOVE_TRIM_L, trim_r = MOVE_TRIM_R;

	if (al < DEAD_MM_S && ar < DEAD_MM_S) {
		tm_printf((UB*)TAG "  NEITHER wheel turned -- below the motors' dead band\n");
		return;
	}
	if (al < DEAD_MM_S || ar < DEAD_MM_S) {
		tm_printf((UB*)TAG "  %s wheel did NOT turn -- dead band, wiring, or driver channel\n",
			  al < DEAD_MM_S ? "LEFT" : "RIGHT");
		return;
	}
	slow = al < ar ? al : ar;
	fast = al < ar ? ar : al;
	diff_pct = (fast - slow) / slow * 100.0f;
	if (al > ar) trim_l = (unsigned)__builtin_lroundf((float)MOVE_TRIM_L * ar / al);
	else trim_r = (unsigned)__builtin_lroundf((float)MOVE_TRIM_R * al / ar);
	if (trim_l < 50U) trim_l = 50U;
	if (trim_r < 50U) trim_r = 50U;

	if (diff_pct < 2.0f)
		tm_printf((UB*)TAG "  %s: L %s  R %s -> MATCHED (%s%% apart)\n",
			  what, f1(b1, al), f1(b2, ar), f1(b3, diff_pct));
	else
		tm_printf((UB*)TAG "  %s: L %s  R %s -> %s is %s%% FASTER; "
			  "try MOVE_TRIM_L=%u MOVE_TRIM_R=%u\n",
			  what, f1(b1, al), f1(b2, ar), al > ar ? "LEFT" : "RIGHT",
			  f1(b3, diff_pct), trim_l, trim_r);
}

static void report(const move_t *m, const move_result_t *res)
{
	char b1[16], b2[16], b3[16], b4[16];
	float h_brake = heading_deg(res->l_mm, res->r_mm);
	float h_fin = heading_deg(res->l_fin, res->r_fin);

	tm_printf((UB*)TAG "  command L %d%%  R %d%%  (fwd +), ran %u ms\n",
		  (int)res->l_cmd, (int)res->r_cmd, (unsigned)res->run_ms);
	tm_printf((UB*)TAG "  travel at brake L %s mm  R %s mm; after coast L %s  R %s\n",
		  f1(b1, res->l_mm), f1(b2, res->r_mm), f1(b3, res->l_fin), f1(b4, res->r_fin));
	if (res->have_speed)
		tm_printf((UB*)TAG "  steady speed L %s mm/s  R %s mm/s\n",
			  f1(b1, res->l_speed), f1(b2, res->r_speed));

	if (m->end == END_TIMED) {
		if (res->have_speed) report_balance(res->l_speed, res->r_speed, "speed mm/s");
		return;
	}

	if (m->left_dir == m->right_dir && m->left_share == m->right_share) {
		/* Straight: any heading change is the two wheels not matching. */
		if (res->have_speed) report_balance(res->l_speed, res->r_speed, "speed mm/s");
		else tm_printf((UB*)TAG "  ended before a steady speed -- no balance verdict\n");
		tm_printf((UB*)TAG "  encoder heading drift %s deg (settled %s). "
			  "Measure the real sideways drift from the tape line.\n",
			  fdeg(b1, h_brake), fdeg(b2, h_fin));
	} else if (m->left_dir != m->right_dir) {
		/* Spin: both wheels should travel the same distance opposite ways. */
		float over = (h_fin - (float)m->target) * (m->target > 0 ? 1.0f : -1.0f);

		tm_printf((UB*)TAG "  encoder heading: braked at %s, settled at %s "
			  "(overshoot %s deg)\n", fdeg(b1, h_brake), fdeg(b2, h_fin), f1(b3, over));
		report_balance(res->l_mm, res->r_mm, "wheel travel mm");
		tm_printf((UB*)TAG "  measure the REAL angle A; effective track = %u * %d / A mm\n",
			  (unsigned)MOVE_TRACK_MM, (int)__builtin_lroundf(absf(h_fin)));
	} else {
		/* Arc: inner/outer ratio should equal MOVE_ARC_INNER. */
		float outer = absf(m->left_share > m->right_share ? res->l_mm : res->r_mm);
		float inner = absf(m->left_share > m->right_share ? res->r_mm : res->l_mm);
		float diff = absf(res->r_mm - res->l_mm);

		tm_printf((UB*)TAG "  encoder heading: braked at %s, settled at %s\n",
			  fdeg(b1, h_brake), fdeg(b2, h_fin));
		if (outer > 1.0f)
			tm_printf((UB*)TAG "  inner/outer travel %s%% (commanded %u%%)\n",
				  f1(b1, inner / outer * 100.0f), (unsigned)MOVE_ARC_INNER);
		if (diff > 1.0f)
			tm_printf((UB*)TAG "  turn radius (robot centre) ~%s mm\n",
				  f1(b1, (float)MOVE_TRACK_MM * (res->l_mm + res->r_mm) / (2.0f * diff)));
	}
}

static const char *status_text(move_status_t s)
{
	switch (s) {
	case MOVE_ABORT_GO: return "stopped by GO button";
	case MOVE_TIMEOUT:  return "timeout: target not reached";
	case MOVE_STALL_L:  return "LEFT wheel stalled: raise MOVE_PWM (or MOVE_ARC_INNER)";
	case MOVE_STALL_R:  return "RIGHT wheel stalled: raise MOVE_PWM (or MOVE_ARC_INNER)";
	case MOVE_WRONG_L:  return "LEFT wheel turned the WRONG WAY: motor or encoder sign";
	case MOVE_WRONG_R:  return "RIGHT wheel turned the WRONG WAY: motor or encoder sign";
	case MOVE_IO:       return "encoder read failed or encoder INVALID";
	default:            return "ok";
	}
}

/* Encoder noise spikes absorbed so far (they do not affect the counts). */
static void report_glitches(void)
{
	encoder_fault_t f;

	if (encoder_source_fault(&f) >= E_OK && f.glitches != 0U)
		tm_printf((UB*)TAG "  encoder glitches ignored so far: %u (noise; counts unaffected)\n",
			  (unsigned)f.glitches);
}

/* Explains why the encoder went invalid (first fault only). */
static void report_encoder_fault(void)
{
	static const char *const reason_text[] = {
		"none recorded (the snapshot read itself failed)",
		"MULTI_EDGE: edges on both A and B in one IRQ (IRQ too late or heavy noise)",
		"BAD_STEP: a pin changed with no edge on it (lost edge)",
		"BAD_LEVEL: edge direction disagrees with the pin level (short glitch)",
	};
	encoder_fault_t f;

	if (encoder_source_fault(&f) < E_OK) {
		tm_printf((UB*)TAG "  encoder fault details unavailable\n");
		return;
	}
	tm_printf((UB*)TAG "  ENCODER FAULT: %s wheel, %s\n",
		  f.wheel == 1U ? "LEFT" : (f.wheel == 2U ? "RIGHT" : "no"),
		  reason_text[f.reason <= ENCODER_FAULT_BAD_LEVEL ? f.reason : 0U]);
	if (f.reason != ENCODER_FAULT_NONE)
		tm_printf((UB*)TAG "  edges%s%s%s%s  AB before %u%u  AB now %u%u  at count %d\n",
			  (f.events & 0x08U) ? " A-rise" : "", (f.events & 0x04U) ? " A-fall" : "",
			  (f.events & 0x80U) ? " B-rise" : "", (f.events & 0x40U) ? " B-fall" : "",
			  (unsigned)((f.previous_state >> 1U) & 1U), (unsigned)(f.previous_state & 1U),
			  (unsigned)((f.current_state >> 1U) & 1U), (unsigned)(f.current_state & 1U),
			  (int)f.count);
}

void motor_moves_hwtest_run(void)
{
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
	s_go_last_pressed = go_pressed_raw();

	tm_printf((UB*)TAG "motor manoeuvre test (OPEN LOOP): PWM %u%%, trim L %u%% R %u%%, "
		  "distance %u mm, track %u mm, arc inner %u%%\n",
		  (unsigned)MOVE_PWM, (unsigned)MOVE_TRIM_L, (unsigned)MOVE_TRIM_R,
		  (unsigned)MOVE_DIST_MM, (unsigned)MOVE_TRACK_MM, (unsigned)MOVE_ARC_INNER);
	tm_printf((UB*)TAG "each GO press runs the next step; GO during a move = STOP\n");
	if (MOVE_SKIP_BALANCE)
		tm_printf((UB*)TAG "MOVE_SKIP_BALANCE=1: skipping the in-air balance sweep\n");

	for (i = MOVE_SKIP_BALANCE ? BALANCE_STEPS : 0U; i < MOVE_COUNT; i++) {
		const move_t *m = &s_moves[i];
		move_result_t res;
		move_status_t st;

		if (m->chain) {
			tm_printf((UB*)TAG "---- step %u/%u: %s ----\n", i + 1U,
				  (unsigned)MOVE_COUNT, m->name);
			tk_dly_tsk(CHAIN_PAUSE_MS);
		} else {
			if (m->setup != NULL)
				tm_printf((UB*)TAG "SETUP: %s\n", m->setup);
			tm_printf((UB*)TAG "---- step %u/%u: %s -- press GO ----\n", i + 1U,
				  (unsigned)MOVE_COUNT, m->name);
			wait_for_go();
			tm_printf((UB*)TAG "moving in %u s, hands clear\n", START_DELAY_MS / 1000U);
			tk_dly_tsk(START_DELAY_MS);
		}
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);

		st = run_move(m, &res);
		(void)motor_emergency_stop();
		report(m, &res);
		report_glitches();
		if (st != MOVE_OK) {
			tm_printf((UB*)TAG "MOVE ENDED: %s -- test stopped, reset to retry\n",
				  status_text(st));
			if (st == MOVE_IO) report_encoder_fault();
			for (;;) tk_dly_tsk(5000U);
		}
	}
	tm_printf((UB*)TAG "==== sequence done; motors stopped. Rebuild with the suggested "
		  "MOVE_TRIM_L/MOVE_TRIM_R and rerun to confirm. ====\n");
	for (;;) tk_dly_tsk(5000U);
}

#endif /* APP_SENSOR_TEST == 13 */
