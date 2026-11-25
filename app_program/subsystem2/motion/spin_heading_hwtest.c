/*
 * spin_heading_hwtest.c - turn-in-place accuracy test (SENSOR_TEST=spin_heading).
 *
 * Step 1 of the spin-scan width measurement. The robot turns on the spot to a
 * sequence of headings using WHEEL ENCODERS ONLY (the IMU has no gyro), stops,
 * and prints the heading the encoders believe it reached. You compare that
 * with the real angle (tape marks / protractor) to find:
 *   - the PWM needed to turn reliably,
 *   - how much it overshoots after braking,
 *   - the effective track width (wheel slip makes it differ from the ruler).
 *
 * Heading (positive = LEFT / counter-clockwise, seen from above):
 *   heading = (right_mm - left_mm) / track_mm      [radians]
 *
 * Safety: nothing moves until the GO button (GP20) is pressed; each press runs
 * one turn; pressing GO during a turn stops the motors and ends the test.
 * Every turn also stops on timeout, stall, wrong-way motion or overtravel.
 */
#include "../../app.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "motor_driver.h"
#include "spin_heading_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 12

/* Effective track from the spin test (ruler 110 mm; wheel contact and slip
 * make it act wider). Calibrated on the mat: heading within ~2 deg/turn. */
#ifndef SPIN_TRACK_MM
#define SPIN_TRACK_MM 116U
#endif
#ifndef SPIN_PWM
#define SPIN_PWM 25U          /* % while far from the target */
#endif
#ifndef SPIN_PWM_SLOW
#define SPIN_PWM_SLOW 20U     /* % for the last SLOW_ZONE_DEG */
#endif

/* Signed % added to the RIGHT wheel's PWM to even out unequal motors. */
#ifndef SPIN_RIGHT_TRIM
#define SPIN_RIGHT_TRIM 0
#endif

_Static_assert(SPIN_TRACK_MM >= 50U && SPIN_TRACK_MM <= 300U, "SPIN_TRACK_MM out of range");
_Static_assert(SPIN_RIGHT_TRIM >= -20 && SPIN_RIGHT_TRIM <= 20, "SPIN_RIGHT_TRIM must be -20..20");
_Static_assert(SPIN_PWM <= 80U && SPIN_PWM_SLOW <= SPIN_PWM && SPIN_PWM_SLOW >= 10U,
	       "need 10 <= SPIN_PWM_SLOW <= SPIN_PWM <= 80");

/* Same calibration as the vehicle (app_main.c, verified with ODO-H01/BAR-H02). */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define LEFT_SIGN          (-1)    /* left encoder counts down going forward */
#define RIGHT_SIGN         1
/* Motor command signs for forward (motion_task.c MOTION_*_POLARITY). */
#define LEFT_MOTOR_FWD     1
#define RIGHT_MOTOR_FWD    (-1)

#define GO_PIN             20U
#define TAG                "[HWTEST][SPIN] "
#define PI_F               3.14159265f
#define LOOP_MS            10U
#define SLOW_ZONE_DEG      15.0f
#define SETTLE_MS          600U    /* after braking, before the final reading */
#define TURN_TIMEOUT_BASE_MS 3000U /* plus TURN_TIMEOUT_MS_PER_DEG per degree */
#define TURN_TIMEOUT_MS_PER_DEG 50U
#define STALL_MS           1000U   /* no progress for this long = stalled */
#define STALL_DEG          0.5f
#define WRONG_WAY_DEG      5.0f
#define LIMIT_DEG          135.0f  /* overtravel guard for the +/-90 steps */
#define START_DELAY_MS     3000U

typedef enum { TURN_OK = 0, TURN_ABORT_GO, TURN_TIMEOUT, TURN_STALL,
	       TURN_WRONG_WAY, TURN_OVERTRAVEL, TURN_IO } turn_result_t;

static bool s_go_last_pressed;

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

/* True once per press (debounced: two samples LOOP_MS apart). */
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
}

/* Heading in degrees, positive = left, from the encoder totals. */
static bool read_heading(float *deg, float *left_mm, float *right_mm)
{
	encoder_snapshot_t s;
	float mm_per_count = WHEEL_CIRC_MM / COUNTS_PER_REV;
	float l, r;

	if (encoder_source_snapshot(&s) < E_OK || !s.valid) return false;
	l = (float)(LEFT_SIGN * s.left_count) * mm_per_count;
	r = (float)(RIGHT_SIGN * s.right_count) * mm_per_count;
	*deg = (r - l) / (float)SPIN_TRACK_MM * (180.0f / PI_F);
	if (left_mm != NULL) *left_mm = l;
	if (right_mm != NULL) *right_mm = r;
	return true;
}

/* dir +1 = turn left (left wheel back, right wheel forward), -1 = right. */
static void drive(int dir, unsigned pwm)
{
	int right_pwm = (int)pwm + SPIN_RIGHT_TRIM;
	int16_t left_wheel, right_wheel;

	if (right_pwm < 0) right_pwm = 0;
	left_wheel = (int16_t)(-dir * (int)pwm);            /* forward-positive */
	right_wheel = (int16_t)(dir * right_pwm);

	/* E_BUSY = reversal brake interval; the next loop retries. */
	(void)motor_set_speed(MOTOR_1, (int16_t)(left_wheel * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(right_wheel * RIGHT_MOTOR_FWD));
}

/* Print a heading as L12.3 / R12.3 using tenths (tm_printf has no %f). */
static void fmt_deg(char *buf, float deg)
{
	int t = (int)__builtin_lroundf(deg * 10.0f);
	char side = t > 0 ? 'L' : (t < 0 ? 'R' : 'C');
	unsigned a = (unsigned)(t < 0 ? -t : t);
	unsigned whole = a / 10U, tenth = a % 10U, i = 0U;
	char digits[8];
	unsigned n = 0U;

	buf[i++] = side;
	do { digits[n++] = (char)('0' + whole % 10U); whole /= 10U; } while (whole && n < 7U);
	while (n) buf[i++] = digits[--n];
	buf[i++] = '.';
	buf[i++] = (char)('0' + tenth);
	buf[i] = '\0';
}

/* *left_mm / *right_mm: each wheel's forward-positive travel during the turn. */
static turn_result_t turn_to(float target, bool guard_limit,
			     float *stop_deg, float *final_deg,
			     float *left_mm, float *right_mm)
{
	SYSTIM t0, now;
	float h, start, progress_h, l0, r0, l1, r1;
	uint32_t progress_ms, elapsed, timeout_ms;
	int dir;
	turn_result_t result = TURN_OK;

	*left_mm = 0.0f;
	*right_mm = 0.0f;
	if (!read_heading(&h, &l0, &r0)) return TURN_IO;
	start = h;
	dir = target > h ? 1 : -1;
	timeout_ms = TURN_TIMEOUT_BASE_MS + TURN_TIMEOUT_MS_PER_DEG *
		     (uint32_t)__builtin_fabsf(target - h);
	progress_h = h;
	tk_get_otm(&t0);
	progress_ms = 0U;

	for (;;) {
		float remaining;

		if (!read_heading(&h, NULL, NULL)) { result = TURN_IO; break; }
		remaining = (target - h) * (float)dir;
		if (remaining <= 0.0f) break;
		tk_get_otm(&now);
		elapsed = now.lo - t0.lo;
		if (go_pressed_raw()) { result = TURN_ABORT_GO; break; }
		if (elapsed > timeout_ms) { result = TURN_TIMEOUT; break; }
		if ((start - h) * (float)dir > WRONG_WAY_DEG) { result = TURN_WRONG_WAY; break; }
		if (guard_limit && (h > LIMIT_DEG || h < -LIMIT_DEG)) {
			result = TURN_OVERTRAVEL;
			break;
		}
		if ((h - progress_h) * (float)dir >= STALL_DEG) {
			progress_h = h;
			progress_ms = elapsed;
		} else if (elapsed - progress_ms > STALL_MS) {
			result = TURN_STALL;
			break;
		}
		drive(dir, remaining > SLOW_ZONE_DEG ? SPIN_PWM : SPIN_PWM_SLOW);
		tk_dly_tsk(LOOP_MS);
	}
	(void)motor_brake_all();
	*stop_deg = h;
	tk_dly_tsk(SETTLE_MS);
	if (read_heading(final_deg, &l1, &r1)) {
		*left_mm = l1 - l0;
		*right_mm = r1 - r0;
	} else if (result == TURN_OK) {
		result = TURN_IO;
	}
	return result;
}

static const char *result_text(turn_result_t r)
{
	switch (r) {
	case TURN_ABORT_GO:   return "stopped by GO button";
	case TURN_TIMEOUT:    return "timeout: robot did not reach the target";
	case TURN_STALL:      return "stalled: wheels not turning, raise SPIN_PWM";
	case TURN_WRONG_WAY:  return "turned the WRONG WAY: motor or encoder sign wrong";
	case TURN_OVERTRAVEL: return "overtravel guard tripped";
	case TURN_IO:         return "encoder read failed or encoder INVALID";
	default:              return "ok";
	}
}

void spin_heading_hwtest_run(void)
{
	/* Absolute targets (deg, + = left); returning to 0 between turns shows drift. */
	static const int16_t steps[] = { 30, 0, -30, 0, 60, 0, -60, 0, 90, 0, -90, 0, 360 };
	unsigned i;
	ER er;
	float prev_final = 0.0f;

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

	tm_printf((UB*)TAG "turn-in-place heading test: encoders only, track %u mm, "
		  "PWM %u%% (slow %u%% in last %u deg), right trim %+d%%\n",
		  (unsigned)SPIN_TRACK_MM, (unsigned)SPIN_PWM, (unsigned)SPIN_PWM_SLOW,
		  (unsigned)SLOW_ZONE_DEG, (int)SPIN_RIGHT_TRIM);
	tm_printf((UB*)TAG "FIRST RUN: put the robot on a box with the wheels OFF the "
		  "ground and do step 1, to check both wheels spin opposite ways\n");
	tm_printf((UB*)TAG "SETUP: robot on the floor with clear space; tape a line along "
		  "its nose for 0 deg, mark L/R 30, 60, 90 deg around it\n");
	tm_printf((UB*)TAG "each GO press runs one turn; GO during a turn = STOP\n");

	for (i = 0U; i < sizeof(steps) / sizeof(steps[0]); i++) {
		char want[12], stop_s[12], fin_s[12], move_s[12];
		float stop_deg, final_deg, moved, turn_dir, left_mm, right_mm;
		int over10;
		turn_result_t r;

		fmt_deg(want, (float)steps[i]);
		tm_printf((UB*)TAG "---- step %u/%u: turn to %s%s -- press GO ----\n",
			  i + 1U, (unsigned)(sizeof(steps) / sizeof(steps[0])), want,
			  steps[i] == 360 ? " (one FULL left turn: robot should end on the tape line)" : "");
		wait_for_go();
		while (go_pressed_raw())            /* a held button would abort the turn */
			tk_dly_tsk(LOOP_MS);
		s_go_last_pressed = false;
		tm_printf((UB*)TAG "turning in %u s, hands clear\n", START_DELAY_MS / 1000U);
		tk_dly_tsk(START_DELAY_MS);
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);

		turn_dir = (float)steps[i] >= prev_final ? 1.0f : -1.0f;
		r = turn_to((float)steps[i], steps[i] != 360, &stop_deg, &final_deg,
			    &left_mm, &right_mm);
		(void)motor_emergency_stop();
		moved = final_deg - prev_final;
		prev_final = final_deg;
		fmt_deg(stop_s, stop_deg);
		fmt_deg(fin_s, final_deg);
		fmt_deg(move_s, moved);
		/* Overshoot past the target in the direction of this turn (+ = too far). */
		over10 = (int)__builtin_lroundf((final_deg - (float)steps[i]) * turn_dir * 10.0f);
		tm_printf((UB*)TAG "target %s: braked at %s, settled at %s (this turn %s, "
			  "overshoot %c%d.%d deg)\n", want, stop_s, fin_s, move_s,
			  over10 < 0 ? '-' : '+', (over10 < 0 ? -over10 : over10) / 10,
			  (over10 < 0 ? -over10 : over10) % 10);
		/* Clean spin: equal and opposite travel. Unequal = the robot drifts
		 * sideways while turning (pivot off the axle midpoint). */
		{
			int l = (int)__builtin_lroundf(left_mm);
			int rr = (int)__builtin_lroundf(right_mm);
			int la = l < 0 ? -l : l, ra = rr < 0 ? -rr : rr;
			int big = la > ra ? la : ra;
			int diff_pct = big > 0 ? (100 * (la > ra ? la - ra : ra - la)) / big : 0;

			tm_printf((UB*)TAG "  wheels: left %+d mm, right %+d mm, difference %d%%%s\n",
				  l, rr, diff_pct,
				  (l > 0) == (rr > 0) && l != 0 && rr != 0 ?
				  "  WHEELS TURNED THE SAME WAY: not spinning in place" :
				  (diff_pct > 10 ? (ra < la ?
				  "  right wheel slower: try SPIN_RIGHT_TRIM=+" :
				  "  right wheel faster: try SPIN_RIGHT_TRIM=-") : "  ok"));
		}
		if (r != TURN_OK) {
			tm_printf((UB*)TAG "TURN ENDED: %s -- test stopped, reset to retry\n",
				  result_text(r));
			for (;;) tk_dly_tsk(5000U);
		}
		tm_printf((UB*)TAG "  measure the REAL heading now. If this turn was really "
			  "A deg: track = %u * %d / A mm\n", (unsigned)SPIN_TRACK_MM,
			  (int)__builtin_lroundf(moved < 0.0f ? -moved : moved));
	}
	tm_printf((UB*)TAG "==== sequence done; motors stopped. Report real angles per step. ====\n");
	for (;;) tk_dly_tsk(5000U);
}

#endif /* APP_SENSOR_TEST == 12 */
