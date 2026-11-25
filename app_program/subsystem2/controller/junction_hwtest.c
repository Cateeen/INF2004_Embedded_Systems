/*
 * junction_hwtest.c - junction detection and action test (SENSOR_TEST=junction).
 *
 * Follows the line exactly as the vehicle does (line_steering with the
 * line_steering.h settings, wheel PID with feed-forward from app_main.c) and
 * feeds every 10 ms sample of all three IR sensors, plus the distance from
 * the encoders, to junction_detector. A junction is the LEFT (barcode) sensor
 * black for about a tape width, then the centre AND right line sensors both
 * black ~8 mm later (the left sensor's spot leads them by ~8 mm, measured;
 * 18 mm tape reads ~12 mm wide to these sensors). To simulate
 * a "+" without one, lay a straight piece of tape ACROSS the line.
 *
 * Every left-sensor event is printed as it is decided, with the measured
 * widths, so the thresholds can be tuned:
 *   JUNCTION       accepted
 *   narrow         left black under JN_LEFT_MIN_MM (barcode bar / noise)
 *   too wide       left black over JN_LEFT_MAX_MM
 *   unconfirmed    left width fine, line sensors never both black in time
 *
 * JN_ACTION, on the first junction:
 *   stop      stop there (check where it stopped)
 *   straight  carry on over it; every junction is counted
 *   left      drive JN_FORWARD_MM (wheels over the crossing), spin 90 deg
 *             (right turns drive JN_FORWARD_RIGHT_MM instead)
 *   right       on the spot by the encoders, then follow the new line for
 *   uturn       JN_AFTER_MM (uturn spins 180 deg, to the left; in a no-line
 *               start there is no line behind, so it spins a fixed 180 and
 *               drives back to cross the same tape again)
 *
 * NO-LINE START: if the line sensors are white at GO (no line, just a
 * crossing tape on the mat), the robot drives straight instead of following,
 * up to JN_BLIND_MM, to find the crossing. If the left sensor starts ON the
 * tape it first backs off until it is just past the tape's edge, so the
 * forward pass measures the whole tape width. After a left/right turn it
 * follows the crossing tape as the new line.
 *
 * Each GO press runs once (until a stop, JN_RUN_MM, a lost line, or GO
 * again = STOP), prints a report, and waits for the next GO. While waiting
 * it shows the live readings of all three sensors.
 *
 * Safety: nothing moves until GO (GP20); GO during a run stops the motors.
 * Runs also stop on timeout, a stalled wheel, a wheel turning the wrong way,
 * or an encoder fault.
 *
 * LF_STRADDLE=1 (test-only, see straddle_hwtest.h): for the layout with the
 * two line sensors further apart than the tape and the left/barcode sensor
 * moved behind them. Following uses the straddle steering instead of
 * line_steering, and a junction is the centre AND right sensors black
 * together for JN_BOTH_MIN_MM (they can only both see tape at a crossing).
 * The left sensor and the JN_LEFT_MIN_MM, JN_LEFT_MAX_MM, JN_OFFSET_MM and
 * JN_TOL_MM settings are not used. Both white is the normal "centred"
 * reading, so the test cannot tell "no line" by itself: by default every run
 * follows the line. JN_NO_LINE=1 selects the NO-LINE START instead (for a
 * single tape laid across the mat, no "+" track): drive straight up to
 * JN_BLIND_MM until centre+right are both black, then do JN_ACTION -- a
 * left/right turn follows the crossing tape as the new line, a U-turn spins
 * a fixed 180 and drives back across the same tape. If both line sensors
 * start ON the tape it first backs off it.
 */
#include "../../app.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "../../subsystem3/line_barcode/line_adc_reader.h"
#include "../../subsystem3/line_barcode/line_observation.h"
#include "../../subsystem3/line_barcode/junction_detector.h"
#include "../motion/motor_driver.h"
#include "../motion/wheel_speed_control.h"
#include "line_steering.h"
#include "straddle_hwtest.h"
#include "junction_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 18

/* 0 stop, 1 straight, 2 left, 3 right, 4 uturn */
#ifndef JN_ACTION
#define JN_ACTION 0
#endif
#ifndef JN_FORWARD_MM
#define JN_FORWARD_MM 100U    /* after detection, before a left turn / U-turn; measured */
#endif
#ifndef JN_FORWARD_RIGHT_MM
#define JN_FORWARD_RIGHT_MM 120U  /* before a right turn (see below); measured */
#endif
#ifndef JN_SEEK
#define JN_SEEK 1             /* 1: turn until the line is found; 0: fixed angle */
#endif
#ifndef JN_SEEK_BEFORE_DEG
#define JN_SEEK_BEFORE_DEG 30 /* start looking this far before the target angle */
#endif
#ifndef JN_SEEK_AFTER_DEG
#define JN_SEEK_AFTER_DEG 30  /* give up this far past it */
#endif
_Static_assert(JN_SEEK == 0 || JN_SEEK == 1, "JN_SEEK must be 0 or 1");
_Static_assert(JN_SEEK_BEFORE_DEG >= 0 && JN_SEEK_BEFORE_DEG <= 80 &&
	       JN_SEEK_AFTER_DEG >= 0 && JN_SEEK_AFTER_DEG <= 80,
	       "JN_SEEK_BEFORE_DEG and JN_SEEK_AFTER_DEG must be 0..80");
#ifndef JN_AFTER_MM
#define JN_AFTER_MM 400U      /* line following after a turn */
#endif
#ifndef JN_RUN_MM
#define JN_RUN_MM 1500U       /* longest run before the junction */
#endif
#ifndef JN_TURN_TRIM_DEG
#define JN_TURN_TRIM_DEG 4    /* brake this early (spin_heading: ~3.5 deg overshoot) */
#endif
#ifndef JN_TRACE
#define JN_TRACE 1U
#endif
#ifndef JN_BLIND_MM
#define JN_BLIND_MM 200U      /* no-line start: straight drive to find the crossing */
#endif
_Static_assert(JN_BLIND_MM >= 50U && JN_BLIND_MM <= 1000U, "need 50 <= JN_BLIND_MM <= 1000");
/* Straddle seek: stop the spin when the tape is IN THE GAP between the line
 * sensors (the leading sensor went black, then white again), not as soon as
 * the leading sensor touches it -- that left the robot a few degrees short,
 * with the tape still under the sensor. 0 off, 1 left spins only (left turn
 * and U-turn), 2 all spins. Stops anyway if the far sensor goes black. */
#ifndef JN_SEEK_GAP
#define JN_SEEK_GAP 1
#endif
_Static_assert(JN_SEEK_GAP >= 0 && JN_SEEK_GAP <= 2, "JN_SEEK_GAP must be 0, 1 or 2");
#ifndef JN_NO_LINE
#define JN_NO_LINE 0          /* straddle: 1 = no-line start (single tape across) */
#endif
_Static_assert(JN_NO_LINE == 0 || JN_NO_LINE == 1, "JN_NO_LINE must be 0 or 1");
#ifndef JN_LEFT_MIN_MM
#define JN_LEFT_MIN_MM 8
#endif
#ifndef JN_LEFT_MAX_MM
#define JN_LEFT_MAX_MM 80
#endif
#ifndef JN_OFFSET_MM
#define JN_OFFSET_MM 8
#endif
#ifndef JN_TOL_MM
#define JN_TOL_MM 12
#endif
#ifndef JN_BOTH_MIN_MM
#define JN_BOTH_MIN_MM 5
#endif

_Static_assert(JN_ACTION >= 0 && JN_ACTION <= 4, "JN_ACTION must be 0..4");
_Static_assert(JN_FORWARD_MM <= 200U && JN_FORWARD_RIGHT_MM <= 200U,
	       "need JN_FORWARD_MM, JN_FORWARD_RIGHT_MM <= 200");
/* Forward drive before the spin. Right turns differ from left ones: the
 * tracked tape edge is off the robot's centreline, which shifts the sensors
 * one way after a left spin and the other way after a right spin. */
#define JN_FORWARD_ACTION_MM (JN_ACTION == 3 ? JN_FORWARD_RIGHT_MM : JN_FORWARD_MM)
_Static_assert(JN_AFTER_MM >= 50U && JN_AFTER_MM <= 3000U, "need 50 <= JN_AFTER_MM <= 3000");
_Static_assert(JN_RUN_MM >= 200U && JN_RUN_MM <= 5000U, "need 200 <= JN_RUN_MM <= 5000");
_Static_assert(JN_TURN_TRIM_DEG >= 0 && JN_TURN_TRIM_DEG <= 20, "need 0 <= JN_TURN_TRIM_DEG <= 20");
_Static_assert(JN_TRACE <= 1U, "JN_TRACE must be 0 or 1");

#if LF_STRADDLE
#define SPEED              LF_SL_SPEED     /* = the vehicle's unless overridden */
#else
#define SPEED              LINE_STEERING_SPEED_CM_S
#endif

/* Wheel speed PID: same as motion_config in app_main.c. */
#define PID_KP             2.0f
#define PID_KI             4.0f
#define PID_KD             0.0f
#define PID_BASE           22U
#define PID_MAX            60U
#define PID_FF             1.4f
#define PID_FF_REF_CM_S    15.0f
#define PID_PERIOD_MS      50U
#define STEER_MS           50U     /* = controller period */

/* Same calibration as the vehicle (app_main.c) and spin_heading. */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define TRACK_MM           116.0f
#define LEFT_SIGN          (-1)
#define RIGHT_SIGN         1
#define LEFT_MOTOR_FWD     1
#define RIGHT_MOTOR_FWD    (-1)
#define SPIN_PWM           25      /* spin_heading: 25 % far, 20 % for the last 15 deg */
#define SPIN_PWM_SLOW      20
#define SLOW_ZONE_DEG      15.0f

#define GO_PIN             20U
#define TAG                "[HWTEST][JN] "
#define PI_F               3.14159265f
#define POLL_MS            10U
#define START_DELAY_MS     2000U
#define SHOW_MS            500U
#define SETTLE_MS          400U
#define STALL_MS           1000U
#define STALL_MM           2.0f
#define WRONG_WAY_MM       20.0f
#define MAX_EVENTS         16U
#define MIN_WHEEL_CM_S     0.5f    /* PID needs targets > 0: "stopped" inner wheel */
#define STALL_EXEMPT_CM_S  3.0f    /* a wheel asked for less is not stall-checked */

#define BACKOFF_PWM        20      /* % reversing off the tape (no-line start) */
#define BACKOFF_PAST_MM    8.0f    /* stop this far past the tape's edge */
#define BACKOFF_MAX_MM     80.0f

typedef enum { END_DIST = 0, END_JUNCTION, END_LOST, END_NOT_ON_LINE, END_GO, END_TIMEOUT,
	       END_STALL, END_WRONG_WAY, END_ENCODER, END_SENSOR, END_PID,
	       END_BACKOFF, END_SEEK_FAIL } end_t;

typedef struct {
	junction_event_t event;
	float at_mm, left_mm, both_mm;
} event_log_t;

static bool s_go_last_pressed;
static wheel_speed_control_t s_pid;
static wheel_speed_output_t s_out;
static junction_detector_t s_det;
static event_log_t s_events[MAX_EVENTS];
static unsigned s_event_count;
static unsigned s_kind_count[5];
static float s_dist0_l, s_dist0_r;     /* encoder zero for the run's distance */

static const char *const s_pos_name[4] = {
	[POS_JUNCTION] = "CENTRE", [POS_LEFT] = "LEFT", [POS_RIGHT] = "RIGHT",
	[POS_LOST] = "LOST"
};
static const char *const s_event_name[5] = {
	"-", "JUNCTION",
	LF_STRADDLE ? "narrow (both black too short)" : "narrow (barcode bar?)",
	"too wide", "unconfirmed"
};
#if LF_STRADDLE
static straddle_junction_t s_sj;
#endif
static const char *const s_action_name[5] = { "stop", "straight", "left", "right", "uturn" };

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

static bool go_pressed_raw(void) { return gpio_get_val(GO_PIN) == 0U; }

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

static uint32_t now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
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

/* One sample to the junction detector in use; straddle events are mapped to
 * the junction_detector ones so the logging and report are shared. */
static junction_event_t detect(const ir_readings_t *ir, float d)
{
#if LF_STRADDLE
	switch (sj_update(&s_sj, ir->center.black, ir->right.black, d)) {
	case SJ_DETECTED: return JUNCTION_DETECTED;
	case SJ_NARROW:   return JUNCTION_REJECT_NARROW;
	default:          return JUNCTION_NONE;
	}
#else
	return junction_detector_update(&s_det, ir->barcode.black, ir->center.black,
					ir->right.black, d);
#endif
}

static void detector_reset(void)
{
#if LF_STRADDLE
	sj_reset(&s_sj, (float)JN_BOTH_MIN_MM);
#else
	junction_detector_reset(&s_det);
#endif
}

static bool read_ir(line_adc_reader_t *reader, ir_readings_t *ir, uint8_t *pos)
{
	uint32_t ts;

	if (line_adc_reader_read(reader, ir, &ts) != E_OK) return false;
#if LF_STRADDLE
	sl_hysteresis(ir);
#endif
	*pos = line_following_position(ir);
	return true;
}

static bool read_mm(float *l, float *r)
{
	encoder_snapshot_t s;
	const float mm_per_count = WHEEL_CIRC_MM / COUNTS_PER_REV;

	if (encoder_source_snapshot(&s) < E_OK || !s.valid) return false;
	*l = (float)(LEFT_SIGN * s.left_count) * mm_per_count;
	*r = (float)(RIGHT_SIGN * s.right_count) * mm_per_count;
	return true;
}

static void drive_forward(uint8_t pwm_l, uint8_t pwm_r)
{
	(void)motor_set_speed(MOTOR_1, (int16_t)((int)pwm_l * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)((int)pwm_r * RIGHT_MOTOR_FWD));
}

/* dir +1 = spin left (left wheel back, right forward), -1 = right. */
static void drive_spin(int dir, int pwm)
{
	(void)motor_set_speed(MOTOR_1, (int16_t)(-dir * pwm * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(dir * pwm * RIGHT_MOTOR_FWD));
}

static void wait_for_go(line_adc_reader_t *reader)
{
	uint32_t last_show = now_ms() - SHOW_MS;

	while (!go_edge()) {
		ir_readings_t ir;
		uint8_t pos;

		if ((uint32_t)(now_ms() - last_show) >= SHOW_MS) {
			last_show = now_ms();
			if (!read_ir(reader, &ir, &pos)) fail_forever("IR read", E_IO);
			tm_printf((UB*)TAG "  IR left %4u %s  centre %4u %s  right %4u %s  -> %s\n",
				  (unsigned)ir.barcode.raw, ir.barcode.black ? "BLK" : "wht",
				  (unsigned)ir.center.raw, ir.center.black ? "BLK" : "wht",
				  (unsigned)ir.right.raw, ir.right.black ? "BLK" : "wht",
				  s_pos_name[pos & 3U]);
		}
		tk_dly_tsk(POLL_MS);
	}
	while (go_pressed_raw())
		tk_dly_tsk(POLL_MS);
	s_go_last_pressed = false;
}

static void log_event(junction_event_t e, float at_mm)
{
	char b1[16], b2[16], b3[16];
	float left_mm, both_mm;

#if LF_STRADDLE
	left_mm = 0.0f;
	both_mm = s_sj.both_mm;
#else
	junction_detector_last(&s_det, &left_mm, &both_mm);
#endif
	s_kind_count[e]++;
	if (s_event_count < MAX_EVENTS)
		s_events[s_event_count++] = (event_log_t){ e, at_mm, left_mm, both_mm };
	if (LF_STRADDLE)
		tm_printf((UB*)TAG "  >> %s at %s mm: centre+right both black %s mm\n",
			  s_event_name[e], f1(b1, at_mm), f1(b3, both_mm));
	else
		tm_printf((UB*)TAG "  >> %s at %s mm: left black %s mm, centre+right black %s mm "
			  "in window\n", s_event_name[e], f1(b1, at_mm), f1(b2, left_mm),
			  f1(b3, both_mm));
}

/* Distance travelled this run (mean of both wheels, forward-positive). */
static float run_distance(float l, float r)
{
	return ((l - s_dist0_l) + (r - s_dist0_r)) * 0.5f;
}

/* Wheel targets from a speed/turn command; a wheel asked to stop or reverse
 * (straddle pivot) is held at MIN_WHEEL_CM_S: the PID only drives forward. */
static bool set_targets(line_steering_command_t c, float *left, float *right)
{
	*left = (float)(c.linear_cm_s - c.turn_cm_s);
	*right = (float)(c.linear_cm_s + c.turn_cm_s);
	if (*left < MIN_WHEEL_CM_S) *left = MIN_WHEEL_CM_S;
	if (*right < MIN_WHEEL_CM_S) *right = MIN_WHEEL_CM_S;
	return wheel_speed_control_set_targets(&s_pid, *left, *right);
}

/*
 * Follow the line for up to max_mm (this leg). stop_at_junction: return
 * END_JUNCTION on the first one, motors still running (the caller decides).
 * *junctions counts every accepted junction.
 */
static end_t follow(line_adc_reader_t *reader, float max_mm, bool stop_at_junction,
		    unsigned *junctions)
{
	const uint32_t timeout_ms =
		4000U + (uint32_t)(2.0f * max_mm / ((float)SPEED * 10.0f) * 1000.0f);
	line_steering_t steering;
#if LF_STRADDLE
	straddle_t straddle;
#endif
	line_steering_command_t cmd;
	ir_readings_t ir;
	uint8_t pos;
	float l, r, l0, r0, pl, pr, prog_l = 0.0f, prog_r = 0.0f, sl = 0.0f, sr = 0.0f;
	float tgt_l = (float)SPEED, tgt_r = (float)SPEED;
	uint32_t t0, t, next_steer, next_pid, last_pid, prog_l_ms = 0U, prog_r_ms = 0U;
	char b1[16], b2[16], b3[16];

	if (!read_ir(reader, &ir, &pos)) return END_SENSOR;
	if (!LF_STRADDLE && pos == POS_LOST) return END_NOT_ON_LINE;
	if (!read_mm(&l0, &r0)) return END_ENCODER;
	pl = l0;
	pr = r0;
	line_steering_reset(&steering);
	t0 = now_ms();
#if LF_STRADDLE
	sl_reset(&straddle, SPEED);
	cmd = sl_update(&straddle, pos, t0);
#else
	cmd = line_steering_update(&steering, pos, t0);
#endif
	if (cmd.stop_latched || !wheel_speed_control_reset(&s_pid) ||
	    !set_targets(cmd, &tgt_l, &tgt_r))
		return END_PID;
	s_out = (wheel_speed_output_t){ PID_BASE, PID_BASE };
	next_steer = t0 + STEER_MS;
	last_pid = t0;
	next_pid = t0 + PID_PERIOD_MS;
	drive_forward(s_out.left_pwm_percent, s_out.right_pwm_percent);

	for (;;) {
		junction_event_t e;
		float d, leg;

		tk_dly_tsk(POLL_MS);
		t = now_ms();
		if (!read_mm(&l, &r)) return END_ENCODER;
		if (!read_ir(reader, &ir, &pos)) return END_SENSOR;
		d = run_distance(l, r);
		leg = ((l - l0) + (r - r0)) * 0.5f;

		e = detect(&ir, d);
		if (e != JUNCTION_NONE) {
			log_event(e, d);
			if (e == JUNCTION_DETECTED) {
				(*junctions)++;
				if (stop_at_junction) return END_JUNCTION;
			}
		}

		if (go_pressed_raw()) return END_GO;
		if (t - t0 > timeout_ms) return END_TIMEOUT;
		if (l - l0 < -WRONG_WAY_MM || r - r0 < -WRONG_WAY_MM) return END_WRONG_WAY;
		if (absf((l - l0) - prog_l) >= STALL_MM) { prog_l = l - l0; prog_l_ms = t - t0; }
		if (absf((r - r0) - prog_r) >= STALL_MM) { prog_r = r - r0; prog_r_ms = t - t0; }
		/* The inner wheel of a straddle pivot is meant to stand still. */
		if (tgt_l < STALL_EXEMPT_CM_S) prog_l_ms = t - t0;
		if (tgt_r < STALL_EXEMPT_CM_S) prog_r_ms = t - t0;
		if (t - t0 - prog_l_ms > STALL_MS || t - t0 - prog_r_ms > STALL_MS)
			return END_STALL;
		if (leg >= max_mm) return END_DIST;

		if ((int32_t)(t - next_steer) >= 0) {
			next_steer += STEER_MS;
#if LF_STRADDLE
			cmd = sl_update(&straddle, pos, t);
#else
			cmd = line_steering_update(&steering, pos, t);
#endif
			if (cmd.stop_latched) return END_LOST;
			if (!set_targets(cmd, &tgt_l, &tgt_r)) return END_PID;
			if (JN_TRACE) {
#if LF_STRADDLE
				const char *state_name = sl_state_name(straddle.state);
#else
				const char *state_name = "";
#endif
				tm_printf((UB*)TAG "  %5u %-6s %+3d  L %4u C %4u R %4u  %6s mm  "
					  "%s/%s cm/s  %s\n", (unsigned)(t - t0),
					  s_pos_name[pos & 3U], (int)cmd.turn_cm_s,
					  (unsigned)ir.barcode.raw, (unsigned)ir.center.raw,
					  (unsigned)ir.right.raw, f1(b1, d), f1(b2, sl), f1(b3, sr),
					  state_name);
			}
		}
		if ((int32_t)(t - next_pid) >= 0) {
			float dt = (float)(t - last_pid) / 1000.0f;

			next_pid += PID_PERIOD_MS;
			sl = (l - pl) / 10.0f / dt;
			sr = (r - pr) / 10.0f / dt;
			pl = l;
			pr = r;
			last_pid = t;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (!wheel_speed_control_update(&s_pid, sl, sr, dt, &s_out)) return END_PID;
		}
		drive_forward(s_out.left_pwm_percent, s_out.right_pwm_percent);
	}
}

/*
 * Straight ahead at SPEED for mm (PID keeps both wheels equal), no steering.
 * from_rest: reset the PID and start from the base PWM (else carry on from
 * the current motion). reader != NULL: also run the junction detector and
 * the trace; with stop_at_junction, return END_JUNCTION on the first one.
 */
static end_t drive_straight(line_adc_reader_t *reader, float mm, bool from_rest,
			    bool stop_at_junction, unsigned *junctions)
{
	const uint32_t timeout_ms =
		3000U + (uint32_t)(2.0f * mm / ((float)SPEED * 10.0f) * 1000.0f);
	float l, r, l0, r0, pl, pr, sl = 0.0f, sr = 0.0f, prog = 0.0f;
	uint32_t t0, t, last_pid, next_pid, next_trace, prog_ms = 0U;
	char b1[16], b2[16], b3[16];

	if (mm <= 0.0f) return END_DIST;
	if (!read_mm(&l0, &r0)) return END_ENCODER;
	if (from_rest) {
		if (!wheel_speed_control_reset(&s_pid)) return END_PID;
		s_out = (wheel_speed_output_t){ PID_BASE, PID_BASE };
	}
	if (!wheel_speed_control_set_targets(&s_pid, (float)SPEED, (float)SPEED)) return END_PID;
	pl = l0;
	pr = r0;
	t0 = last_pid = now_ms();
	next_pid = t0 + PID_PERIOD_MS;
	next_trace = t0;
	drive_forward(s_out.left_pwm_percent, s_out.right_pwm_percent);
	for (;;) {
		float moved;

		tk_dly_tsk(POLL_MS);
		t = now_ms();
		if (!read_mm(&l, &r)) return END_ENCODER;
		moved = ((l - l0) + (r - r0)) * 0.5f;
		if (reader != NULL) {
			ir_readings_t ir;
			uint8_t pos;
			junction_event_t e;
			float d;

			if (!read_ir(reader, &ir, &pos)) return END_SENSOR;
			d = run_distance(l, r);
			e = detect(&ir, d);
			if (e != JUNCTION_NONE) {
				log_event(e, d);
				if (e == JUNCTION_DETECTED) {
					(*junctions)++;
					if (stop_at_junction) return END_JUNCTION;
				}
			}
			if (JN_TRACE && (int32_t)(t - next_trace) >= 0) {
				next_trace += STEER_MS;
				tm_printf((UB*)TAG "  %5u %-6s  --  L %4u C %4u R %4u  %6s mm  "
					  "%s/%s cm/s\n", (unsigned)(t - t0), s_pos_name[pos & 3U],
					  (unsigned)ir.barcode.raw, (unsigned)ir.center.raw,
					  (unsigned)ir.right.raw, f1(b1, d), f1(b2, sl), f1(b3, sr));
			}
		}
		if (go_pressed_raw()) return END_GO;
		if (t - t0 > timeout_ms) return END_TIMEOUT;
		if (moved >= mm) return END_DIST;
		if (moved - prog >= STALL_MM) { prog = moved; prog_ms = t - t0; }
		else if (t - t0 - prog_ms > STALL_MS) return END_STALL;
		if ((int32_t)(t - next_pid) >= 0) {
			float dt = (float)(t - last_pid) / 1000.0f;

			next_pid += PID_PERIOD_MS;
			sl = (l - pl) / 10.0f / dt;
			sr = (r - pr) / 10.0f / dt;
			pl = l;
			pr = r;
			last_pid = t;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (!wheel_speed_control_update(&s_pid, sl, sr, dt, &s_out)) return END_PID;
		}
		drive_forward(s_out.left_pwm_percent, s_out.right_pwm_percent);
	}
}

/*
 * Started with the left sensor already on the crossing tape: back up slowly
 * until it is BACKOFF_PAST_MM past the tape's edge, so the forward pass sees
 * the whole tape width (starting part-way across would measure too little).
 */
static end_t back_off_tape(line_adc_reader_t *reader)
{
	float l0, r0, l, r, back = 0.0f, white_at = -1.0f, prog = 0.0f;
	uint32_t t0, t, prog_ms = 0U;
	end_t end = END_DIST;
	char b1[16];

	if (!read_mm(&l0, &r0)) return END_ENCODER;
	t0 = now_ms();
	for (;;) {
		ir_readings_t ir;
		uint8_t pos;

		/* E_BUSY = reversal brake interval; the next loop retries. */
		(void)motor_set_speed(MOTOR_1, (int16_t)(-BACKOFF_PWM * LEFT_MOTOR_FWD));
		(void)motor_set_speed(MOTOR_2, (int16_t)(-BACKOFF_PWM * RIGHT_MOTOR_FWD));
		tk_dly_tsk(POLL_MS);
		t = now_ms();
		if (!read_mm(&l, &r)) { end = END_ENCODER; break; }
		if (!read_ir(reader, &ir, &pos)) { end = END_SENSOR; break; }
		back = -((l - l0) + (r - r0)) * 0.5f;
		if (white_at < 0.0f &&
		    (LF_STRADDLE ? !ir.center.black && !ir.right.black : !ir.barcode.black))
			white_at = back;
		if (white_at >= 0.0f && back - white_at >= BACKOFF_PAST_MM) break;
		if (go_pressed_raw()) { end = END_GO; break; }
		if (back >= BACKOFF_MAX_MM) { end = END_BACKOFF; break; }
		if (t - t0 > 4000U) { end = END_TIMEOUT; break; }
		if (back - prog >= STALL_MM) { prog = back; prog_ms = t - t0; }
		else if (t - t0 - prog_ms > STALL_MS) { end = END_STALL; break; }
	}
	(void)motor_brake_all();
	tk_dly_tsk(SETTLE_MS);
	if (end == END_DIST)
		tm_printf((UB*)TAG "  backed off %s mm: %s now just before the tape\n",
			  f1(b1, back), LF_STRADDLE ? "line sensors" : "left sensor");
	return end;
}

/*
 * Spin on the spot by deg (+ left) using the encoders; returns the heading
 * change after settling in *done_deg.
 *
 * reader != NULL (JN_SEEK): turn until the line is found instead of a fixed
 * angle -- fast to deg - JN_SEEK_BEFORE_DEG (the sensors cannot reach the
 * new line before that), then slowly until the centre or right line sensor
 * reads black, giving up at deg + JN_SEEK_AFTER_DEG (END_SEEK_FAIL). The
 * exact angle, wheel slip and small position errors then no longer decide
 * whether the sensors land on the line; a small sideways error just leaves
 * the robot at a slight angle to it, which line following corrects.
 */
static end_t spin(line_adc_reader_t *reader, float deg, float *done_deg)
{
	const int dir = deg > 0.0f ? 1 : -1;
	const float target = absf(deg) - (float)JN_TURN_TRIM_DEG;
	const float seek_from = absf(deg) - (float)JN_SEEK_BEFORE_DEG;
	const float seek_to = absf(deg) + (float)JN_SEEK_AFTER_DEG;
	float l0, r0, l, r, h = 0.0f, progress = 0.0f;
	uint32_t t0, t, progress_ms = 0U;
	end_t end = END_DIST;
	/* Leading sensor for this spin: the centre (left) one spinning left. */
	const bool gap_seek = LF_STRADDLE && (JN_SEEK_GAP == 2 || (JN_SEEK_GAP == 1 && dir > 0));
	bool lead_seen = false;

	*done_deg = 0.0f;
	if (!read_mm(&l0, &r0)) return END_ENCODER;
	t0 = now_ms();
	for (;;) {
		float remaining;

		if (!read_mm(&l, &r)) { end = END_ENCODER; break; }
		h = ((r - r0) - (l - l0)) / TRACK_MM * (180.0f / PI_F) * (float)dir;
		if (reader != NULL) {
			if (h >= seek_from) {
				ir_readings_t ir;
				uint8_t pos;

				if (!read_ir(reader, &ir, &pos)) { end = END_SENSOR; break; }
				if (gap_seek) {
					bool lead = dir > 0 ? ir.center.black : ir.right.black;
					bool far = dir > 0 ? ir.right.black : ir.center.black;

					if (far) break;                 /* past the gap: stop */
					if (lead) lead_seen = true;
					else if (lead_seen) break;      /* tape now in the gap */
				} else if (ir.center.black || ir.right.black) {
					break;                          /* found it */
				}
				if (h >= seek_to) { end = END_SEEK_FAIL; break; }
				remaining = 0.0f;                              /* slow */
			} else {
				remaining = seek_from - h;
			}
		} else {
			remaining = target - h;
			if (remaining <= 0.0f) break;
		}
		t = now_ms();
		if (go_pressed_raw()) { end = END_GO; break; }
		if (t - t0 > 4000U) { end = END_TIMEOUT; break; }
		if (h < -10.0f) { end = END_WRONG_WAY; break; }
		if (h - progress >= 1.0f) { progress = h; progress_ms = t - t0; }
		else if (t - t0 - progress_ms > STALL_MS) { end = END_STALL; break; }
		drive_spin(dir, remaining > SLOW_ZONE_DEG ? SPIN_PWM : SPIN_PWM_SLOW);
		tk_dly_tsk(POLL_MS);
	}
	(void)motor_brake_all();
	tk_dly_tsk(SETTLE_MS);
	if (read_mm(&l, &r))
		*done_deg = ((r - r0) - (l - l0)) / TRACK_MM * (180.0f / PI_F);
	return end;
}

static const char *end_text(end_t e)
{
	switch (e) {
	case END_DIST:        return "distance reached";
	case END_JUNCTION:    return "stopped at the junction";
	case END_LOST:        return LF_STRADDLE ?
				       "LINE LOST (straddle pivot/search found nothing)" :
				       "LINE LOST > 300 ms";
	case END_NOT_ON_LINE: return "NOT ON THE LINE (both line sensors white)";
	case END_GO:          return "stopped by GO button";
	case END_TIMEOUT:     return "timeout";
	case END_STALL:       return "a wheel STALLED";
	case END_WRONG_WAY:   return "a wheel turned the WRONG WAY";
	case END_ENCODER:     return "encoder INVALID";
	case END_SENSOR:      return "IR ADC read failed";
	case END_PID:         return "PID rejected its inputs";
	case END_BACKOFF:     return LF_STRADDLE ?
				       "a line sensor still black after backing off 80 mm" :
				       "left sensor still black after backing off 80 mm";
	case END_SEEK_FAIL:   return "turned past the target and the line sensors never found the line";
	default:              return "?";
	}
}

static bool fatal(end_t e)
{
	return e == END_STALL || e == END_WRONG_WAY || e == END_ENCODER || e == END_SENSOR ||
	       e == END_PID;
}

static void report(const junction_config_t *cfg, unsigned junctions)
{
	char b1[16], b2[16], b3[16];
	unsigned i;

#if LF_STRADDLE
	(void)cfg;
	(void)b2;
	tm_printf((UB*)TAG "  both-black events: %u junction, %u narrow\n",
		  s_kind_count[JUNCTION_DETECTED], s_kind_count[JUNCTION_REJECT_NARROW]);
	for (i = 0U; i < s_event_count; i++)
		tm_printf((UB*)TAG "    %2u. %-30s at %6s mm  both %5s mm\n", i + 1U,
			  s_event_name[s_events[i].event], f1(b1, s_events[i].at_mm),
			  f1(b3, s_events[i].both_mm));
	if (junctions == 0U && s_kind_count[JUNCTION_REJECT_NARROW] != 0U)
		tm_printf((UB*)TAG "  HINT: centre+right were black together but for less than "
			  "%u mm. If that was the crossing, lower JN_BOTH_MIN_MM, or raise JN_GAP_MM "
			  "(now %u) if the trace shows them flickering.\n",
			  (unsigned)JN_BOTH_MIN_MM, (unsigned)JN_GAP_MM);
	if (junctions == 0U && s_event_count == 0U)
		tm_printf((UB*)TAG "  HINT: centre and right were never black together. Over the "
			  "crossing tape both must read > %u; check the trace C/R columns and that "
			  "the crossing tape spans both sensors.\n",
			  (unsigned)LINE_ADC_LINE_BLACK_THRESHOLD);
	return;
#endif

	tm_printf((UB*)TAG "  left-sensor events: %u junction, %u narrow, %u too wide, "
		  "%u unconfirmed\n", s_kind_count[JUNCTION_DETECTED],
		  s_kind_count[JUNCTION_REJECT_NARROW], s_kind_count[JUNCTION_REJECT_WIDE],
		  s_kind_count[JUNCTION_REJECT_UNCONFIRMED]);
	for (i = 0U; i < s_event_count; i++)
		tm_printf((UB*)TAG "    %2u. %-22s at %6s mm  left %5s mm  both %5s mm\n", i + 1U,
			  s_event_name[s_events[i].event], f1(b1, s_events[i].at_mm),
			  f1(b2, s_events[i].left_mm), f1(b3, s_events[i].both_mm));
	if (junctions == 0U && s_kind_count[JUNCTION_REJECT_UNCONFIRMED] != 0U)
		tm_printf((UB*)TAG "  HINT: the left sensor saw a tape-width stripe but centre+right "
			  "were not both black (> %u) inside the window %s..(%s + width + %s) mm "
			  "after it. Check the trace: raise JN_TOL_MM, change JN_OFFSET_MM, or "
			  "lower JN_BOTH_MIN_MM.\n", (unsigned)LINE_ADC_LINE_BLACK_THRESHOLD,
			  f1(b1, cfg->offset_mm - cfg->tol_mm), f1(b2, cfg->offset_mm),
			  f1(b3, cfg->tol_mm));
	if (junctions == 0U && s_kind_count[JUNCTION_REJECT_NARROW] != 0U &&
	    s_kind_count[JUNCTION_REJECT_UNCONFIRMED] == 0U)
		tm_printf((UB*)TAG "  HINT: only narrow left stripes. If one of them was the crossing "
			  "tape, lower JN_LEFT_MIN_MM below its width (barcode bars are ~3-9 mm).\n");
	if (s_kind_count[JUNCTION_REJECT_WIDE] != 0U)
		tm_printf((UB*)TAG "  HINT: a left stripe over %s mm was ignored. If that was the "
			  "crossing, raise JN_LEFT_MAX_MM.\n", f1(b1, cfg->left_max_mm));
	if (junctions == 0U && s_event_count == 0U)
		tm_printf((UB*)TAG "  HINT: the left sensor never saw black. Check its live reading "
			  "over the tape (black > %u) and that the crossing reaches it.\n",
			  (unsigned)LINE_ADC_BARCODE_BLACK_THRESHOLD);
}

void junction_hwtest_run(void)
{
	const wheel_speed_config_t pid_config = {
		PID_KP, PID_KI, PID_KD, (float)SPEED, (uint8_t)PID_BASE, (uint8_t)PID_MAX,
		PID_FF, PID_FF_REF_CM_S
	};
	const junction_config_t jn_config = {
		(float)JN_LEFT_MIN_MM, (float)JN_LEFT_MAX_MM, (float)JN_OFFSET_MM,
		(float)JN_TOL_MM, (float)JN_BOTH_MIN_MM
	};
	static line_adc_reader_t reader;
	line_adc_config_t ir_config;
	unsigned run = 0U;
	char b1[16], b2[16], b3[16], b4[16];
	ER er;

	er = gpio_set_pin(GO_PIN, GPIO_MODE_IN);
	if (er < E_OK) fail_forever("GO pin", er);
	clr_w(GPIO(GO_PIN), GPIO_PDE);
	set_w(GPIO(GO_PIN), GPIO_PUE);
	er = line_adc_reader_default_config(&ir_config, POLL_MS);
	if (er == E_OK) er = line_adc_reader_init(&reader, &ir_config);
	if (er < E_OK) fail_forever("IR ADC init", er);
	er = encoder_source_register();
	if (er < E_OK) fail_forever("encoder registration", er);
	er = gpio_dispatcher_start();
	if (er < E_OK) fail_forever("GPIO dispatcher", er);
	er = motor_driver_init();
	if (er < E_OK) fail_forever("motor driver init", er);
	if (!wheel_speed_control_init(&s_pid, &pid_config)) fail_forever("PID config", E_PAR);
	if (!junction_detector_init(&s_det, &jn_config)) fail_forever("junction config", E_PAR);
	s_go_last_pressed = go_pressed_raw();

	tm_printf((UB*)TAG "junction test: action %s, line following at %u cm/s (line_steering.h)\n",
		  s_action_name[JN_ACTION], (unsigned)SPEED);
	if (LF_STRADDLE)
		tm_printf((UB*)TAG "STRADDLE mode (test-only): straddle steering; junction = centre+"
			  "right both black >= %u mm (gaps < %u mm bridged, re-armed after %u mm); "
			  "left sensor not used\n", (unsigned)JN_BOTH_MIN_MM, (unsigned)JN_GAP_MM,
			  (unsigned)JN_REARM_MM);
	else
		tm_printf((UB*)TAG "junction = left black %s..%s mm, then centre+right both black "
			  ">= %s mm from %s mm after it (+/- %u mm)\n",
			  f1(b1, jn_config.left_min_mm), f1(b2, jn_config.left_max_mm),
			  f1(b3, jn_config.both_min_mm), f1(b4, jn_config.offset_mm),
			  (unsigned)JN_TOL_MM);
	if (JN_ACTION >= 2 && JN_SEEK)
		tm_printf((UB*)TAG "turn: forward %u mm, then spin %s until a line sensor finds the "
			  "line (looking from %u to %u deg), then follow %u mm\n",
			  (unsigned)JN_FORWARD_ACTION_MM,
			  JN_ACTION == 4 ? "180" : (JN_ACTION == 2 ? "left" : "right"),
			  (unsigned)((JN_ACTION == 4 ? 180 : 90) - JN_SEEK_BEFORE_DEG),
			  (unsigned)((JN_ACTION == 4 ? 180 : 90) + JN_SEEK_AFTER_DEG),
			  (unsigned)JN_AFTER_MM);
	else if (JN_ACTION >= 2)
		tm_printf((UB*)TAG "turn: forward %u mm, then spin %s deg (brake %u deg early), "
			  "then follow %u mm\n", (unsigned)JN_FORWARD_ACTION_MM,
			  JN_ACTION == 4 ? "180" : (JN_ACTION == 2 ? "+90" : "-90"),
			  (unsigned)JN_TURN_TRIM_DEG, (unsigned)JN_AFTER_MM);
	if (LF_STRADDLE && JN_NO_LINE)
		tm_printf((UB*)TAG "NO-LINE START (JN_NO_LINE=1): lay ONE tape across the mat, place "
			  "the robot square to it up to %u mm before it; it drives straight to it, "
			  "then does '%s'%s\n", (unsigned)JN_BLIND_MM, s_action_name[JN_ACTION],
			  JN_ACTION == 2 || JN_ACTION == 3 ?
			  " and follows the tape (needs ~JN_AFTER_MM of tape that side)" : "");
	else
		tm_printf((UB*)TAG "simulate a + : lay a straight piece of tape ACROSS the line ahead\n");

	for (;;) {
		unsigned junctions = 0U;
		bool no_line = false;
		float l, r;
		end_t end;

		run++;
		tm_printf((UB*)TAG "---- run %u: place the robot ON the line -- or, with no line, "
			  "with the left sensor on a tape laid across -- then press GO ----\n", run);
		wait_for_go(&reader);
		tm_printf((UB*)TAG "moving in %u s, hands clear\n", START_DELAY_MS / 1000U);
		tk_dly_tsk(START_DELAY_MS);
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);
		if (!read_mm(&l, &r)) fail_forever("encoder", E_IO);
		s_dist0_l = l;
		s_dist0_r = r;
		s_event_count = 0U;
		for (unsigned k = 0U; k < 5U; k++) s_kind_count[k] = 0U;
		detector_reset();
		if (JN_TRACE)
			tm_printf((UB*)TAG "   t_ms pos    turn  raw IR (black: L>%u, C/R>%u)   "
				  "distance  L/R wheel\n", (unsigned)LINE_ADC_BARCODE_BLACK_THRESHOLD,
				  (unsigned)LINE_ADC_LINE_BLACK_THRESHOLD);

		{
			ir_readings_t ir0;
			uint8_t pos0;

			if (!read_ir(&reader, &ir0, &pos0)) {
				end = END_SENSOR;
			} else if (LF_STRADDLE ? !JN_NO_LINE : pos0 != POS_LOST) {
				end = follow(&reader, (float)JN_RUN_MM, JN_ACTION != 1, &junctions);
			} else {
				/* No line under the line sensors: drive straight to the
				 * crossing (backing off it first if the left sensor
				 * starts on it). */
				tm_printf((UB*)TAG "  NO-LINE START: line sensors white -- driving straight "
					  "to find the crossing (up to %u mm)\n", (unsigned)JN_BLIND_MM);
				no_line = true;
				end = END_DIST;
				if (LF_STRADDLE ? ir0.center.black || ir0.right.black :
						  ir0.barcode.black) {
					tm_printf((UB*)TAG "  %s starts ON the tape: backing off "
						  "it first\n", LF_STRADDLE ? "a line sensor" :
						  "left sensor");
					end = back_off_tape(&reader);
					detector_reset();
				}
				if (end == END_DIST)
					end = drive_straight(&reader, (float)JN_BLIND_MM, true, true,
							     &junctions);
				if (end == END_DIST)
					tm_printf((UB*)TAG "  no junction within %u mm\n",
						  (unsigned)JN_BLIND_MM);
				if (end == END_JUNCTION && JN_ACTION == 1) {
					end = drive_straight(NULL, 100.0f, false, false, NULL);
					tm_printf((UB*)TAG "  carried straight on 100 mm (no line to "
						  "follow in a no-line start)\n");
				}
			}
		}

		if (end == END_JUNCTION) {
			float at = 0.0f, done = 0.0f;

			if (read_mm(&l, &r)) at = run_distance(l, r);
			if (JN_ACTION == 0) {
				(void)motor_brake_all();
				tk_dly_tsk(SETTLE_MS);
				if (read_mm(&l, &r))
					tm_printf((UB*)TAG "  STOPPED: braked at %s mm, rolled to %s mm "
						  "-- check where the sensors are over the crossing\n",
						  f1(b1, at), f1(b2, run_distance(l, r)));
				/* end stays END_JUNCTION: "stopped at the junction". */
			} else {
				float want = JN_ACTION == 2 ? 90.0f : (JN_ACTION == 3 ? -90.0f : 180.0f);
				/* No line behind in a no-line start: a U-turn spins a fixed
				 * 180 and drives back across the same tape instead. */
				/* Straddle: the sensors are ~JN_FORWARD_MM ahead of the axle,
				 * so after forward + 180 the crossing tape is under the
				 * axle and driving "back" never crosses it again. A U-turn
				 * there seeks the line behind instead (lay a stem). */
				const bool uturn_back = JN_ACTION == 4 && no_line && !LF_STRADDLE;
				const bool seek = JN_SEEK && !uturn_back;

				end = drive_straight(NULL, (float)JN_FORWARD_ACTION_MM, false, false,
						     NULL);
				(void)motor_brake_all();
				tk_dly_tsk(SETTLE_MS);
				if (end == END_DIST) {
					tm_printf((UB*)TAG "  forward %u mm done; spinning %s deg%s\n",
						  (unsigned)JN_FORWARD_ACTION_MM, f1(b1, want),
						  uturn_back ? " (fixed: no line behind to look for)" : "");
					end = spin(seek ? &reader : NULL, want, &done);
					if (seek && end == END_DIST)
						tm_printf((UB*)TAG "  spin: line found after %s deg (nominal "
							  "%s; encoders, includes the braking overshoot)\n",
							  f1(b2, done), f1(b1, want));
					else
						tm_printf((UB*)TAG "  spin: wanted %s deg, encoders say %s "
							  "deg -- compare with the real angle\n",
							  f1(b1, want), f1(b2, done));
					if (end == END_SEEK_FAIL) {
						if (JN_ACTION == 4 && LF_STRADDLE && no_line)
							tm_printf((UB*)TAG "  HINT: a U-turn looks for the line "
								  "BEHIND. Lay a strip of tape along the approach "
								  "(a T with the crossing tape), with the robot "
								  "straddling it at the start.\n");
						else
							tm_printf((UB*)TAG "  HINT: %s\n", JN_ACTION == 4 ?
								  "expected after a U-turn when there is no line "
								  "behind (single crossing tape)." :
								  "the sensors never crossed the new line: the "
								  "wheels were too far from it when spinning -- "
								  "adjust JN_FORWARD_MM / JN_FORWARD_RIGHT_MM.");
					}
				}
				if (end == END_DIST && uturn_back) {
					/* Check the U-turn: heading back, the robot must cross
					 * the same tape again (sensors ~JN_FORWARD_MM from it). */
					unsigned back = 0U;
					float d0 = 0.0f, d1 = 0.0f;
					const float limit = (float)JN_FORWARD_MM + (float)JN_BLIND_MM;

					if (read_mm(&l, &r)) d0 = run_distance(l, r);
					tm_printf((UB*)TAG "  driving back to cross the same tape again (up to "
						  "%s mm)\n", f1(b1, limit));
					detector_reset();
					end = drive_straight(&reader, limit, true, true, &back);
					(void)motor_brake_all();
					tk_dly_tsk(SETTLE_MS);
					if (read_mm(&l, &r)) d1 = run_distance(l, r);
					junctions += back;
					if (end == END_JUNCTION)
						tm_printf((UB*)TAG "  U-TURN OK: crossed the tape again after "
							  "%s mm on the way back (expected ~%u)\n",
							  f1(b1, d1 - d0), (unsigned)JN_FORWARD_MM);
					else if (end == END_DIST)
						tm_printf((UB*)TAG "  U-TURN FAILED: did not cross the tape "
							  "within %s mm on the way back -- the spin angle was "
							  "far off; check the robot's real heading\n",
							  f1(b1, limit));
				} else if (end == END_DIST) {
					ir_readings_t ir;
					uint8_t pos;

					if (read_ir(&reader, &ir, &pos))
						tm_printf((UB*)TAG "  after the turn: IR left %u centre %u "
							  "right %u -> %s\n", (unsigned)ir.barcode.raw,
							  (unsigned)ir.center.raw, (unsigned)ir.right.raw,
							  s_pos_name[pos & 3U]);
					detector_reset();
					end = follow(&reader, (float)JN_AFTER_MM, false, &junctions);
					if (end == END_NOT_ON_LINE && JN_ACTION == 4)
						tm_printf((UB*)TAG "  no line after the U-turn: expected "
							  "when there is no line behind (single crossing "
							  "tape); check the spin angle instead.\n");
					else if (end == END_NOT_ON_LINE)
						tm_printf((UB*)TAG "  HINT: no line under the sensors after "
							  "the turn. Too far / not far enough forward: change "
							  "JN_FORWARD_MM / JN_FORWARD_RIGHT_MM (~ line sensors to "
							  "the wheel axle); or the spin over/undershot: "
							  "JN_TURN_TRIM_DEG.\n");
				}
			}
		}
		(void)motor_brake_all();
		(void)motor_emergency_stop();
		tk_dly_tsk(SETTLE_MS);
		if (read_mm(&l, &r))
			tm_printf((UB*)TAG "==== RESULT: %s; %u junction(s); travelled %s mm ====\n",
				  end_text(end), junctions, f1(b1, run_distance(l, r)));
		else
			tm_printf((UB*)TAG "==== RESULT: %s; %u junction(s) ====\n", end_text(end),
				  junctions);
		report(&jn_config, junctions);
		if (fatal(end)) {
			encoder_fault_t f;

			if (end == END_ENCODER && encoder_source_fault(&f) >= E_OK &&
			    f.reason != ENCODER_FAULT_NONE)
				tm_printf((UB*)TAG "  ENCODER FAULT: %s wheel, reason %u, %u glitches, "
					  "%u both-channel jumps tolerated\n",
					  f.wheel == 1U ? "LEFT" : "RIGHT", (unsigned)f.reason,
					  (unsigned)f.glitches, (unsigned)f.ambiguous);
			tm_printf((UB*)TAG "test stopped, reset to retry\n");
			for (;;) tk_dly_tsk(5000U);
		}
	}
}

#endif /* APP_SENSOR_TEST == 18 */
