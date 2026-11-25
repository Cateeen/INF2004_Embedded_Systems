/*
 * diag_width_hwtest.c - obstacle width by a diagonal pass, then pull out to the
 * clear side (SENSOR_TEST=diag_width). DRIVES THE MOTORS.
 *
 * A flat face echoes only while the sensor is in front of it, whichever way the
 * sensor points, so the robot drives diagonally past the face with the servo
 * keeping the sensor facing it, and the width is the sideways distance between
 * where the echo starts and stops (subsystem5/obstacle/diag_pass.c). The robot
 * only translates during the measurement, so the sensor offset and the beam
 * width cancel out.
 *
 * Each GO press, starting on the line facing the obstacle about 30 cm away:
 *   1. Back up DIAG_BACKUP_MM straight (room for step 3).
 *   2. Turn to L DIAG_ANGLE_DEG. The servo is held DIAG_ANGLE_DEG to the RIGHT
 *      the whole test, so the sensor faces the obstacle again.
 *   3. FORWARD on that heading (the sensor slides left and closer) until it is
 *      past the left edge. Positioning only.
 *   4. REVERSE on the same heading (the sensor slides right and away): the echo
 *      starts at the left edge and stops at the right edge. Width = the sideways
 *      distance between them, from the encoders. Moving away, it can never get
 *      too close, and it ends on the right (the default bypass side).
 *   5. Choose the side it sticks out less on (RIGHT on a tie), turn 90 deg to
 *      it, drive until the car centre is the edge + half the car width +
 *      DIAG_CLEAR_MM out, and turn back along the line (DIAG_PULLOUT=1).
 * Straight legs use the tuned wheel-speed PID at DIAG_SPEED_CM_S with a heading
 * hold from the encoders; turns use the spin_heading calibration.
 * GO during a move stops the motors and ends the test; so do a timeout, a
 * stall, an encoder fault, missing readings, getting too close to the face
 * before passing its left edge, or the face going out of range.
 */
#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "../../subsystem5/obstacle/obs_pico_port.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"
#include "../../subsystem5/obstacle/diag_pass.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "motor_driver.h"
#include "wheel_speed_control.h"
#include "diag_width_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 16

#ifndef SPIN_TRACK_MM
#define SPIN_TRACK_MM 116U
#endif
#ifndef SPIN_PWM
#define SPIN_PWM 25U
#endif
#ifndef SPIN_PWM_SLOW
#define SPIN_PWM_SLOW 20U
#endif
#ifndef DIAG_ANGLE_DEG
#define DIAG_ANGLE_DEG 20U        /* pass heading L this, servo R this (servo limit 20) */
#endif
#ifndef DIAG_SPEED_CM_S
#define DIAG_SPEED_CM_S 15U       /* along the path; sideways = speed * sin(angle) */
#endif
#ifndef DIAG_BACKUP_MM
#define DIAG_BACKUP_MM 100U       /* straight back-up first: room to reach the left edge */
#endif
#ifndef DIAG_CAR_WIDTH_MM
#define DIAG_CAR_WIDTH_MM 140U
#endif
#ifndef DIAG_CLEAR_MM
#define DIAG_CLEAR_MM 50U         /* gap between the car side and the obstacle edge */
#endif
#ifndef DIAG_PULLOUT
#define DIAG_PULLOUT 1            /* 1 = pull out to the clear side after measuring */
#endif
#ifndef DIAG_TRUE_WIDTH_MM
#define DIAG_TRUE_WIDTH_MM 0U     /* known width: print the error */
#endif
#ifndef DIAG_SENSOR_OFFSET_MM
#define DIAG_SENSOR_OFFSET_MM 70U /* wheel axle to the sensor face */
#endif

_Static_assert(DIAG_ANGLE_DEG >= 10U && DIAG_ANGLE_DEG <= 20U, "DIAG_ANGLE_DEG must be 10..20 (servo limit)");
_Static_assert(DIAG_SPEED_CM_S >= 5U && DIAG_SPEED_CM_S <= 20U, "DIAG_SPEED_CM_S must be 5..20");
_Static_assert(DIAG_BACKUP_MM <= 300U, "DIAG_BACKUP_MM must be 0..300");
_Static_assert(DIAG_CAR_WIDTH_MM >= 80U && DIAG_CAR_WIDTH_MM <= 250U, "DIAG_CAR_WIDTH_MM out of range");
_Static_assert(DIAG_CLEAR_MM <= 200U, "DIAG_CLEAR_MM out of range");

/* Same calibration as the other motion tests and the vehicle. */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define LEFT_SIGN          (-1)
#define RIGHT_SIGN         1
#define LEFT_MOTOR_FWD     1
#define RIGHT_MOTOR_FWD    (-1)
#define PID_KP_V           2.0f    /* vehicle gains (app_main.c) */
#define PID_KI_V           4.0f
#define PID_BASE_V         22U
#define PID_MAX_V          60U
#define PID_PERIOD_MS      50U

#define GO_PIN             20U
#define TAG                "[HWTEST][DIAG-WIDTH] "
#define PI_F               3.14159265f
#define DEG                (PI_F / 180.0f)
#define LOOP_MS            10U
#define START_DELAY_MS     3000U
#define SLOW_ZONE_DEG      15.0f
#define SETTLE_MS          500U
#define TURN_TIMEOUT_MS    8000U
#define STALL_MS           1000U
#define STALL_DEG          0.3f
#define STALL_MM           2.0f
#define WRONG_WAY_DEG      5.0f
#define WRONG_WAY_MM       20.0f
#define READ_TIMEOUT_MS    800U
#define RETURN_LEAD_DEG    3.0f    /* final turn brakes this early (coast) */
#define HEADING_GAIN       0.5f    /* cm/s per degree of heading error */
#define START_FACE_MM      300.0f  /* assumed start distance until the face is heard */
#define FACE_MAX_MM        500.0f  /* first face echo within this (then FACE_SEARCH_MM) */
#define FACE_SEARCH_MM     120.0f  /* ... or within this beyond where it should be */
#define FACE_CLUSTER_MM    60.0f
#define MIN_FACE_MM        100.0f  /* never drive the sensor closer than this to the face */
#define MAX_FACE_MM        880.0f  /* reliable range (bench: 90 cm) */
#define SEARCH_MM          80.0f   /* face not heard this far left of the line: it is right */
#define MAX_LEG_MM         900.0f
#define TIE_MM             10.0f   /* right wins if it sticks out at most this much more */

typedef enum { RUN_OK = 0, RUN_ABORT_GO, RUN_TIMEOUT, RUN_STALL, RUN_WRONG_WAY,
	       RUN_ENCODER, RUN_SENSOR, RUN_PID, RUN_NOT_FOUND, RUN_TOO_CLOSE,
	       RUN_OUT_OF_RANGE } run_status_t;

typedef enum { LEG_BACKUP = 0, LEG_POSITION, LEG_CROSS, LEG_PULLOUT } leg_t;

/* Pose from the encoders: x mm right of the line, y mm along it, heading deg
 * relative to the line (+ = left), all from the robot's start. */
typedef struct {
	float x, y, h;
	float l, r;              /* last wheel travel, mm */
	float base;              /* absolute heading of the line */
} pose_t;

typedef struct {
	bool entered, exited;
	float entry_x, exit_x;
	bool entry_known;
} leg_result_t;

static pose_t s_pose;
static bool s_go_last_pressed;
static obstacle_event_message_t s_latest;
static bool s_new_reading;
static uint32_t s_last_reading_ms;
static diag_pass_t s_pass;
static wheel_speed_control_t s_pid;

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

static uint32_t now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

static bool go_pressed_raw(void)
{
	return gpio_get_val(GO_PIN) == 0U;           /* active low */
}

/* Keep the obstacle event queue empty (a full queue faults the task) and keep
 * the newest ahead reading. */
static void drain_events(void)
{
	for (;;) {
		obstacle_event_message_t event;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL);

		if (received != (INT)sizeof(event)) return;
		if (event.header.type == VEHICLE_MESSAGE_OBSTACLE_EVENT &&
		    event.obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING) {
			s_latest = event;
			s_new_reading = true;
			s_last_reading_ms = now_ms();
		}
	}
}

static bool read_wheels(float *l, float *r)
{
	encoder_snapshot_t s;
	const float mm_per_count = WHEEL_CIRC_MM / COUNTS_PER_REV;

	if (encoder_source_snapshot(&s) < E_OK || !s.valid) return false;
	*l = (float)(LEFT_SIGN * s.left_count) * mm_per_count;
	*r = (float)(RIGHT_SIGN * s.right_count) * mm_per_count;
	return true;
}

static float abs_heading(float l, float r)
{
	return (r - l) / (float)SPIN_TRACK_MM / DEG;
}

/* Integrate the pose from the wheel travel since the last call. */
static bool pose_update(void)
{
	float l, r, ds, h, hm;

	if (!read_wheels(&l, &r)) return false;
	h = abs_heading(l, r) - s_pose.base;
	ds = 0.5f * ((l - s_pose.l) + (r - s_pose.r));
	hm = 0.5f * (h + s_pose.h) * DEG;            /* midpoint heading over the step */
	s_pose.x += ds * -__builtin_sinf(hm);
	s_pose.y += ds * __builtin_cosf(hm);
	s_pose.h = h;
	s_pose.l = l;
	s_pose.r = r;
	return true;
}

static bool pose_reset(void)
{
	float l, r;

	if (!read_wheels(&l, &r)) return false;
	s_pose = (pose_t){ 0.0f, 0.0f, 0.0f, l, r, abs_heading(l, r) };
	return true;
}

static void sensor_pos(float *x, float *y)
{
	*x = s_pose.x + (float)DIAG_SENSOR_OFFSET_MM * -__builtin_sinf(s_pose.h * DEG);
	*y = s_pose.y + (float)DIAG_SENSOR_OFFSET_MM * __builtin_cosf(s_pose.h * DEG);
}

/* Sensor distance to the face expected from the pose: from the tracker once
 * the face was heard, else from the assumed start distance. */
static float expected_face_mm(void)
{
	float sx, sy, mm;

	sensor_pos(&sx, &sy);
	if (diag_pass_expected_mm(&s_pass, sy, &mm)) return mm;
	return (START_FACE_MM + (float)DIAG_SENSOR_OFFSET_MM) - sy;
}

static void wait_ms(uint32_t ms)
{
	uint32_t t0 = now_ms();

	while (now_ms() - t0 < ms) {
		drain_events();
		(void)pose_update();
		tk_dly_tsk(LOOP_MS);
	}
}

static bool go_edge(void)
{
	bool now = go_pressed_raw();
	bool edge = false;

	if (now != s_go_last_pressed) {
		wait_ms(LOOP_MS * 2U);
		if (go_pressed_raw() == now) {
			edge = now && !s_go_last_pressed;
			s_go_last_pressed = now;
		}
	}
	return edge;
}

static void wait_for_go(void)
{
	while (!go_edge()) {
		drain_events();
		tk_dly_tsk(LOOP_MS);
	}
	while (go_pressed_raw())
		wait_ms(LOOP_MS);
	s_go_last_pressed = false;
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

/* Heading as L12.3 / R12.3 / C0.0. */
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

/* dir +1 = turn left, -1 = right. */
static void spin(int dir, unsigned pwm)
{
	int16_t left = (int16_t)(-dir * (int)pwm);
	int16_t right = (int16_t)(dir * (int)pwm);

	(void)motor_set_speed(MOTOR_1, (int16_t)(left * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(right * RIGHT_MOTOR_FWD));
}

static void drive(int dir, uint8_t pwm_l, uint8_t pwm_r)
{
	(void)motor_set_speed(MOTOR_1, (int16_t)(dir * (int)pwm_l * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(dir * (int)pwm_r * RIGHT_MOTOR_FWD));
}

/* Turn in place to a heading relative to the line; brake `lead` deg early. */
static run_status_t turn_to(float target, float lead)
{
	float start, progress_h;
	uint32_t t0 = now_ms(), progress_ms = 0U;
	int dir;
	run_status_t status = RUN_OK;

	if (!pose_update()) return RUN_ENCODER;
	start = progress_h = s_pose.h;
	dir = target > s_pose.h ? 1 : -1;
	for (;;) {
		float remaining;
		uint32_t elapsed = now_ms() - t0;

		if (!pose_update()) { status = RUN_ENCODER; break; }
		remaining = (target - s_pose.h) * (float)dir;
		if (remaining <= lead) break;
		if (go_pressed_raw()) { status = RUN_ABORT_GO; break; }
		if (elapsed > TURN_TIMEOUT_MS) { status = RUN_TIMEOUT; break; }
		if ((start - s_pose.h) * (float)dir > WRONG_WAY_DEG) { status = RUN_WRONG_WAY; break; }
		if ((s_pose.h - progress_h) * (float)dir >= STALL_DEG) {
			progress_h = s_pose.h;
			progress_ms = elapsed;
		} else if (elapsed - progress_ms > STALL_MS) {
			status = RUN_STALL;
			break;
		}
		spin(dir, remaining > SLOW_ZONE_DEG ? SPIN_PWM : SPIN_PWM_SLOW);
		drain_events();
		tk_dly_tsk(LOOP_MS);
	}
	(void)motor_brake_all();
	wait_ms(SETTLE_MS);
	return status;
}

/* Feed a new reading to the pass tracker; print it. */
static diag_event_t take_reading(const char *label)
{
	char b1[16], b2[16];
	float sx, sy;
	uint16_t mm = s_latest.distance_mm;
	diag_event_t ev;
	bool valid = mm != OBS_DIST_NONE && mm >= OBS_MIN_RANGE_MM && mm <= OBS_MAX_RANGE_MM;

	s_new_reading = false;
	sensor_pos(&sx, &sy);
	/* Until the face is heard, its first echo must be near where it should
	 * be from the assumed start distance (the background is much farther). */
	if (!s_pass.have_face) {
		float limit = expected_face_mm() + FACE_SEARCH_MM;

		s_pass.face_max_mm = limit > MAX_FACE_MM ? MAX_FACE_MM : limit;
	}
	ev = diag_pass_feed(&s_pass, sx, sy, mm);
	if (valid)
		tm_printf((UB*)TAG "  %s sensor x %6s  %4u mm  %s\n", label, f1(b1, sx), (unsigned)mm,
			  s_pass.on_face ? "face" : "-");
	else
		tm_printf((UB*)TAG "  %s sensor x %6s  NONE\n", label, f1(b1, sx));
	if (ev == DIAG_EVENT_ENTERED)
		tm_printf((UB*)TAG "  >> face heard from x %s%s\n", f1(b2, s_pass.entry_x),
			  s_pass.entry_known ? "" : " (already in front of it)");
	if (ev == DIAG_EVENT_EXITED)
		tm_printf((UB*)TAG "  >> face lost at x %s\n", f1(b2, s_pass.exit_x));
	return ev;
}

/* One straight leg at a fixed heading with the wheel-speed PID.
 * BACKUP: reverse `target` mm. POSITION: forward until past the left edge.
 * CROSS: reverse across the face. PULLOUT: forward until x reaches `target`. */
static run_status_t run_leg(leg_t leg, int dir, float heading, float target, leg_result_t *res)
{
	static const char *const labels[] = { "BAK", "POS", "X  ", "OUT" };
	const float v = (float)DIAG_SPEED_CM_S;
	const char *label = labels[leg];
	wheel_speed_output_t out = { PID_BASE_V, PID_BASE_V };
	float l0 = s_pose.l, r0 = s_pose.r, pl = s_pose.l, pr = s_pose.r;
	float prog_l = 0.0f, prog_r = 0.0f, x_start = s_pose.x;
	uint32_t t0 = now_ms(), next = t0 + PID_PERIOD_MS, last = t0;
	uint32_t prog_l_ms = 0U, prog_r_ms = 0U;
	run_status_t status = RUN_OK;

	*res = (leg_result_t){0};
	if (!wheel_speed_control_reset(&s_pid) || !wheel_speed_control_set_targets(&s_pid, v, v))
		return RUN_PID;
	drain_events();
	s_new_reading = false;
	s_last_reading_ms = now_ms();
	drive(dir, out.left_pwm_percent, out.right_pwm_percent);

	for (;;) {
		uint32_t t, run;
		float dl, dr, travelled;

		tk_dly_tsk(LOOP_MS);
		drain_events();
		if (!pose_update()) { status = RUN_ENCODER; break; }
		t = now_ms();
		run = t - t0;
		dl = s_pose.l - l0;
		dr = s_pose.r - r0;
		travelled = 0.5f * (absf(dl) + absf(dr));

		if (go_pressed_raw()) { status = RUN_ABORT_GO; break; }
		if (dl * (float)dir < -WRONG_WAY_MM || dr * (float)dir < -WRONG_WAY_MM) {
			status = RUN_WRONG_WAY;
			break;
		}
		if (absf(dl - prog_l) >= STALL_MM) { prog_l = dl; prog_l_ms = run; }
		if (absf(dr - prog_r) >= STALL_MM) { prog_r = dr; prog_r_ms = run; }
		if (run - prog_l_ms > STALL_MS || run - prog_r_ms > STALL_MS) { status = RUN_STALL; break; }
		if (travelled > MAX_LEG_MM) { status = RUN_TIMEOUT; break; }

		if (leg == LEG_BACKUP) {
			if (travelled >= target) break;
		} else if (leg == LEG_PULLOUT) {
			if ((s_pose.x - target) * (target > x_start ? 1.0f : -1.0f) >= 0.0f) break;
		} else {
			float sx, sy, face;

			if (t - s_last_reading_ms > READ_TIMEOUT_MS) { status = RUN_SENSOR; break; }
			if (s_new_reading) {
				diag_event_t ev = take_reading(label);

				if (ev == DIAG_EVENT_ENTERED && !res->entered) {
					res->entered = true;
					res->entry_x = s_pass.entry_x;
					res->entry_known = s_pass.entry_known;
				}
				if (ev == DIAG_EVENT_EXITED) {
					res->exited = true;
					res->exit_x = s_pass.exit_x;
					/* POSITION: past the left edge. CROSS: crossed it. */
					if (leg == LEG_POSITION || res->entered) break;
				}
			}
			sensor_pos(&sx, &sy);
			face = expected_face_mm();
			if (leg == LEG_POSITION) {
				/* Not heard on the left at all: the face lies to the right,
				 * and the crossing will find it. */
				if (!res->entered && !s_pass.on_face && sx < -SEARCH_MM) break;
				if (face < MIN_FACE_MM) {
					if (!res->entered && !s_pass.on_face) break;
					status = RUN_TOO_CLOSE;
					break;
				}
			} else if (face > MAX_FACE_MM) {
				status = RUN_OUT_OF_RANGE;
				break;
			}
		}

		if ((int32_t)(t - next) < 0) continue;
		next += PID_PERIOD_MS;
		{
			float dt = (float)(t - last) / 1000.0f;
			float sl = (s_pose.l - pl) * (float)dir / 10.0f / dt;
			float sr = (s_pose.r - pr) * (float)dir / 10.0f / dt;
			/* Heading hold: e > 0 = must turn left. */
			float e = heading - s_pose.h;
			float tl = v - (float)dir * HEADING_GAIN * e;
			float tr = v + (float)dir * HEADING_GAIN * e;

			pl = s_pose.l;
			pr = s_pose.r;
			last = t;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (tl < 1.0f) tl = 1.0f;
			if (tr < 1.0f) tr = 1.0f;
			if (!wheel_speed_control_set_targets(&s_pid, tl, tr) ||
			    !wheel_speed_control_update(&s_pid, sl, sr, dt, &out)) {
				status = RUN_PID;
				break;
			}
		}
		drive(dir, out.left_pwm_percent, out.right_pwm_percent);
	}
	(void)motor_brake_all();
	wait_ms(SETTLE_MS);
	return status;
}

static const char *status_text(run_status_t s)
{
	switch (s) {
	case RUN_ABORT_GO:     return "stopped by GO button";
	case RUN_TIMEOUT:      return "timeout: move did not finish";
	case RUN_STALL:        return "stalled: wheels not turning (raise DIAG_SPEED_CM_S or SPIN_PWM)";
	case RUN_WRONG_WAY:    return "moved the WRONG WAY: motor or encoder sign";
	case RUN_ENCODER:      return "encoder INVALID";
	case RUN_SENSOR:       return "no distance readings from the HC-SR04";
	case RUN_PID:          return "wheel-speed PID rejected its input";
	case RUN_NOT_FOUND:    return "face not found / not crossed";
	case RUN_TOO_CLOSE:    return "too close to the face before passing its left edge "
				      "(obstacle too far left: raise DIAG_BACKUP_MM)";
	case RUN_OUT_OF_RANGE: return "face out of range before its right edge (too wide or too far right)";
	default:               return "ok";
	}
}

static run_status_t pass(void)
{
	char b1[16], b2[16], b3[16], b4[16];
	leg_result_t res, cross;
	diag_plan_t plan;
	run_status_t st;
	const float angle = (float)DIAG_ANGLE_DEG;

	if (!pose_reset()) return RUN_ENCODER;
	diag_pass_start(&s_pass, FACE_MAX_MM, FACE_CLUSTER_MM, false);

	if (DIAG_BACKUP_MM > 0U) {
		tm_printf((UB*)TAG "backing up %u mm\n", (unsigned)DIAG_BACKUP_MM);
		st = run_leg(LEG_BACKUP, -1, 0.0f, (float)DIAG_BACKUP_MM, &res);
		if (st != RUN_OK) return st;
	}
	tm_printf((UB*)TAG "turning to L%u deg (sensor faces the obstacle)\n", (unsigned)DIAG_ANGLE_DEG);
	st = turn_to(angle, 0.0f);
	if (st != RUN_OK) return st;

	tm_printf((UB*)TAG "forward until past the LEFT edge:\n");
	st = run_leg(LEG_POSITION, 1, angle, 0.0f, &res);
	if (st != RUN_OK) return st;

	tm_printf((UB*)TAG "reversing across the face:\n");
	diag_pass_start(&s_pass, FACE_MAX_MM, FACE_CLUSTER_MM, true);
	st = run_leg(LEG_CROSS, -1, angle, 0.0f, &cross);
	if (st != RUN_OK) return st;
	if (!cross.entered || !cross.entry_known || !cross.exited) return RUN_NOT_FOUND;

	if (!diag_pass_plan(cross.entry_x, cross.exit_x, (float)DIAG_CAR_WIDTH_MM,
			    (float)DIAG_CLEAR_MM, TIE_MM, &plan))
		return RUN_NOT_FOUND;
	tm_printf((UB*)TAG "RESULT:\n");
	tm_printf((UB*)TAG "  WIDTH        : %s mm  (left edge x %s, right edge x %s; forward leg lost it at x %s)\n",
		  f1(b1, plan.width_mm), f1(b2, plan.left_x), f1(b3, plan.right_x),
		  res.exited ? f1(b4, res.exit_x) : "-");
	tm_printf((UB*)TAG "  extents      : %s mm left of the line, %s mm right of it\n",
		  f1(b1, plan.left_extent_mm), f1(b2, plan.right_extent_mm));
	if (DIAG_TRUE_WIDTH_MM != 0U)
		tm_printf((UB*)TAG "  error        : %s mm against the true %u mm\n",
			  f1(b1, plan.width_mm - (float)DIAG_TRUE_WIDTH_MM), (unsigned)DIAG_TRUE_WIDTH_MM);
	tm_printf((UB*)TAG "  CSV,%s,%s,%s\n", f1(b1, plan.width_mm), f1(b2, plan.left_x),
		  f1(b3, plan.right_x));
	tm_printf((UB*)TAG "  bypass side  : %s (car centre to x %s mm)\n",
		  plan.side == DIAG_SIDE_RIGHT ? "RIGHT" : "LEFT", f1(b1, plan.target_x));

	if (DIAG_PULLOUT) {
		float heading = plan.side == DIAG_SIDE_RIGHT ? -90.0f : 90.0f;
		float edge = plan.side == DIAG_SIDE_RIGHT ? plan.right_x : plan.left_x;
		float face;
		char b5[16];

		tm_printf((UB*)TAG "pulling out to the %s\n", plan.side == DIAG_SIDE_RIGHT ? "right" : "left");
		st = turn_to(heading, 0.0f);
		if (st != RUN_OK) return st;
		st = run_leg(LEG_PULLOUT, 1, heading, plan.target_x, &res);
		if (st != RUN_OK) return st;
		st = turn_to(0.0f, RETURN_LEAD_DEG);
		if (st != RUN_OK) return st;
		(void)pose_update();
		face = s_pass.have_face ? s_pass.face_y - s_pose.y : 0.0f;
		tm_printf((UB*)TAG "  done: car centre x %s mm (target %s), heading %s, axle %s mm "
			  "behind the face; car side to obstacle edge %s mm\n",
			  f1(b1, s_pose.x), f1(b2, plan.target_x), fdeg(b3, s_pose.h), f1(b4, face),
			  f1(b5, absf(s_pose.x - edge) - 0.5f * (float)DIAG_CAR_WIDTH_MM));
	}
	return RUN_OK;
}

void diag_width_hwtest_run(void)
{
	static const obs_task_config_t task_config = {
		50U, 500U, 1000U, 5000U, 500U, 300U,
		NULL, NULL, NULL, NULL
	};
	const wheel_speed_config_t pid_config = {
		PID_KP_V, PID_KI_V, 0.0f, (float)DIAG_SPEED_CM_S, PID_BASE_V, PID_MAX_V
	};
	obs_pico_port_config_t hardware;
	uint16_t side_us, ahead_us;
	unsigned n = 0U;
	ER er;

	er = gpio_set_pin(GO_PIN, GPIO_MODE_IN);
	if (er < E_OK) fail_forever("GO pin", er);
	clr_w(GPIO(GO_PIN), GPIO_PDE);
	set_w(GPIO(GO_PIN), GPIO_PUE);
	er = encoder_source_register();
	if (er < E_OK) fail_forever("encoder registration", er);
	er = obs_pico_port_register();
	if (er < E_OK) fail_forever("ultrasonic registration", er);
	er = gpio_dispatcher_start();
	if (er < E_OK) fail_forever("GPIO dispatcher", er);
	er = motor_driver_init();
	if (er < E_OK) fail_forever("motor driver init", er);
	if (!wheel_speed_control_init(&s_pid, &pid_config)) fail_forever("PID config", E_PAR);

	/* Hold the servo DIAG_ANGLE_DEG to the RIGHT: the obstacle task points the
	 * servo at its "ahead" angle, so give "ahead" the pulse for R angle. The
	 * pulses stay within the verified range; the right end is 1 us beyond it
	 * only to keep the calibration strictly increasing (scans are never
	 * requested, so it is never sent). */
	hardware = obs_pico_port_default_config();
	ahead_us = hardware.servo.ahead_us;
	if (!obs_servo_pulse(&hardware.servo,
			     (uint8_t)(hardware.servo.ahead_angle_deg - DIAG_ANGLE_DEG), &side_us))
		fail_forever("servo angle", E_PAR);
	hardware.servo.right_us = (uint16_t)(side_us - 1U);
	hardware.servo.ahead_us = side_us;
	hardware.servo.left_us = ahead_us;
	hardware.echo_voltage_verified_3v3_safe = true;
	hardware.servo_limits_verified = true;
	er = obs_pico_port_task_start(&hardware, &task_config);
	if (er < E_OK) fail_forever("ultrasonic task start", er);
	s_go_last_pressed = go_pressed_raw();

	tm_printf((UB*)TAG "diagonal-pass width test: back up %u mm, heading L%u, servo R%u (%u us), "
		  "%u cm/s, car %u mm wide, clearance %u mm, pull-out %s\n",
		  (unsigned)DIAG_BACKUP_MM, (unsigned)DIAG_ANGLE_DEG, (unsigned)DIAG_ANGLE_DEG,
		  (unsigned)side_us, (unsigned)DIAG_SPEED_CM_S, (unsigned)DIAG_CAR_WIDTH_MM,
		  (unsigned)DIAG_CLEAR_MM, DIAG_PULLOUT ? "on" : "off");
	tm_printf((UB*)TAG "SETUP: robot on the line facing the obstacle ~30 cm away, face square "
		  "to the line; ~60 cm free behind and ~30 cm to each side. CHECK: the sensor "
		  "now points %u deg to the RIGHT of the robot.\n", (unsigned)DIAG_ANGLE_DEG);
	tm_printf((UB*)TAG "each GO press runs one pass; GO during a move = STOP\n");

	for (;;) {
		run_status_t st;

		tm_printf((UB*)TAG "---- pass %u: press GO ----\n", ++n);
		wait_for_go();
		tm_printf((UB*)TAG "moving in %u s, hands clear\n", START_DELAY_MS / 1000U);
		wait_ms(START_DELAY_MS);
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);
		st = pass();
		(void)motor_emergency_stop();
		if (st != RUN_OK) {
			tm_printf((UB*)TAG "PASS ENDED: %s -- test stopped, reset to retry\n",
				  status_text(st));
			for (;;) {
				drain_events();
				tk_dly_tsk(100U);
			}
		}
		tm_printf((UB*)TAG "put the robot back on the line facing the obstacle for the next pass\n");
	}
}

#endif /* APP_SENSOR_TEST == 16 */
