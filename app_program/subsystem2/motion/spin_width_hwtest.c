/*
 * spin_width_hwtest.c - obstacle width by turning the robot (SENSOR_TEST=spin_width).
 *
 * The HC-SR04 beam is wider than the servo's +/-20 deg sweep, so the servo stays
 * pointing ahead and the whole robot turns in place instead. Every reading is
 * tagged with the MEASURED encoder heading, so a turn's stop overshoot does not
 * matter. The maths (edges, trim, width, free gaps, edge tracking) is the shared
 * module subsystem5/obstacle/spin_scan.c; the whole manoeuvre below is
 * subsystem2/motion/obstacle_bypass.c, which the vehicle runs too (this file
 * only sets it up from the SCAN_* build options and drives it with GO).
 *
 * Scan (one per GO press), starting pointed at the obstacle (heading 0):
 *   1. FIND   step right in SCAN_FIND_STEP_DEG steps until the obstacle is lost
 *             for 2 readings in a row (these readings are not used for the width).
 *   2. Turn to just past the right edge, far enough to also see SCAN_LANE_MM of
 *             room beside it.
 *   3. SWEEP  step left in SCAN_STEP_DEG steps, storing every reading, across the
 *             obstacle until it is lost on the left and the room beyond the left
 *             edge has been seen (or L SCAN_HALF_DEG).
 *   4. Turn back to heading 0 and print the result.
 *   5. BYPASS (SCAN_BYPASS=1): choose the side - RIGHT unless the obstacle
 *             sticks out further right than left - turn 90 deg to it, drive
 *             until the car centre is the edge + SCAN_MARGIN_MM (the scan's
 *             width error) + SCAN_CLEAR_MM + half the car out, and turn back
 *             along the line. A side whose edge was not seen, or whose free
 *             gap is too narrow, is not used. The car stops there, beside the
 *             obstacle but still behind it.
 *   6. PASS + REJOIN (SCAN_REJOIN=1): drive forward to SCAN_MIN_DEPTH_MM past
 *             the face, turn 90 deg to face the line and read across: the
 *             sensor now faces the obstacle's side square on (a strong echo).
 *             Anything within the strip the car will cross to rejoin the line
 *             = still beside it: turn back, drive SCAN_PASS_STEP_MM, look
 *             again. When clear, drive half the car + SCAN_CLEAR_MM further
 *             (the car is that wide crossing back), turn to the line, drive
 *             to it and turn back along it. The looks also give the
 *             obstacle's length to +/- half a step.
 * All measuring steps turn the same way (left), so there is no reversal or long
 * turn in the middle of the measurement. If nothing is heard ahead at the start,
 * or with SCAN_FULL=1, it sweeps the full R..L SCAN_HALF_DEG instead.
 *
 * Width: the obstacle is the run of readings within SCAN_CLUSTER_MM of the echo
 * nearest ahead. Each edge lies half way between the last reading that still
 * hears it and the first that does not, pulled in by SCAN_TRIM_DEG (calibrate:
 * with SCAN_TRUE_WIDTH_MM set, the report prints the trim that would give the
 * true width). The trim depends on distance, so always scan from about the same
 * distance (the vehicle stops at 300 mm).
 *
 * GO during a turn stops the motors and ends the test; so do a timeout, a stall,
 * an encoder fault or a missing distance reading.
 */
#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "../../subsystem5/obstacle/obs_pico_port.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"
#include "../../subsystem5/obstacle/spin_scan.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "motor_driver.h"
#include "obstacle_bypass.h"
#include "spin_width_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 15

/* Turning calibration from the spin_heading test. */
#ifndef SPIN_TRACK_MM
#define SPIN_TRACK_MM 116U
#endif
#ifndef SPIN_PWM
#define SPIN_PWM 25U
#endif
#ifndef SPIN_PWM_SLOW
#define SPIN_PWM_SLOW 20U
#endif
/* Scan settings. Floats (SCAN_TRIM_DEG) may be written with decimals. */
#ifndef SCAN_HALF_DEG
#define SCAN_HALF_DEG 70U         /* never turn further than this either way (= vehicle) */
#endif
#ifndef SCAN_STEP_DEG
#define SCAN_STEP_DEG 3U          /* measuring step (sweep) */
#endif
#ifndef SCAN_FIND_STEP_DEG
#define SCAN_FIND_STEP_DEG 6U     /* edge-finding step (not measured) */
#endif
#ifndef SCAN_SENSOR_OFFSET_MM
#define SCAN_SENSOR_OFFSET_MM 70U /* wheel axle to the sensor face */
#endif
#ifndef SCAN_CLUSTER_MM
#define SCAN_CLUSTER_MM 60U       /* reading belongs to the obstacle within this of the ahead echo */
#endif
#ifndef SCAN_LANE_MM
#define SCAN_LANE_MM 200U         /* free gap needed beside the obstacle: robot width + clearance */
#endif
#ifndef SCAN_BEYOND_MM
#define SCAN_BEYOND_MM 400U       /* side objects farther than this beyond the obstacle are
				   * ignored: assumed depth + robot length + margin */
#endif
#ifndef SCAN_MAX_OBSTACLE_MM
#define SCAN_MAX_OBSTACLE_MM 450U /* echoes farther than this are background, not the obstacle */
#endif
#ifndef SCAN_TRIM_DEG
#define SCAN_TRIM_DEG 15.0        /* beam: pull each edge in by this much (bench, 20 cm) */
#endif
#ifndef SCAN_TRUE_WIDTH_MM
#define SCAN_TRUE_WIDTH_MM 0U     /* known width: print the trim that matches it */
#endif
#ifndef SCAN_FULL
#define SCAN_FULL 0               /* 1 = always sweep the full R..L SCAN_HALF_DEG */
#endif
#ifndef SCAN_BYPASS
#define SCAN_BYPASS 1             /* 1 = pull out to the chosen side after the scan */
#endif
#ifndef SCAN_CAR_WIDTH_MM
#define SCAN_CAR_WIDTH_MM 140U
#endif
#ifndef SCAN_CLEAR_MM
#define SCAN_CLEAR_MM 50U         /* gap between the car side and the obstacle edge */
#endif
#ifndef SCAN_MARGIN_MM
#define SCAN_MARGIN_MM 80U        /* extra, for the scan's width error (bench: about +/-75 mm) */
#endif
#ifndef SCAN_SPEED_CM_S
#define SCAN_SPEED_CM_S 15U       /* pull-out speed (the PID was tuned at 15) */
#endif
#ifndef SCAN_REJOIN
#define SCAN_REJOIN 1             /* 1 = after pulling out, drive past and rejoin the line */
#endif
#ifndef SCAN_MIN_DEPTH_MM
#define SCAN_MIN_DEPTH_MM 50U     /* first look this far past the face */
#endif
#ifndef SCAN_PASS_STEP_MM
#define SCAN_PASS_STEP_MM 100U    /* then every this much */
#endif
#ifndef SCAN_MAX_DEPTH_MM
#define SCAN_MAX_DEPTH_MM 600U    /* give up if still beside it this far past the face */
#endif

_Static_assert(SPIN_TRACK_MM >= 50U && SPIN_TRACK_MM <= 300U, "SPIN_TRACK_MM out of range");
_Static_assert(SPIN_PWM <= 80U && SPIN_PWM_SLOW <= SPIN_PWM && SPIN_PWM_SLOW >= 10U,
	       "need 10 <= SPIN_PWM_SLOW <= SPIN_PWM <= 80");
_Static_assert(SCAN_HALF_DEG >= 20U && SCAN_HALF_DEG <= 75U, "SCAN_HALF_DEG must be 20..75");
_Static_assert(SCAN_STEP_DEG >= 1U && SCAN_STEP_DEG <= 5U, "SCAN_STEP_DEG must be 1..5");
_Static_assert(SCAN_FIND_STEP_DEG >= SCAN_STEP_DEG && SCAN_FIND_STEP_DEG <= 10U,
	       "need SCAN_STEP_DEG <= SCAN_FIND_STEP_DEG <= 10");
_Static_assert(SCAN_SENSOR_OFFSET_MM <= 200U, "SCAN_SENSOR_OFFSET_MM out of range");
_Static_assert(SCAN_CLUSTER_MM >= 20U && SCAN_CLUSTER_MM <= 300U, "SCAN_CLUSTER_MM out of range");
_Static_assert(SCAN_LANE_MM <= 500U, "SCAN_LANE_MM out of range");
_Static_assert(SCAN_BEYOND_MM >= 100U && SCAN_BEYOND_MM <= 1000U, "SCAN_BEYOND_MM must be 100..1000");
_Static_assert(SCAN_MAX_OBSTACLE_MM >= 200U && SCAN_MAX_OBSTACLE_MM <= 900U,
	       "SCAN_MAX_OBSTACLE_MM must be 200..900");
_Static_assert(SCAN_CAR_WIDTH_MM >= 80U && SCAN_CAR_WIDTH_MM <= 250U, "SCAN_CAR_WIDTH_MM out of range");
_Static_assert(SCAN_CLEAR_MM <= 200U && SCAN_MARGIN_MM <= 200U, "SCAN_CLEAR_MM / SCAN_MARGIN_MM out of range");
_Static_assert(SCAN_SPEED_CM_S >= 5U && SCAN_SPEED_CM_S <= 20U, "SCAN_SPEED_CM_S must be 5..20");
_Static_assert(SCAN_MIN_DEPTH_MM <= 300U, "SCAN_MIN_DEPTH_MM must be 0..300");
_Static_assert(SCAN_PASS_STEP_MM >= 30U && SCAN_PASS_STEP_MM <= 200U, "SCAN_PASS_STEP_MM must be 30..200");
_Static_assert(SCAN_MAX_DEPTH_MM >= 100U && SCAN_MAX_DEPTH_MM <= 1000U, "SCAN_MAX_DEPTH_MM must be 100..1000");

/* With the bypass on, a side is free only if the car fits beside the
 * obstacle with the margin and the clearance on both sides of it. */
#define BYPASS_LANE_MM     (SCAN_CAR_WIDTH_MM + SCAN_MARGIN_MM + 2U * SCAN_CLEAR_MM)
#if SCAN_BYPASS && (SCAN_CAR_WIDTH_MM + SCAN_MARGIN_MM + 2U * SCAN_CLEAR_MM) > SCAN_LANE_MM
#define LANE_MM            BYPASS_LANE_MM
#else
#define LANE_MM            SCAN_LANE_MM
#endif

#define GO_PIN             20U
#define TAG                "[HWTEST][SPIN-WIDTH] "
#define START_DELAY_MS     3000U
#define LOOP_MS            10U
#define READ_TIMEOUT_MS    800U

static bool s_go_last_pressed;
static obstacle_event_message_t s_latest;
static bool s_have_latest;

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

/* Keep the obstacle task's event queue empty (a full queue faults it) and
 * remember the newest ahead reading. */
static void drain_events(void)
{
	for (;;) {
		obstacle_event_message_t event;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL);

		if (received != (INT)sizeof(event)) return;
		if (event.header.type == VEHICLE_MESSAGE_OBSTACLE_EVENT &&
		    event.obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING) {
			s_latest = event;
			s_have_latest = true;
		}
	}
}

/* obstacle_bypass io: the newest reading, GO = stop, drain while waiting. */
static bool io_latest_ahead(void *context, uint16_t *mm, uint32_t *timestamp_ms)
{
	(void)context;
	if (!s_have_latest) return false;
	*mm = s_latest.distance_mm;
	*timestamp_ms = s_latest.header.timestamp_ms;
	return true;
}

static bool io_abort(void *context)
{
	(void)context;
	return go_pressed_raw();
}

static void io_poll(void *context)
{
	(void)context;
	drain_events();
}

static void wait_ms(uint32_t ms)
{
	uint32_t t0 = now_ms();

	while (now_ms() - t0 < ms) {
		drain_events();
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

/* A distance measured at or after `after` (ms). */
static bool fresh_distance(uint32_t after, uint16_t *mm)
{
	uint32_t t0 = now_ms();

	for (;;) {
		drain_events();
		if (s_have_latest &&
		    (int32_t)(s_latest.header.timestamp_ms - after) >= 0) {
			*mm = s_latest.distance_mm;
			return true;
		}
		if (now_ms() - t0 > READ_TIMEOUT_MS) return false;
		tk_dly_tsk(LOOP_MS);
	}
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

void spin_width_hwtest_run(void)
{
	static const obs_task_config_t task_config = {
		50U, 500U, 1000U, 5000U, 500U, 300U,
		NULL, NULL, NULL, NULL
	};
	static const obstacle_bypass_io_t io = {
		io_latest_ahead, io_abort, io_poll, NULL, TAG
	};
	static obstacle_bypass_config_t cfg;
	obs_pico_port_config_t hardware;
	unsigned n = 0U;
	char b1[16];
	ER er;

	cfg = obstacle_bypass_default_config();
	cfg.track_mm = (float)SPIN_TRACK_MM;
	cfg.spin_pwm = (uint8_t)SPIN_PWM;
	cfg.spin_pwm_slow = (uint8_t)SPIN_PWM_SLOW;
	cfg.half_deg = (float)SCAN_HALF_DEG;
	cfg.step_deg = (float)SCAN_STEP_DEG;
	cfg.find_step_deg = (float)SCAN_FIND_STEP_DEG;
	cfg.sensor_offset_mm = (float)SCAN_SENSOR_OFFSET_MM;
	cfg.cluster_mm = (float)SCAN_CLUSTER_MM;
	cfg.lane_mm = (float)SCAN_LANE_MM;
	cfg.beyond_mm = (float)SCAN_BEYOND_MM;
	cfg.max_obstacle_mm = (float)SCAN_MAX_OBSTACLE_MM;
	cfg.trim_deg = (float)(SCAN_TRIM_DEG);
	cfg.true_width_mm = (float)SCAN_TRUE_WIDTH_MM;
	cfg.full_sweep = SCAN_FULL != 0;
	cfg.bypass = SCAN_BYPASS != 0;
	cfg.rejoin = SCAN_REJOIN != 0;
	cfg.car_width_mm = (float)SCAN_CAR_WIDTH_MM;
	cfg.clear_mm = (float)SCAN_CLEAR_MM;
	cfg.margin_mm = (float)SCAN_MARGIN_MM;
	cfg.speed_cm_s = (float)SCAN_SPEED_CM_S;
	cfg.min_depth_mm = (float)SCAN_MIN_DEPTH_MM;
	cfg.pass_step_mm = (float)SCAN_PASS_STEP_MM;
	cfg.max_depth_mm = (float)SCAN_MAX_DEPTH_MM;

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
	hardware = obs_pico_port_default_config();
	hardware.echo_voltage_verified_3v3_safe = true;
	hardware.servo_limits_verified = true;
	er = obs_pico_port_task_start(&hardware, &task_config);
	if (er < E_OK) fail_forever("ultrasonic task start", er);
	s_go_last_pressed = go_pressed_raw();

	tm_printf((UB*)TAG "spin-scan width test: find right edge in %u deg steps, measure leftwards "
		  "in %u deg steps (limit %u deg each way), lane %u mm, track %u mm, PWM %u/%u%%, "
		  "sensor %u mm ahead of the axle, trim %s deg%s\n",
		  (unsigned)SCAN_FIND_STEP_DEG, (unsigned)SCAN_STEP_DEG, (unsigned)SCAN_HALF_DEG,
		  (unsigned)LANE_MM, (unsigned)SPIN_TRACK_MM, (unsigned)SPIN_PWM,
		  (unsigned)SPIN_PWM_SLOW, (unsigned)SCAN_SENSOR_OFFSET_MM,
		  f1(b1, (float)(SCAN_TRIM_DEG)), SCAN_FULL ? ", FULL sweep" : "");
	tm_printf((UB*)TAG "SETUP: robot on the floor facing the obstacle, ~20 cm from the sensor, "
		  "obstacle centred; clear space around; servo points ahead\n");
	if (SCAN_BYPASS)
		tm_printf((UB*)TAG "BYPASS on: after the scan it turns 90 deg, drives up to 500 mm "
			  "sideways (car %u mm, clearance %u, margin %u, %u cm/s) and turns back%s; "
			  "put it back on the line before the next scan (SCAN_BYPASS=0 = scan only)\n",
			  (unsigned)SCAN_CAR_WIDTH_MM, (unsigned)SCAN_CLEAR_MM,
			  (unsigned)SCAN_MARGIN_MM, (unsigned)SCAN_SPEED_CM_S,
			  SCAN_REJOIN ? ", then drives past it (looking across at the line) and rejoins the line" : "");
	tm_printf((UB*)TAG "the same manoeuvre runs in the vehicle (obstacle_bypass.c)\n");
	tm_printf((UB*)TAG "each GO press runs one scan; GO during a move = STOP\n");

	for (;;) {
		uint16_t mm;
		obstacle_bypass_status_t st;

		tm_printf((UB*)TAG "---- scan %u: press GO ----\n", ++n);
		wait_for_go();
		if (fresh_distance(now_ms(), &mm)) {
			if (mm != OBS_DIST_NONE)
				tm_printf((UB*)TAG "ahead now: %u mm%s\n", (unsigned)mm,
					  mm < 170U || mm > 250U ? "  (aim for ~200 mm, the vehicle stop distance: the trim depends on distance)" : "");
			else
				tm_printf((UB*)TAG "ahead now: NONE (nothing in front?)\n");
		}
		tm_printf((UB*)TAG "turning in %u s, hands clear\n", START_DELAY_MS / 1000U);
		wait_ms(START_DELAY_MS);
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);
		st = obstacle_bypass_run(&cfg, &io);
		(void)motor_emergency_stop();
		if (st != OBSTACLE_BYPASS_OK)
			tm_printf((UB*)TAG "result: %s\n", obstacle_bypass_status_text(st));
		if (obstacle_bypass_is_fault(st)) {
			tm_printf((UB*)TAG "SCAN ENDED -- test stopped, reset to retry\n");
			for (;;) {
				drain_events();
				tk_dly_tsk(100U);
			}
		}
	}
}

#endif /* APP_SENSOR_TEST == 15 */
