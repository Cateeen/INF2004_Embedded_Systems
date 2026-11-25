/*
 * line_follow_hwtest.c - line following on the mat (SENSOR_TEST=line_follow).
 *
 * Tests the line_steering component on its own, in closed loop, without the
 * controller/safety/obstacle tasks: live IR readings -> line_following_position
 * -> line_steering_update (the REAL component) -> wheel targets
 * (left = speed - turn, right = speed + turn, as the motion task does) -> the
 * vehicle's own wheel speed PID (gains copied from motion_config in
 * app_main.c) -> motors. Encoders give wheel speeds and an x/y/heading track.
 *
 * It answers two questions:
 *   WIGGLE  - how often the steering flips LEFT<->RIGHT, how far the heading
 *             and the robot centre swing on a straight, how much of the time
 *             the robot is centred, and whether the IR edge flickers.
 *   CURVES  - the tightest radius the current SPEED/TURN can drive, how long
 *             the steering sits at full turn on one side (saturation), line
 *             losses, and how much of the commanded turn the wheels deliver.
 *
 * Each GO press runs once until LF_DIST_MM, a lost line (the component's own
 * 300 ms stop latch), or GO again (= STOP). Then it prints a report and waits
 * for the next GO, so you can run a straight, then a curve, then retune.
 * While waiting it shows the live IR reading so you can place the robot.
 *
 * Tune without editing code: make SENSOR_TEST=line_follow LF_SPEED=15 LF_TURN=4
 * (these override LINE_STEERING_SPEED_CM_S / LINE_STEERING_TURN_CM_S for this
 * build only; copy what works into line_steering.h).
 *
 * Note: the vehicle adds a little latency this test does not (line task 10 ms
 * + controller 50 ms + motion mailbox); raise LF_STEER_MS to mimic more.
 *
 * Safety: nothing moves until GO (GP20); GO during a run stops the motors.
 * Runs also stop on timeout, a stalled wheel, a wheel turning the wrong way,
 * or an encoder fault (those end the test; reset to retry).
 */
#include "../../app.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "../../subsystem3/line_barcode/line_adc_reader.h"
#include "../../subsystem3/line_barcode/line_observation.h"
#include "../motion/motor_driver.h"
#include "../motion/wheel_speed_control.h"
#include "line_steering.h"
#include "line_follow_hwtest.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 17

#ifndef LF_DIST_MM
#define LF_DIST_MM 1500U      /* run length; curves need room */
#endif
#ifndef LF_STEER_MS
#define LF_STEER_MS 50U       /* = controller period in app_main.c */
#endif
#ifndef LF_TRACE
#define LF_TRACE 1U           /* per-steering-update trace rows */
#endif
#ifndef LF_CURVE_R_MM
#define LF_CURVE_R_MM 0U      /* tightest curve radius on your mat; 0 = unknown */
#endif
#ifndef LF_TRACK_MM
#define LF_TRACK_MM 116U      /* effective track, calibrated with spin_heading */
#endif
#ifndef LF_FF
#define LF_FF 1.4             /* wheel feed-forward %PWM per cm/s (0 = flat base) */
#endif
#ifndef LF_BARCODE
#define LF_BARCODE 0          /* 1: also decode barcodes on the way (two decoders) */
#endif
#ifndef LF_BC_MODULE
#define LF_BC_MODULE 35U      /* counts per narrow module, = the vehicle's setting */
#endif
#ifndef LF_BC_SENSOR_Y_MM
#define LF_BC_SENSOR_Y_MM 40  /* barcode sensor left of the robot's centreline */
#endif
#ifndef LF_BC_SLOW
#define LF_BC_SLOW 0          /* cm/s over a barcode (straddle only); 0 = no slow-down */
#endif
#ifndef LF_BC_SLOW_MM
#define LF_BC_SLOW_MM 60U     /* back to full speed after this much white */
#endif

#ifndef LF_SLOPE
#define LF_SLOPE 0            /* 1: whole run on the SLOPE (ramp) profile; no IMU here */
#endif

/*
 * LF_STRADDLE=1: test-only "straddle" steering instead of line_steering.c
 * (the vehicle code is not touched): the line sensors sit further apart than
 * the tape, tape in the gap = straight, one sensor = gentle turn, held too
 * long = pivot back to the bend, both white too long = search. Logic and
 * LF_SL_* settings: straddle_hwtest.h. Speed and steering default to the
 * MAIN ROBOT's (vehicle_controller_task.h; LF_SPEED / LF_SL_* on the make
 * line override them); LF_TURN, LF_TURN_MIN, LF_RAMP_MS and LF_SLOPE are
 * unused in this mode.
 */
#include "straddle_hwtest.h"

#if LF_STRADDLE
#define SPEED  LF_SL_SPEED
#define TURN   LF_SL_TURN
#define TMIN   LF_SL_TURN
#define RAMP   0
#elif LF_SLOPE
#define SPEED  LINE_STEERING_SLOPE_SPEED_CM_S
#define TURN   LINE_STEERING_SLOPE_TURN_CM_S
#define TMIN   LINE_STEERING_SLOPE_TURN_MIN_CM_S
#define RAMP   LINE_STEERING_SLOPE_RAMP_MS
#else
#define SPEED  LINE_STEERING_SPEED_CM_S
#define TURN   LINE_STEERING_TURN_CM_S          /* full turn */
#define TMIN   LINE_STEERING_TURN_MIN_CM_S      /* first correction on a side */
#define RAMP   LINE_STEERING_RAMP_MS
#endif
#define VEHICLE_MAX_SPEED_CM_S 20   /* = app_main.c; PID verified up to here */

_Static_assert(TURN >= 1 && TURN < SPEED, "need 1 <= LF_TURN < LF_SPEED");
_Static_assert(SPEED + TURN <= VEHICLE_MAX_SPEED_CM_S,
	       "LF_SPEED + LF_TURN must be <= 20 cm/s (vehicle motion limit)");
_Static_assert(LF_DIST_MM >= 200U && LF_DIST_MM <= 5000U, "need 200 <= LF_DIST_MM <= 5000");
_Static_assert(LF_STEER_MS >= 10U && LF_STEER_MS <= 200U && LF_STEER_MS % 10U == 0U,
	       "LF_STEER_MS must be 10..200 in steps of 10");
_Static_assert(LF_TRACE <= 1U, "LF_TRACE must be 0 or 1");
_Static_assert(LF_TRACK_MM >= 80U && LF_TRACK_MM <= 200U, "need 80 <= LF_TRACK_MM <= 200");
_Static_assert((LF_FF) >= 0.0 && (LF_FF) <= 5.0, "need 0 <= LF_FF <= 5");
_Static_assert(LF_BC_SLOW == 0 || (LF_STRADDLE && LF_BARCODE && LF_BC_SLOW > TURN &&
				   LF_BC_SLOW <= SPEED),
	       "LF_BC_SLOW needs LF_STRADDLE=1 LF_BARCODE=1 and LF_SL_TURN < LF_BC_SLOW <= speed");
_Static_assert(LF_BC_SLOW_MM >= 10U && LF_BC_SLOW_MM <= 500U, "need 10 <= LF_BC_SLOW_MM <= 500");

/* Wheel speed PID: same as motion_config in app_main.c (MOT-H01). */
#define PID_KP             2.0f
#define PID_KI             4.0f
#define PID_KD             0.0f
#define PID_BASE           22U
#define PID_MAX            60U
#define PID_FF_REF_CM_S    15.0f   /* PID_BASE holds this speed */
#define PID_PERIOD_MS      50U     /* = odometry publish period */

/* Same calibration as the vehicle (app_main.c). */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define LEFT_SIGN          (-1)    /* left encoder counts down going forward */
#define RIGHT_SIGN         1
#define LEFT_MOTOR_FWD     1       /* motion_task.c MOTION_LEFT_POLARITY */
#define RIGHT_MOTOR_FWD    (-1)    /* motion_task.c MOTION_RIGHT_POLARITY */

#define GO_PIN             20U
#define TAG                "[HWTEST][LF] "
#define PI_F               3.14159265f
#define RAD2DEG            (180.0f / PI_F)
#define POLL_MS            10U     /* sensor, pose and safety tick */
#define START_DELAY_MS     2000U
#define SHOW_MS            500U    /* live IR display while waiting */
#define SETTLE_MS          500U
#define STALL_MS           1000U
#define STALL_MM           2.0f
#define WRONG_WAY_MM       20.0f
#define MAX_SAMPLES        600U    /* 30 s at 50 ms; later samples not stored */
#define CURVE_DEG          30.0f   /* net heading change that makes a run a "curve" */

/* Rough judgement thresholds; the raw numbers are printed regardless. */
#define WIGGLE_HDG_DEG     8.0f    /* straight: heading swing beyond +/- this */
#define WIGGLE_SWING_MM    20.0f   /* straight: centre swing (peak-to-peak) */
#define CENTRED_MIN_PCT    25.0f   /* JUNCTION (both black) share of on-line time */
#define SATURATED_MS       700U    /* one-sided full turn this long = at its limit */
#define INSIDE_DUTY_PCT    75.0f   /* curve: share of updates turning inward */
#define ACHIEVED_MIN_PCT   70.0f   /* delivered / commanded turn */
#define MIN_JUDGE_MS       1500U   /* shorter runs: numbers only, no wiggle/turn hints */
#define LOST_CROSS_RISE    150U    /* right raw above its white floor while LOST =
				    * tape crossed toward it (went wide: rise ~30;
				    * turned past the tape: rise ~440) */
#define MIN_WHEEL_CM_S     0.5f    /* PID needs targets > 0: "stopped" inner wheel */
#define STALL_EXEMPT_CM_S  3.0f    /* a wheel asked for less is not stall-checked */

/* Both white is LOST for the ramped steering, but "centred" for straddle. */
#define IS_LOST(p)         (!LF_STRADDLE && (p) == POS_LOST)

typedef enum { RUN_OK = 0, RUN_LINE_LOST, RUN_ABORT_GO, RUN_TIMEOUT, RUN_STALL_L,
	       RUN_STALL_R, RUN_WRONG_L, RUN_WRONG_R, RUN_ENCODER, RUN_SENSOR,
	       RUN_PID } run_status_t;

typedef struct {
	uint32_t t;
	float x, y, h;          /* mm, mm, rad (+ = left/CCW); robot centre */
	int16_t vl10, vr10;     /* measured wheel speed x10 cm/s */
	uint16_t raw_c, raw_r;
	uint8_t pos;
	int8_t turn;
} sample_t;

typedef struct {
	run_status_t status;
	uint32_t run_ms;
	float dist_mm;
	unsigned n;                     /* samples stored */
	unsigned updates;               /* steering updates */
	unsigned pos_count[4];          /* per POS_* */
	unsigned turn_left, turn_right; /* updates commanding each side */
	unsigned flips;                 /* LEFT<->RIGHT reversals */
	unsigned changes;               /* any change of steering command */
	unsigned sensor_changes;        /* position changes at the 10 ms tick */
	uint32_t longest_side_ms;
	int8_t longest_side;            /* +1 left, -1 right */
	int8_t side_at_end;             /* side being commanded when the run ended */
	unsigned lost_events;
	uint32_t lost_total_ms, lost_max_ms;
	float net_h, abs_h;             /* rad: net heading change, sum of |dh| */
	float achieved_sum;             /* sum of delivered turn while turning */
	float commanded_sum;            /* sum of |commanded turn| at the same samples */
	unsigned achieved_n;
	uint16_t c_min, c_max, r_min, r_max;
	uint16_t lost_right_peak;       /* highest right raw in the latest LOST stretch */
	uint16_t lost_centre_peak;      /* highest centre raw in the latest LOST stretch */
	/* LF_STRADDLE only */
	unsigned sl_corrections;        /* gentle corrections started */
	unsigned sl_recoveries;         /* overshoot pivots */
	unsigned sl_searches;           /* blind searches */
	unsigned sl_false_searches;     /* ... that found the tape on the OTHER sensor */
	unsigned sl_counters;           /* counter-steers after a correction */
	uint32_t sl_longest_correct_ms, sl_longest_recover_ms;
	uint8_t sl_state_at_end;
} run_result_t;

/* Wheel targets from a speed/turn command; a wheel asked to stop or reverse
 * is held at MIN_WHEEL_CM_S (the PID only drives forward, targets > 0). */
static bool set_targets(wheel_speed_control_t *pid, line_steering_command_t c,
			float *left, float *right)
{
	*left = (float)(c.linear_cm_s - c.turn_cm_s);
	*right = (float)(c.linear_cm_s + c.turn_cm_s);
	if (*left < MIN_WHEEL_CM_S) *left = MIN_WHEEL_CM_S;
	if (*right < MIN_WHEEL_CM_S) *right = MIN_WHEEL_CM_S;
	return wheel_speed_control_set_targets(pid, *left, *right);
}

static sample_t s_samples[MAX_SAMPLES];
static bool s_go_last_pressed;
static const char *const s_pos_name[4] = {
	[POS_JUNCTION] = "CENTRE", [POS_LEFT] = "LEFT", [POS_RIGHT] = "RIGHT",
	[POS_LOST] = "LOST"
};

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

static bool read_line(line_adc_reader_t *reader, ir_readings_t *ir, uint8_t *pos)
{
	uint32_t ts;

	if (line_adc_reader_read_line(reader, ir, &ts) != E_OK) return false;
#if LF_STRADDLE
	sl_hysteresis(ir);
#endif
	*pos = line_following_position(ir);
	return true;
}

#if LF_BARCODE
/*
 * Barcode decoding while following (LF_BARCODE=1). Three decoders read the
 * left (barcode) sensor on the same pass, all with the vehicle's barcode
 * settings (LF_BC_MODULE counts per narrow module, 25 % tolerance; the
 * barcode's real size is learned per frame):
 *   A "vehicle": exactly the vehicle's line task (line_barcode_task.c): a
 *     sample every 10 ms, hysteresis (black >= 1000, white <= 600), 4-count
 *     debounce, and the distance from odometry published every 50 ms and
 *     extrapolated at the last rate in between (sample_count);
 *   B "fast": a sample every 2 ms with fresh encoder counts, same hysteresis
 *     and debounce -- the best this sensor can do, the reference;
 *   C "fast + at sensor": as B, but the distance is the travel of the point
 *     under the barcode sensor (LF_BC_SENSOR_Y_MM left of the robot's
 *     centre), not of the centre: when the robot yaws, a point off the
 *     centreline speeds up and slows down (40 mm off, 30 deg/s of yaw is
 *     +/-21 mm/s on 120), which distorts bar widths. That point's travel is
 *     a weighted mix of the wheels: left * (1/2 + y/track) + right * (1/2 - y/track).
 * If B decodes and A does not, the vehicle's sampling is too slow for the
 * speed; if only C decodes, the wiggle (yaw) distorts the widths; if none
 * does, look at the widths, rejects and the wiggle.
 *
 * LF_BC_SLOW=<cm/s> (straddle): drop to that speed at the first black A sees
 * on the barcode sensor, back to full speed once it has read white for
 * LF_BC_SLOW_MM -- the slow-down the vehicle would do, applied at the
 * controller's 50 ms steering rate. Compare A's result with and without it.
 */
#define BC_FAST_MS        2U
#define BC_READ_US        200U    /* time one barcode ADC read takes, roughly */
#define BC_SLOW_MS        10U     /* = line task period_ms in app_main.c */
#define BC_ODOM_MS        50U     /* = odometry task period in app_main.c */
#define BC_ODOM_MAX_AGE   200U    /* = line task odometry_max_age_ms */
#define BC_MIN_SAMPLES    3.0f    /* vehicle samples per narrow bar for a safe margin */
#define BC_WHITE_ADC      600U
#define BC_BLACK_ADC      LINE_ADC_BARCODE_BLACK_THRESHOLD
#define BC_DEBOUNCE       4
#define BC_MM_PER_COUNT   (WHEEL_CIRC_MM / COUNTS_PER_REV)
#define BC_WEIGHT_LEFT    (0.5f + (float)(LF_BC_SENSOR_Y_MM) / (float)LF_TRACK_MM)
#define BC_DECODERS       3U

typedef struct {
	barcode_scanner_t scanner;
	bool black;
	bool deb_black, deb_pending;        /* B, C: debounce */
	int32_t deb_start;
	char decoded[BARCODE_MAX_PAYLOAD + 1U];
	unsigned chars, rejects;
} bc_decoder_t;

static const barcode_scanner_config_t s_bc_config = {LF_BC_MODULE, 25U, 0U, 0U, 0U};
static bc_decoder_t s_bc[BC_DECODERS];  /* [0] A vehicle, [1] B fast, [2] C at sensor */
static const char *const s_bc_name[BC_DECODERS] = {
	"A vehicle 10ms", "B fast 2ms", "C fast, at sensor"
};
static uint32_t s_bc_last_slow_ms;
/* Decoder A's distance, as the vehicle's line task gets it. */
static int32_t s_odo_count;
static uint32_t s_odo_ms;
static float s_odo_rate;                /* counts per ms over the last period */
/* LF_BC_SLOW: slowed down over a barcode. */
static bool s_bc_slow;
static int32_t s_bc_white_start;
static unsigned s_bc_slowdowns;
static float s_bc_slow_mm;
static uint16_t s_bc_adc_min, s_bc_adc_max;
/* While either decoder is inside a barcode: how the robot moved. */
static bool s_bc_was_over;
static bool s_bc_any_edge;
static uint32_t s_bc_last_seq;
static int32_t s_bc_now_count, s_bc_last_edge_count;
static uint32_t s_bc_last_track_ms;
#define BC_OVER_GAP_MM    20.0f   /* "over the barcode": within this of a bar edge */
static unsigned s_bc_passes, s_bc_turn_changes;
static int s_bc_turn_max, s_bc_last_turn;
static float s_bc_h_min, s_bc_h_max, s_bc_over_mm, s_bc_over_ms;

/* Forward-positive wheel counts: the robot centre's (mean) and the point
 * under the barcode sensor's (weighted). Left counts down going forward. */
static bool bc_counts(int32_t *mean, int32_t *at_sensor)
{
	encoder_snapshot_t s;
	float l, r;

	if (encoder_source_snapshot(&s) < E_OK || !s.valid) return false;
	l = (float)(LEFT_SIGN * s.left_count);
	r = (float)(RIGHT_SIGN * s.right_count);
	*mean = (int32_t)(((int64_t)LEFT_SIGN * s.left_count +
			   (int64_t)RIGHT_SIGN * s.right_count) / 2);
	*at_sensor = (int32_t)__builtin_lroundf(l * BC_WEIGHT_LEFT + r * (1.0f - BC_WEIGHT_LEFT));
	return true;
}

/* B and C: debounce that places an accepted edge where the change began. */
static bool bc_fast_update(unsigned i, bool black, int32_t count, int32_t *edge_out)
{
	bc_decoder_t *d = &s_bc[i];
	int32_t edge = count;

	if (black == d->deb_black) {
		d->deb_pending = false;
	} else {
		if (!d->deb_pending) {
			d->deb_pending = true;
			d->deb_start = count;
		}
		if ((int32_t)((uint32_t)count - (uint32_t)d->deb_start) >= BC_DEBOUNCE ||
		    (int32_t)((uint32_t)d->deb_start - (uint32_t)count) >= BC_DEBOUNCE) {
			d->deb_black = black;
			d->deb_pending = false;
			edge = d->deb_start;
		}
	}
	*edge_out = edge;
	return barcode_scanner_update(&d->scanner, d->deb_black, edge);
}

static bool bc_start(line_adc_reader_t *reader)
{
	ir_readings_t ir;
	uint32_t ts;
	int32_t count, at_sensor;
	unsigned i;

	if (line_adc_reader_read_barcode(reader, &ir, &ts) != E_OK ||
	    !bc_counts(&count, &at_sensor))
		return false;
	for (i = 0U; i < BC_DECODERS; i++) {
		s_bc[i] = (bc_decoder_t){0};
		s_bc[i].black = s_bc[i].deb_black = ir.barcode.raw >= BC_BLACK_ADC;
		if (!barcode_scanner_init(&s_bc[i].scanner, &s_bc_config, s_bc[i].black,
					  i == 2U ? at_sensor : count))
			return false;
	}
	s_bc_last_slow_ms = now_ms();
	s_bc_last_track_ms = s_bc_last_slow_ms;
	s_odo_count = count;
	s_odo_ms = s_bc_last_slow_ms;
	s_odo_rate = 0.0f;
	s_bc_slow = false;
	s_bc_white_start = count;
	s_bc_slowdowns = 0U;
	s_bc_slow_mm = 0.0f;
	s_bc_any_edge = false;
	s_bc_last_seq = s_bc[1].scanner.edge.sequence;
	s_bc_now_count = s_bc_last_edge_count = count;
	s_bc_adc_min = 0xFFFFU;
	s_bc_adc_max = 0U;
	s_bc_was_over = false;
	s_bc_passes = s_bc_turn_changes = 0U;
	s_bc_turn_max = 0;
	s_bc_over_mm = s_bc_over_ms = 0.0f;
	return true;
}

static void bc_events(unsigned i)
{
	barcode_char_info_t info;
	uint8_t payload[BARCODE_MAX_PAYLOAD];
	uint8_t length;

	while (barcode_scanner_take_event(&s_bc[i].scanner, &info)) {
		if (info.reject == BARCODE_REJECT_NONE) s_bc[i].chars++;
		else s_bc[i].rejects++;
		tm_printf((UB*)TAG "  [%c] char=%c %s pattern=%s widths(counts)=%u %u %u %u %u %u "
			  "%u %u %u\n", 'A' + (int)i, info.symbol ? info.symbol : '?',
			  barcode_reject_name(info.reject),
			  info.raw_pattern[0] ? info.raw_pattern : "-",
			  (unsigned)info.run_counts[0], (unsigned)info.run_counts[1],
			  (unsigned)info.run_counts[2], (unsigned)info.run_counts[3],
			  (unsigned)info.run_counts[4], (unsigned)info.run_counts[5],
			  (unsigned)info.run_counts[6], (unsigned)info.run_counts[7],
			  (unsigned)info.run_counts[8]);
	}
	if (barcode_scanner_take_result(&s_bc[i].scanner, payload, BARCODE_MAX_PAYLOAD, &length)) {
		unsigned k;

		for (k = 0U; k < length; k++) s_bc[i].decoded[k] = (char)payload[k];
		s_bc[i].decoded[length] = '\0';
		tm_printf((UB*)TAG "  [%c] DECODED \"%s\"\n", 'A' + (int)i, s_bc[i].decoded);
	}
}

/* One 2 ms sample: decoders B and C every time, decoder A every 10 ms. */
static bool bc_sample(line_adc_reader_t *reader)
{
	ir_readings_t ir;
	uint32_t ts, now;
	int32_t count, at_sensor, edge, edge_c;
	uint16_t raw;
	bool black;

	if (line_adc_reader_read_barcode(reader, &ir, &ts) != E_OK ||
	    !bc_counts(&count, &at_sensor))
		return false;
	raw = ir.barcode.raw;
	if (raw < s_bc_adc_min) s_bc_adc_min = raw;
	if (raw > s_bc_adc_max) s_bc_adc_max = raw;

	/* B and C: hysteresis, then the debounce; C measures at the sensor. */
	black = raw >= BC_BLACK_ADC ? true : (raw <= BC_WHITE_ADC ? false : s_bc[1].black);
	s_bc[1].black = s_bc[2].black = black;
	if (!bc_fast_update(1U, black, count, &edge)) return false;
	if (!bc_fast_update(2U, black, at_sensor, &edge_c)) return false;
	bc_events(2U);
	s_bc_now_count = count;
	if (s_bc[1].scanner.edge.sequence != s_bc_last_seq) {
		s_bc_last_seq = s_bc[1].scanner.edge.sequence;
		s_bc_last_edge_count = edge;
		s_bc_any_edge = true;
	}
	bc_events(1U);

	/* A: what the vehicle sees. Odometry arrives every 50 ms; between
	 * messages the line task extrapolates at the last measured rate. */
	now = now_ms();
	if ((uint32_t)(now - s_odo_ms) >= BC_ODOM_MS) {
		s_odo_rate = (float)(int32_t)((uint32_t)count - (uint32_t)s_odo_count) /
			     (float)(uint32_t)(now - s_odo_ms);
		s_odo_count = count;
		s_odo_ms = now;
	}
	if ((uint32_t)(now - s_bc_last_slow_ms) >= BC_SLOW_MS) {
		uint32_t since = now - s_odo_ms;
		int32_t vehicle_count, edge_a;

		if (since > BC_ODOM_MAX_AGE) since = BC_ODOM_MAX_AGE;
		vehicle_count = (int32_t)((uint32_t)s_odo_count +
			(uint32_t)(int32_t)__builtin_lroundf(s_odo_rate * (float)since));
		s_bc_last_slow_ms = now;
		s_bc[0].black = raw >= BC_BLACK_ADC ? true :
				(raw <= BC_WHITE_ADC ? false : s_bc[0].black);
		if (!bc_fast_update(0U, s_bc[0].black, vehicle_count, &edge_a)) return false;
		bc_events(0U);
	}

	/* LF_BC_SLOW: slow from the first black the vehicle's reading sees,
	 * until it has read white for LF_BC_SLOW_MM (true travel). */
	if (LF_BC_SLOW > 0) {
		if (s_bc[0].deb_black) {
			if (!s_bc_slow) s_bc_slowdowns++;
			s_bc_slow = true;
			s_bc_white_start = count;
		} else if (s_bc_slow) {
			int32_t moved = (int32_t)((uint32_t)count - (uint32_t)s_bc_white_start);

			if ((float)(moved < 0 ? -moved : moved) * BC_MM_PER_COUNT >=
			    (float)LF_BC_SLOW_MM)
				s_bc_slow = false;
		}
	}
	return true;
}

/* Straight/correcting speed for the next steering update. */
static int16_t bc_speed(void)
{
	return (LF_BC_SLOW > 0 && s_bc_slow) ? (int16_t)LF_BC_SLOW : (int16_t)SPEED;
}

/* Mean narrow width in mm over the frames decoder i read (0 if none). */
static float bc_narrow_mm(unsigned i)
{
	const barcode_stats_t *st = barcode_scanner_get_stats(&s_bc[i].scanner);

	return st->narrow.count ? (float)st->narrow.sum / (float)st->narrow.count *
				  BC_MM_PER_COUNT : 0.0f;
}

/* Called every line tick (time t ms) with the heading (rad), travel since
 * the last call (mm) and the steering. */
static void bc_track(float h, float ds, int turn, uint32_t t)
{
	int32_t since = (int32_t)((uint32_t)s_bc_now_count - (uint32_t)s_bc_last_edge_count);
	bool over = s_bc_any_edge &&
		    (float)(since < 0 ? -since : since) * BC_MM_PER_COUNT < BC_OVER_GAP_MM;
	uint32_t dt = t - s_bc_last_track_ms;

	s_bc_last_track_ms = t;
	if (s_bc_slow) s_bc_slow_mm += ds;
	if (!over) {
		s_bc_was_over = false;
		return;
	}
	if (!s_bc_was_over) {
		s_bc_was_over = true;
		s_bc_passes++;
		if (s_bc_passes == 1U) {
			s_bc_h_min = s_bc_h_max = h;
			s_bc_last_turn = turn;
		}
	}
	if (h < s_bc_h_min) s_bc_h_min = h;
	if (h > s_bc_h_max) s_bc_h_max = h;
	if ((turn < 0 ? -turn : turn) > s_bc_turn_max) s_bc_turn_max = turn < 0 ? -turn : turn;
	if (turn != s_bc_last_turn) s_bc_turn_changes++;
	s_bc_last_turn = turn;
	s_bc_over_mm += ds;
	s_bc_over_ms += (float)dt;
}

static void bc_report(void)
{
	float speed = s_bc_over_ms > 0.0f ? s_bc_over_mm / s_bc_over_ms * 100.0f : 0.0f;
	/* Narrow bar as measured (B is the most accurate); else the config. */
	float module_mm = bc_narrow_mm(1U) > 0.0f ? bc_narrow_mm(1U) :
			  (bc_narrow_mm(0U) > 0.0f ? bc_narrow_mm(0U) :
			   (float)LF_BC_MODULE * BC_MM_PER_COUNT);
	char b1[16], b2[16], b3[16], b4[16];
	unsigned i;

	tm_printf((UB*)TAG "  BARCODE (narrow bar measured %s mm; left sensor raw %u..%u, black "
		  ">= %u, white <= %u)\n", f1(b1, module_mm), (unsigned)s_bc_adc_min,
		  (unsigned)s_bc_adc_max, (unsigned)BC_BLACK_ADC, (unsigned)BC_WHITE_ADC);
	if (LF_BC_SLOW > 0)
		tm_printf((UB*)TAG "  slow-down: %u x to %u cm/s, %s mm driven slowly (full speed "
			  "%u cm/s)\n", s_bc_slowdowns, (unsigned)LF_BC_SLOW, f1(b1, s_bc_slow_mm),
			  (unsigned)SPEED);
	for (i = 0U; i < BC_DECODERS; i++) {
		const barcode_stats_t *st = barcode_scanner_get_stats(&s_bc[i].scanner);
		barcode_scanner_config_t suggested;

		tm_printf((UB*)TAG "    %-14s: %s%s%s  (%u chars ok, %u rejected)\n", s_bc_name[i],
			  s_bc[i].decoded[0] ? "DECODED \"" : "nothing decoded",
			  s_bc[i].decoded, s_bc[i].decoded[0] ? "\"" : "", s_bc[i].chars,
			  s_bc[i].rejects);
		if (st->narrow.count != 0U || st->wide.count != 0U)
			tm_printf((UB*)TAG "      narrow %u..%u, wide %u..%u counts\n",
				  (unsigned)(st->narrow.count ? st->narrow.min : 0U),
				  (unsigned)(st->narrow.count ? st->narrow.max : 0U),
				  (unsigned)(st->wide.count ? st->wide.min : 0U),
				  (unsigned)(st->wide.count ? st->wide.max : 0U));
		if (barcode_scanner_suggest_config(&s_bc[i].scanner, &suggested))
			tm_printf((UB*)TAG "      mean narrow width %u counts (%s mm)\n",
				  (unsigned)suggested.counts_per_module,
				  f1(b1, (float)suggested.counts_per_module * BC_MM_PER_COUNT));
	}
	if (s_bc_passes == 0U) {
		tm_printf((UB*)TAG "  HINT: no barcode was seen. The left sensor needs raw > %u on the "
			  "bars; check it reaches the barcode (2 cm left of the line).\n",
			  (unsigned)BC_BLACK_ADC);
		return;
	}
	tm_printf((UB*)TAG "  over the barcode: %s mm at %s cm/s; heading swung %s deg; steering "
		  "changed %u x, up to +/-%d cm/s\n", f1(b1, s_bc_over_mm), f1(b2, speed),
		  f1(b3, (s_bc_h_max - s_bc_h_min) * RAD2DEG), s_bc_turn_changes, s_bc_turn_max);
	if (speed > 0.0f) {
		float a_samples = module_mm / speed * 100.0f / (float)BC_SLOW_MS;
		/* cm/s at which the vehicle gets BC_MIN_SAMPLES per narrow bar */
		float safe = module_mm / (BC_MIN_SAMPLES * (float)BC_SLOW_MS) * 100.0f;

		tm_printf((UB*)TAG "  at %s cm/s a narrow bar lasts %s ms: A (vehicle) gets ~%s samples "
			  "per narrow bar, B ~%s\n", f1(b1, speed), f1(b2, module_mm / speed * 100.0f),
			  f1(b3, a_samples),
			  f1(b4, module_mm / speed * 100.0f / (float)BC_FAST_MS));
		tm_printf((UB*)TAG "  SPEED: for >= %u vehicle samples per narrow bar this barcode needs "
			  "<= %s cm/s over it -> %s\n", (unsigned)BC_MIN_SAMPLES, f1(b1, safe),
			  a_samples >= BC_MIN_SAMPLES ? "no slow-down needed for this size" :
			  (LF_BC_SLOW > 0 ? "still too fast: lower LF_BC_SLOW" :
			   "SLOW DOWN over barcodes (try LF_BC_SLOW below that)"));
	}
	if (s_bc[2].decoded[0] && !s_bc[1].decoded[0])
		tm_printf((UB*)TAG "  VERDICT: only C decodes -- the yaw (wiggle) distorts the bar "
			  "widths at the off-centre sensor; measure distance at the sensor.\n");
	else if (s_bc[1].decoded[0] && !s_bc[0].decoded[0])
		tm_printf((UB*)TAG "  VERDICT: the fast reading decodes, the vehicle's 10 ms reading "
			  "does not -- too fast for the vehicle's sampling%s.\n",
			  LF_BC_SLOW > 0 ? " even with the slow-down" :
			  "; try LF_BC_SLOW (e.g. 5)");
	else if (s_bc[0].decoded[0])
		tm_printf((UB*)TAG "  VERDICT: the vehicle's reading decodes at %s.\n",
			  LF_BC_SLOW > 0 ? "this slow-down" : "full line-following speed");
	else if ((s_bc_h_max - s_bc_h_min) * RAD2DEG > 10.0f)
		tm_printf((UB*)TAG "  VERDICT: neither decodes and the heading swung over 10 deg over "
			  "the barcode -- wiggle is a likely cause (compare the widths above).\n");
	else
		tm_printf((UB*)TAG "  VERDICT: neither decodes. Compare the widths with %u per narrow "
			  "module (wide ~2-3x); a smaller LF_BC_MODULE, or slower LF_SPEED / "
			  "LF_BC_SLOW.\n", (unsigned)LF_BC_MODULE);
}
#endif /* LF_BARCODE */

/* Waits for GO, printing the live IR reading so the robot can be placed. */
static void wait_for_go(line_adc_reader_t *reader)
{
	uint32_t last_show = now_ms() - SHOW_MS;

	while (!go_edge()) {
		ir_readings_t ir;
		uint8_t pos;

		if ((uint32_t)(now_ms() - last_show) >= SHOW_MS) {
			last_show = now_ms();
			if (!read_line(reader, &ir, &pos)) fail_forever("IR read", E_IO);
			tm_printf((UB*)TAG "  IR centre %4u %s  right %4u %s  -> %s\n",
				  (unsigned)ir.center.raw, ir.center.black ? "BLK" : "wht",
				  (unsigned)ir.right.raw, ir.right.black ? "BLK" : "wht",
				  s_pos_name[pos]);
		}
		tk_dly_tsk(POLL_MS);
	}
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

static void drive(uint8_t pwm_l, uint8_t pwm_r)
{
	/* E_BUSY = reversal brake interval; the next tick retries. */
	(void)motor_set_speed(MOTOR_1, (int16_t)((int)pwm_l * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)((int)pwm_r * RIGHT_MOTOR_FWD));
}

static int8_t sign_of(int v) { return (int8_t)(v > 0 ? 1 : (v < 0 ? -1 : 0)); }

static void note_raw(run_result_t *res, const ir_readings_t *ir)
{
	if (ir->center.raw < res->c_min) res->c_min = ir->center.raw;
	if (ir->center.raw > res->c_max) res->c_max = ir->center.raw;
	if (ir->right.raw < res->r_min) res->r_min = ir->right.raw;
	if (ir->right.raw > res->r_max) res->r_max = ir->right.raw;
}

static run_status_t run_once(line_adc_reader_t *reader, wheel_speed_control_t *pid,
			     run_result_t *res)
{
	const uint32_t timeout_ms =
		3000U + (uint32_t)(2.0f * (float)LF_DIST_MM / ((float)SPEED * 10.0f) * 1000.0f);
	line_steering_t steering;
#if LF_STRADDLE
	straddle_t straddle;
#endif
	line_steering_command_t cmd;
	float tgt_l = (float)SPEED, tgt_r = (float)SPEED;
	wheel_speed_output_t out = { PID_BASE, PID_BASE };
	ir_readings_t ir;
	uint8_t pos, pos_tick;
	float l0, r0, l, r, tl, tr, prog_l = 0.0f, prog_r = 0.0f;
	float x = 0.0f, y = 0.0f, h = 0.0f, h_last_steer = 0.0f;
	float sl = 0.0f, sr = 0.0f, pid_l, pid_r;
	uint32_t t0, t, next_steer, next_pid, last_pid, prog_l_ms = 0U, prog_r_ms = 0U;
	uint32_t side_start = 0U, lost_start = 0U;
	int8_t side = 0, last_nonzero = 0;
	int16_t last_turn;
	bool in_lost = false;
	run_status_t status = RUN_OK;
	char b1[16], b2[16], b3[16];

	*res = (run_result_t){ 0 };
	res->c_min = res->r_min = 0xFFFFU;
	if (!read_line(reader, &ir, &pos)) return RUN_SENSOR;
	if (IS_LOST(pos)) {
		tm_printf((UB*)TAG "  NOT ON THE LINE (both sensors white) -- place it so the "
			  "display reads CENTRE, LEFT or RIGHT\n");
		return RUN_LINE_LOST;
	}
	if (LF_STRADDLE && pos != POS_LOST)
		tm_printf((UB*)TAG "  note: straddle mode starts best with the tape BETWEEN the "
			  "sensors (display reads LOST = both white)\n");
	if (!read_mm(&l0, &r0)) return RUN_ENCODER;
	l = tl = pid_l = l0;
	r = tr = pid_r = r0;

	line_steering_reset(&steering);
	line_steering_set_slope(&steering, LF_SLOPE != 0);
#if LF_STRADDLE
	sl_reset(&straddle, SPEED);
#endif
#if LF_BARCODE
	if (!bc_start(reader)) return RUN_SENSOR;
#endif
	t0 = now_ms();
#if LF_STRADDLE
	cmd = sl_update(&straddle, pos, t0);
#else
	cmd = line_steering_update(&steering, pos, t0);
#endif
	if (cmd.stop_latched || !wheel_speed_control_reset(pid) ||
	    !set_targets(pid, cmd, &tgt_l, &tgt_r))
		return RUN_PID;
	last_turn = cmd.turn_cm_s;
	pos_tick = pos;
	next_steer = t0 + LF_STEER_MS;
	last_pid = t0;
	next_pid = t0 + PID_PERIOD_MS;
	drive(out.left_pwm_percent, out.right_pwm_percent);
	if (LF_TRACE)
		tm_printf((UB*)TAG "  t_ms  pos     turn  IRc  IRr   L cm/s R cm/s  hdg deg%s\n",
			  LF_STRADDLE ? "  state" : "");

	for (;;) {
		float dl, dr, ds, dh;
		uint8_t p;

#if LF_BARCODE
		/* The barcode sensor every 2 ms inside the 10 ms line tick. The
		 * kernel tick is 10 ms (CNF_TIMER_PERIOD), so tk_dly_tsk(2) would
		 * wait a whole tick or two: busy-wait instead (~0.2 ms per read). */
		for (unsigned k = 0U; k < POLL_MS / BC_FAST_MS && status == RUN_OK; k++) {
			WaitUsec(BC_FAST_MS * 1000U - BC_READ_US);
			if (!bc_sample(reader)) status = RUN_SENSOR;
		}
		if (status != RUN_OK) break;
#else
		tk_dly_tsk(POLL_MS);
#endif
		t = now_ms();
		if (!read_mm(&l, &r)) { status = RUN_ENCODER; break; }
		if (!read_line(reader, &ir, &p)) { status = RUN_SENSOR; break; }
		note_raw(res, &ir);
		if (p != pos_tick) res->sensor_changes++;
		pos_tick = p;
		/* Which way the tape went when it was lost: a rise on the sensor the
		 * robot was turning AWAY from means the tape crossed toward it (turned
		 * past the line / line ended), not that it slid off the outside. */
		if (p != POS_LOST) {
			res->lost_right_peak = res->lost_centre_peak = 0U;
		} else {
			if (ir.right.raw > res->lost_right_peak)
				res->lost_right_peak = ir.right.raw;
			if (ir.center.raw > res->lost_centre_peak)
				res->lost_centre_peak = ir.center.raw;
		}

		/* Pose of the robot centre (midpoint dead reckoning). */
		dl = l - tl;
		dr = r - tr;
		tl = l;
		tr = r;
		ds = (dl + dr) * 0.5f;
		dh = (dr - dl) / (float)LF_TRACK_MM;
		x += ds * __builtin_cosf(h + dh * 0.5f);
		y += ds * __builtin_sinf(h + dh * 0.5f);
		h += dh;
#if LF_BARCODE
		bc_track(h, ds, cmd.turn_cm_s, t);
#endif

		res->run_ms = t - t0;
		res->dist_mm = ((l - l0) + (r - r0)) * 0.5f;
		if (go_pressed_raw()) { status = RUN_ABORT_GO; break; }
		if (res->run_ms > timeout_ms) { status = RUN_TIMEOUT; break; }
		if (l - l0 < -WRONG_WAY_MM) { status = RUN_WRONG_L; break; }
		if (r - r0 < -WRONG_WAY_MM) { status = RUN_WRONG_R; break; }
		if (absf((l - l0) - prog_l) >= STALL_MM) { prog_l = l - l0; prog_l_ms = res->run_ms; }
		if (absf((r - r0) - prog_r) >= STALL_MM) { prog_r = r - r0; prog_r_ms = res->run_ms; }
		/* The inner wheel of a pivot is meant to stand still. */
		if (tgt_l < STALL_EXEMPT_CM_S) prog_l_ms = res->run_ms;
		if (tgt_r < STALL_EXEMPT_CM_S) prog_r_ms = res->run_ms;
		if (res->run_ms - prog_l_ms > STALL_MS) { status = RUN_STALL_L; break; }
		if (res->run_ms - prog_r_ms > STALL_MS) { status = RUN_STALL_R; break; }
		if (res->dist_mm >= (float)LF_DIST_MM) break;

		/* Steering: the real component, at the controller's rate. */
		if ((int32_t)(t - next_steer) >= 0) {
			int8_t s;

			next_steer += LF_STEER_MS;
			pos = p;
#if LF_STRADDLE
#if LF_BARCODE
			/* LF_BC_SLOW: slower over a barcode, as the vehicle would. */
			if (straddle.config.speed_cm_s != bc_speed())
				(void)straddle_set_speed(&straddle, bc_speed());
#endif
			cmd = sl_update(&straddle, pos, t);
#else
			cmd = line_steering_update(&steering, pos, t);
#endif
			if (cmd.stop_latched) { status = RUN_LINE_LOST; break; }
			if (!set_targets(pid, cmd, &tgt_l, &tgt_r)) {
				status = RUN_PID;
				break;
			}

			res->updates++;
			res->pos_count[pos & 3U]++;
			if (cmd.turn_cm_s != last_turn) res->changes++;
			last_turn = cmd.turn_cm_s;
			s = sign_of(cmd.turn_cm_s);
			if (s > 0) res->turn_left++;
			if (s < 0) res->turn_right++;
			if (s != 0) {
				if (last_nonzero != 0 && s != last_nonzero) res->flips++;
				last_nonzero = s;
			}
			if (s != side) {
				side = s;
				side_start = t;
			}
			if (side != 0 && t - side_start > res->longest_side_ms) {
				res->longest_side_ms = t - side_start;
				res->longest_side = side;
			}
			if (IS_LOST(pos) && !in_lost) {
				in_lost = true;
				lost_start = t;
				res->lost_events++;
			} else if (!IS_LOST(pos) && in_lost) {
				in_lost = false;
				res->lost_total_ms += t - lost_start;
				if (t - lost_start > res->lost_max_ms) res->lost_max_ms = t - lost_start;
			}
			res->abs_h += absf(h - h_last_steer);
			h_last_steer = h;

			if (res->n < MAX_SAMPLES)
				s_samples[res->n++] = (sample_t){
					res->run_ms, x, y, h,
					(int16_t)__builtin_lroundf(sl * 10.0f),
					(int16_t)__builtin_lroundf(sr * 10.0f),
					ir.center.raw, ir.right.raw, pos, (int8_t)cmd.turn_cm_s };
			if (LF_TRACE) {
#if LF_STRADDLE
				const char *state_name = sl_state_name(straddle.state);
#else
				const char *state_name = "";
#endif
				tm_printf((UB*)TAG "  %5u %-7s %+3d  %4u %4u  %6s %6s  %7s  %s\n",
					  (unsigned)res->run_ms, s_pos_name[pos & 3U],
					  (int)cmd.turn_cm_s, (unsigned)ir.center.raw,
					  (unsigned)ir.right.raw, f1(b1, sl), f1(b2, sr),
					  f1(b3, h * RAD2DEG), state_name);
			}
		}

		/* Wheel PID, at the odometry rate, as the motion task runs it. */
		if ((int32_t)(t - next_pid) >= 0) {
			float dt = (float)(t - last_pid) / 1000.0f;

			next_pid += PID_PERIOD_MS;
			sl = (l - pid_l) / 10.0f / dt;
			sr = (r - pid_r) / 10.0f / dt;
			pid_l = l;
			pid_r = r;
			last_pid = t;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (!wheel_speed_control_update(pid, sl, sr, dt, &out)) {
				status = RUN_PID;
				break;
			}
			if (cmd.turn_cm_s != 0) {
				res->achieved_sum += (sr - sl) * 0.5f * (float)sign_of(cmd.turn_cm_s);
				res->commanded_sum += (float)(cmd.turn_cm_s < 0 ? -cmd.turn_cm_s :
							      cmd.turn_cm_s);
				res->achieved_n++;
			}
		}
		drive(out.left_pwm_percent, out.right_pwm_percent);
	}
	(void)motor_brake_all();
#if LF_STRADDLE
	res->sl_corrections = straddle.corrections;
	res->sl_recoveries = straddle.recoveries;
	res->sl_searches = straddle.searches;
	res->sl_false_searches = straddle.false_searches;
	res->sl_counters = straddle.counters;
	res->sl_longest_correct_ms = straddle.longest_correct_ms;
	res->sl_longest_recover_ms = straddle.longest_recover_ms;
	res->sl_state_at_end = (uint8_t)straddle.state;
#endif
	if (in_lost) {
		uint32_t d = res->run_ms - (lost_start - t0);
		res->lost_total_ms += d;
		if (d > res->lost_max_ms) res->lost_max_ms = d;
	}
	res->side_at_end = side;
	res->net_h = h;
	res->status = status;
	tk_dly_tsk(SETTLE_MS);
	return status;
}

/* Straight runs: swing of the centre and the heading about the start->end chord. */
static void straight_swing(const run_result_t *res, float *swing_mm, float *hdg_dev_deg)
{
	const sample_t *a = &s_samples[0], *b = &s_samples[res->n - 1U];
	float cx = b->x - a->x, cy = b->y - a->y;
	float len = __builtin_sqrtf(cx * cx + cy * cy);
	float chord_h, lo = 1e9f, hi = -1e9f, hmax = 0.0f;
	unsigned i;

	*swing_mm = 0.0f;
	*hdg_dev_deg = 0.0f;
	if (res->n < 5U || len < 50.0f) return;
	cx /= len;
	cy /= len;
	chord_h = __builtin_atan2f(cy, cx);
	for (i = 0U; i < res->n; i++) {
		const sample_t *s = &s_samples[i];
		float d = (s->x - a->x) * cy - (s->y - a->y) * cx;
		float e = absf(s->h - chord_h);

		if (d < lo) lo = d;
		if (d > hi) hi = d;
		if (e > hmax) hmax = e;
	}
	*swing_mm = hi - lo;
	*hdg_dev_deg = hmax * RAD2DEG;
}

static const char *status_text(run_status_t s)
{
	switch (s) {
	case RUN_OK:        return "distance reached";
	case RUN_LINE_LOST: return LF_STRADDLE ?
				   "LINE LOST (pivot/search found nothing in LF_SL_PIVOT_MAX_MS)" :
				   "LINE LOST > 300 ms (steering stop latch)";
	case RUN_ABORT_GO:  return "stopped by GO button";
	case RUN_TIMEOUT:   return "timeout: distance not reached";
	case RUN_STALL_L:   return "LEFT wheel stalled";
	case RUN_STALL_R:   return "RIGHT wheel stalled";
	case RUN_WRONG_L:   return "LEFT wheel turned the WRONG WAY: motor or encoder sign";
	case RUN_WRONG_R:   return "RIGHT wheel turned the WRONG WAY: motor or encoder sign";
	case RUN_ENCODER:   return "encoder INVALID";
	case RUN_SENSOR:    return "IR ADC read failed";
	case RUN_PID:       return "PID rejected its inputs";
	default:            return "?";
	}
}

static void report(const run_result_t *res)
{
	const float min_radius = (float)LF_TRACK_MM * (float)SPEED / (2.0f * (float)TURN);
	const float metres = res->dist_mm / 1000.0f;
	const float secs = (float)res->run_ms / 1000.0f;
	const unsigned on_line = LF_STRADDLE ? res->updates :
				 res->updates - res->pos_count[POS_LOST];
	const float net_deg = res->net_h * RAD2DEG;
	const bool curve = absf(net_deg) >= CURVE_DEG;
	/* Centred: both black (ramped edge following), both white (straddle). */
	float centred = on_line ? 100.0f *
		(float)res->pos_count[LF_STRADDLE ? POS_LOST : POS_JUNCTION] / (float)on_line : 0.0f;
	float achieved = res->commanded_sum > 0.0f ?
		res->achieved_sum / res->commanded_sum * 100.0f : 0.0f;
	float zigzag = metres > 0.05f ?
		(res->abs_h - absf(res->net_h)) * RAD2DEG / metres : 0.0f;
	float steer_rate = secs > 0.1f ? (float)res->changes / secs : 0.0f;
	float sensor_rate = secs > 0.1f ? (float)res->sensor_changes / secs : 0.0f;
	bool hinted = false;
	char b1[16], b2[16], b3[16];

	tm_printf((UB*)TAG "==== RESULT: %s ====\n", status_text(res->status));
	tm_printf((UB*)TAG "  ran %u ms, %s mm, %u steering updates; net heading %s%s deg -> "
		  "%s run\n", (unsigned)res->run_ms, f1(b1, res->dist_mm), res->updates,
		  net_deg > 0.05f ? "L" : (net_deg < -0.05f ? "R" : ""), f1(b2, absf(net_deg)),
		  curve ? "CURVE" : "STRAIGHT");
	if (res->updates == 0U) return;
	if (LF_STRADDLE)
		tm_printf((UB*)TAG "  position: GAP (both white) %u  LEFT %u  RIGHT %u  BOTH BLACK "
			  "%u  (tape in the gap %s%% of the time)\n", res->pos_count[POS_LOST],
			  res->pos_count[POS_LEFT], res->pos_count[POS_RIGHT],
			  res->pos_count[POS_JUNCTION], f1(b1, centred));
	else
		tm_printf((UB*)TAG "  position: CENTRE %u  LEFT %u  RIGHT %u  LOST %u  (centred %s%% "
			  "of on-line time)\n", res->pos_count[POS_JUNCTION],
			  res->pos_count[POS_LEFT], res->pos_count[POS_RIGHT],
			  res->pos_count[POS_LOST], f1(b1, centred));
	tm_printf((UB*)TAG "  IR raw range: centre %u..%u  right %u..%u  (black > %u)\n",
		  (unsigned)res->c_min, (unsigned)res->c_max, (unsigned)res->r_min,
		  (unsigned)res->r_max, (unsigned)LINE_ADC_LINE_BLACK_THRESHOLD);

	/* --- wiggle --- */
	tm_printf((UB*)TAG "  WIGGLE: %u L<->R flips (%s per m, one full wiggle every %s mm); "
		  "heading zig-zag %s deg/m\n", res->flips,
		  f1(b1, metres > 0.05f ? (float)res->flips / metres : 0.0f),
		  res->flips > 1U ? f1(b2, 2.0f * res->dist_mm / (float)res->flips) : "-",
		  f1(b3, zigzag));
	tm_printf((UB*)TAG "          steering changes %s/s; IR position changes %s/s "
		  "(10 ms sampling)\n", f1(b1, steer_rate), f1(b2, sensor_rate));
	if (res->run_ms < MIN_JUDGE_MS)
		tm_printf((UB*)TAG "  (run under %u ms: too short to judge wiggle or turning)\n",
			  (unsigned)MIN_JUDGE_MS);
	else if (!curve && res->status != RUN_LINE_LOST) {
		float swing, hdev;

		straight_swing(res, &swing, &hdev);
		tm_printf((UB*)TAG "          centre swings %s mm peak-to-peak; heading strays up "
			  "to %s deg off the line\n", f1(b1, swing), f1(b2, hdev));
		if (LF_STRADDLE && (swing > WIGGLE_SWING_MM || hdev > WIGGLE_HDG_DEG)) {
			tm_printf((UB*)TAG "  HINT: WIGGLING on a straight (%u corrections, %u "
				  "overshoot pivots, %u searches). Pivots on a straight mean "
				  "LF_SL_OVERSHOOT_MS=%u is too short -- try %u; many searches mean "
				  "LF_SL_BLIND_MS=%u is too short. Otherwise lower LF_SL_TURN or widen "
				  "the sensor gap.\n", res->sl_corrections, res->sl_recoveries,
				  res->sl_searches, (unsigned)LF_SL_OVERSHOOT_MS,
				  (unsigned)LF_SL_OVERSHOOT_MS + 100U, (unsigned)LF_SL_BLIND_MS);
			hinted = true;
		} else if (swing > WIGGLE_SWING_MM || hdev > WIGGLE_HDG_DEG) {
			tm_printf((UB*)TAG "  HINT: WIGGLING on a straight. Corrections start at "
				  "+/-%u cm/s and build to %u over %u ms; the robot still overshoots "
				  "until the other sensor sees the line.\n", (unsigned)TMIN,
				  (unsigned)TURN, (unsigned)RAMP);
			tm_printf((UB*)TAG "        Try LF_RAMP_MS=%u (slower build-up)%s -- then "
				  "re-check the curve run.\n", (unsigned)RAMP + 200U,
				  TMIN > 1 ? " or LF_TURN_MIN one lower" : "");
			hinted = true;
		}
	}
	if (LF_STRADDLE) {
		unsigned both_black = res->pos_count[POS_JUNCTION];

		if (both_black * 5U > res->updates && res->updates > 30U) {
			tm_printf((UB*)TAG "  HINT: both sensors black %u of %u updates: the sensors "
				  "are CLOSER than the tape is wide. Move them apart so the tape fits "
				  "between them (display reads LOST when centred).\n", both_black,
				  res->updates);
			hinted = true;
		}
	} else if (centred < CENTRED_MIN_PCT && on_line > 30U) {
		tm_printf((UB*)TAG "  HINT: rarely CENTRE (%s%%). CENTRE = drive straight needs BOTH "
			  "sensors black. If the tape is narrower than the centre-right sensor gap, "
			  "the robot can only ever be LEFT or RIGHT, so it zig-zags forever. Place it "
			  "squarely on the line before GO: the display should say CENTRE.\n",
			  f1(b1, centred));
		hinted = true;
	}
	if (sensor_rate > 3.0f * steer_rate + 2.0f) {
		tm_printf((UB*)TAG "  HINT: IR position flickers (%s/s at 10 ms vs %s/s steering). "
			  "Raw values near the black threshold chatter at the tape edge; look at the "
			  "trace IRc/IRr columns. Threshold hysteresis would help.\n",
			  f1(b1, sensor_rate), f1(b2, steer_rate));
		hinted = true;
	}

	/* --- curves --- */
#if LF_STRADDLE
	{
		const float rec_inner = (float)(LF_SL_REC_SPEED - LF_SL_REC_TURN);
		const float rec_outer = (float)(LF_SL_REC_SPEED + LF_SL_REC_TURN);
		const float rec_radius = rec_inner <= 0.0f ? (float)LF_TRACK_MM * 0.5f :
			(float)LF_TRACK_MM * (float)LF_SL_REC_SPEED / (2.0f * (float)LF_SL_REC_TURN);

		tm_printf((UB*)TAG "  STRADDLE: gentle turn %u cm/s (radius ~%s mm); pivot wheels "
			  "%s/%s cm/s (radius ~%s mm)\n", (unsigned)LF_SL_TURN, f1(b1, min_radius),
			  f1(b2, rec_inner > 0.0f ? rec_inner : 0.0f), f1(b3, rec_outer),
			  f1(b1, rec_radius));
		tm_printf((UB*)TAG "          %u corrections (longest %u ms), %u overshoot pivots "
			  "(longest %u ms), %u blind searches (%u false: tape was in the gap)\n",
			  res->sl_corrections, (unsigned)res->sl_longest_correct_ms,
			  res->sl_recoveries, (unsigned)res->sl_longest_recover_ms,
			  res->sl_searches, res->sl_false_searches);
		tm_printf((UB*)TAG "          %u counter-steers (%u%% of each correction, max %u ms)"
			  "; wheels delivered %s%% of the commanded turn\n", res->sl_counters,
			  (unsigned)LF_SL_COUNTER_PCT, (unsigned)LF_SL_COUNTER_MAX_MS,
			  f1(b1, achieved));
		if (res->updates > 30U && res->pos_count[POS_LOST] * 2U < res->updates)
			tm_printf((UB*)TAG "  HINT: the tape was in the gap only %s%% of the time and "
				  "went LEFT<->RIGHT %u times: the gap between the sensors is too "
				  "small for the robot's yaw. Move the sensors further apart.\n",
				  f1(b2, centred), res->flips);
		if (curve) {
			float radius = absf(res->net_h) > 0.01f ? res->dist_mm / absf(res->net_h) : 0.0f;
			tm_printf((UB*)TAG "          drove an average radius of ~%s mm round the "
				  "curve\n", f1(b1, radius));
		}
		if (res->sl_searches > 0U && res->sl_false_searches * 2U > res->sl_searches) {
			tm_printf((UB*)TAG "  HINT: most searches were false alarms -- the tape was in "
				  "the gap. Raise LF_SL_BLIND_MS (now %u) if they make it wobble.\n",
				  (unsigned)LF_SL_BLIND_MS);
			hinted = true;
		}
		if (res->status == RUN_LINE_LOST) {
			tm_printf((UB*)TAG "  HINT: ended in %s: it pivoted %u ms without the tape "
				  "coming back. If it ran off a sharp bend first, lower LF_SL_BLIND_MS "
				  "(now %u) so it searches sooner%s.\n",
				  sl_state_name(res->sl_state_at_end), (unsigned)LF_SL_PIVOT_MAX_MS,
				  (unsigned)LF_SL_BLIND_MS,
				  LF_SL_BLIND_MS == 0U ? " (it is OFF)" : "");
			hinted = true;
		}
	}
#else
	tm_printf((UB*)TAG "  TURNING: full turn %u cm/s -> tightest radius ~%s mm at %u cm/s; "
		  "wheels delivered %s%% of the commanded turn\n", (unsigned)TURN,
		  f1(b1, min_radius), (unsigned)SPEED, f1(b2, achieved));
	tm_printf((UB*)TAG "          longest one-sided turn %u ms (%s); turning L %u / R %u "
		  "updates; line lost %u x, total %u ms, longest %u ms\n",
		  (unsigned)res->longest_side_ms, res->longest_side > 0 ? "LEFT" :
		  (res->longest_side < 0 ? "RIGHT" : "-"), res->turn_left, res->turn_right,
		  res->lost_events, (unsigned)res->lost_total_ms, (unsigned)res->lost_max_ms);
	if (curve) {
		unsigned inward = net_deg > 0.0f ? res->turn_left : res->turn_right;
		float duty = 100.0f * (float)inward / (float)res->updates;
		float radius = absf(res->net_h) > 0.01f ? res->dist_mm / absf(res->net_h) : 0.0f;

		tm_printf((UB*)TAG "          drove an average radius of ~%s mm; turning INTO "
			  "the curve %s%% of the time\n", f1(b1, radius), f1(b2, duty));
		if (duty >= INSIDE_DUTY_PCT) {
			tm_printf((UB*)TAG "  HINT: near the turning limit -- it is at full turn "
				  "%s%% of the time; little headroom for a tighter bend.\n",
				  f1(b1, duty));
			hinted = true;
		}
	}
	if (res->status == RUN_LINE_LOST && res->updates > 0U) {
		bool crossed_r = res->side_at_end > 0 &&
				 res->lost_right_peak > res->r_min + LOST_CROSS_RISE;
		bool crossed_c = res->side_at_end < 0 &&
				 res->lost_centre_peak > res->c_min + LOST_CROSS_RISE;

		if (crossed_r || crossed_c) {
			tm_printf((UB*)TAG "  NOTE: as the line was lost the %s sensor rose to %u "
				  "(white ~%u): the tape passed ACROSS toward it, so the robot turned "
				  "past the line -- the tape ended or straightened, or it over-turned. "
				  "NOT under-steer.\n", crossed_r ? "RIGHT" : "CENTRE",
				  (unsigned)(crossed_r ? res->lost_right_peak : res->lost_centre_peak),
				  (unsigned)(crossed_r ? res->r_min : res->c_min));
		} else if (res->longest_side_ms >= SATURATED_MS || res->side_at_end != 0) {
			tm_printf((UB*)TAG "  HINT: UNDER-STEER -- it was turning %s at full %u cm/s "
				  "(longest %u ms) and still ran off the line. The bend is tighter "
				  "than ~%s mm. Raise LF_TURN or lower LF_SPEED.\n",
				  res->side_at_end >= 0 ? "LEFT" : "RIGHT", (unsigned)TURN,
				  (unsigned)res->longest_side_ms, f1(b1, min_radius));
		} else {
			tm_printf((UB*)TAG "  HINT: lost the line while driving straight (CENTRE) "
				  "-- a gap, a very sharp corner, or both sensors crossed the tape "
				  "together. Check the trace just before the end.\n");
		}
		hinted = true;
	}
#endif /* LF_STRADDLE */
	/* The first ~1 s is the wheels spinning up from rest; too short to judge. */
	if (res->run_ms >= MIN_JUDGE_MS && res->achieved_n > 10U && achieved < ACHIEVED_MIN_PCT) {
		tm_printf((UB*)TAG "  HINT: the wheels deliver only %s%% of the turn asked for: "
			  "corrections flip before the wheel PID (50 ms) catches up, so the "
			  "robot turns less than LF_TURN suggests.\n", f1(b1, achieved));
		hinted = true;
	}
#if LF_CURVE_R_MM > 0 && !LF_STRADDLE
	{
		float need = (float)LF_TRACK_MM * (float)SPEED / (2.0f * (float)LF_CURVE_R_MM);
		float need_margin = need * 1.3f;   /* steering is only on part of the time */
		float s_max = (float)VEHICLE_MAX_SPEED_CM_S /
			      (1.0f + 1.3f * (float)LF_TRACK_MM / (2.0f * (float)LF_CURVE_R_MM));

		tm_printf((UB*)TAG "  CURVE %u mm needs turn >= %s cm/s at %u cm/s (~%s with "
			  "margin); you have %u\n", (unsigned)LF_CURVE_R_MM, f1(b1, need),
			  (unsigned)SPEED, f1(b2, need_margin), (unsigned)TURN);
		if ((float)TURN < need_margin) {
			unsigned t_need = (unsigned)need_margin + 1U;
			if (SPEED + t_need <= VEHICLE_MAX_SPEED_CM_S)
				tm_printf((UB*)TAG "  HINT: set LF_TURN=%u for this curve.\n", t_need);
			else
				tm_printf((UB*)TAG "  HINT: cannot fit at %u cm/s within the 20 cm/s "
					  "limit; lower LF_SPEED to <= %u.\n", (unsigned)SPEED,
					  (unsigned)s_max);
			hinted = true;
		}
	}
#endif
	if (!hinted)
		tm_printf((UB*)TAG "  looks fine on this run\n");
}

static void report_encoder_fault(void)
{
	encoder_fault_t f;

	if (encoder_source_fault(&f) < E_OK || f.reason == ENCODER_FAULT_NONE) return;
	tm_printf((UB*)TAG "  ENCODER FAULT: %s wheel, reason %u (1 both channels jumped, over "
		  "the limit of %u; 2 lost edge; 3 bad level), %u glitches and %u both-channel "
		  "jumps tolerated before it\n", f.wheel == 1U ? "LEFT" : "RIGHT",
		  (unsigned)f.reason, (unsigned)ENCODER_AMBIGUOUS_LIMIT, (unsigned)f.glitches,
		  (unsigned)f.ambiguous);
	tm_printf((UB*)TAG "  fault detail: edges 0x%02x, state AB %u%u -> %u%u, count %d\n",
		  (unsigned)f.events, (unsigned)((f.previous_state >> 1) & 1U),
		  (unsigned)(f.previous_state & 1U), (unsigned)((f.current_state >> 1) & 1U),
		  (unsigned)(f.current_state & 1U), (int)f.count);
}

/* Encoder noise the decoder absorbed so far (cumulative since boot). */
static void report_encoder_noise(void)
{
	encoder_fault_t f;

	if (encoder_source_fault(&f) < E_OK) return;
	tm_printf((UB*)TAG "  encoder noise since boot: %u one-channel glitches, %u both-channel "
		  "jumps (tolerated, max %u held per wheel)%s\n", (unsigned)f.glitches,
		  (unsigned)f.ambiguous, (unsigned)ENCODER_AMBIGUOUS_LIMIT,
		  f.ambiguous != 0U ? " -- check the encoder wiring/decoupling" : "");
}

void line_follow_hwtest_run(void)
{
	const wheel_speed_config_t config = {
		PID_KP, PID_KI, PID_KD, (float)SPEED, (uint8_t)PID_BASE, (uint8_t)PID_MAX,
		(float)(LF_FF), PID_FF_REF_CM_S
	};
	static line_adc_reader_t reader;
	line_adc_config_t ir_config;
	wheel_speed_control_t pid;
	unsigned run = 0U;
	char b1[16];
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
	if (!wheel_speed_control_init(&pid, &config)) fail_forever("PID config", E_PAR);
	s_go_last_pressed = go_pressed_raw();

	tm_printf((UB*)TAG "line following test: speed %u cm/s, turn %u -> %u cm/s over %u ms "
		  "(full turn wheels %u/%u), steer every %u ms, run %u mm, track %u mm\n",
		  (unsigned)SPEED, (unsigned)TMIN, (unsigned)TURN, (unsigned)RAMP,
		  (unsigned)(SPEED - TURN), (unsigned)(SPEED + TURN), (unsigned)LF_STEER_MS,
		  (unsigned)LF_DIST_MM, (unsigned)LF_TRACK_MM);
	if (LF_BARCODE)
		tm_printf((UB*)TAG "barcode decoding ON: decoder A exactly as the vehicle (10 ms, "
			  "hysteresis %u/%u, debounce 4, 50 ms odometry), B fast (2 ms), C as B measured "
			  "at the sensor; smallest narrow module %u counts (bigger adapts)\n",
			  (unsigned)LINE_ADC_BARCODE_BLACK_THRESHOLD, 600U, (unsigned)LF_BC_MODULE);
	if (LF_BC_SLOW > 0)
		tm_printf((UB*)TAG "barcode slow-down ON: %u cm/s from the first black on the barcode "
			  "sensor until %u mm of white\n", (unsigned)LF_BC_SLOW,
			  (unsigned)LF_BC_SLOW_MM);
	if (LF_STRADDLE)
		tm_printf((UB*)TAG "speed %u cm/s, turn %u, pivot %u/%u: %s the main robot's "
			  "(vehicle_controller_task.h: %u cm/s, turn %u, pivot %u/%u)\n",
			  (unsigned)SPEED, (unsigned)LF_SL_TURN, (unsigned)LF_SL_REC_SPEED,
			  (unsigned)LF_SL_REC_TURN,
			  SPEED == VEHICLE_LINE_SPEED_CM_S && LF_SL_TURN == VEHICLE_LINE_TURN_CM_S &&
			  LF_SL_REC_SPEED == VEHICLE_LINE_PIVOT_SPEED_CM_S &&
			  LF_SL_REC_TURN == VEHICLE_LINE_PIVOT_TURN_CM_S ? "SAME AS" : "DIFFERENT FROM",
			  (unsigned)VEHICLE_LINE_SPEED_CM_S, (unsigned)VEHICLE_LINE_TURN_CM_S,
			  (unsigned)VEHICLE_LINE_PIVOT_SPEED_CM_S, (unsigned)VEHICLE_LINE_PIVOT_TURN_CM_S);
	if (LF_STRADDLE)
		tm_printf((UB*)TAG "steering: STRADDLE (test-only, line_steering.c NOT used): "
			  "tape between the sensors = straight; one sensor = turn %u cm/s; held "
			  "> %u ms = pivot %d/%d cm/s; both white > %u ms = search (0 = off); "
			  "pivot > %u ms = lost; counter-steer %u%%; IR black >= %u / white <= %u"
			  "\n", (unsigned)LF_SL_TURN,
			  (unsigned)LF_SL_OVERSHOOT_MS,
			  LF_SL_REC_SPEED - LF_SL_REC_TURN > 0 ? LF_SL_REC_SPEED - LF_SL_REC_TURN : 0,
			  LF_SL_REC_SPEED + LF_SL_REC_TURN, (unsigned)LF_SL_BLIND_MS,
			  (unsigned)LF_SL_PIVOT_MAX_MS, (unsigned)LF_SL_COUNTER_PCT,
			  (unsigned)LF_SL_BLACK_ADC, (unsigned)LF_SL_WHITE_ADC);
	else
		tm_printf((UB*)TAG "steering profile: %s\n", LF_SLOPE ?
			  "SLOPE (ramp mode, forced for the whole run)" : "NORMAL");
	tm_printf((UB*)TAG "wheel feed-forward %s %%PWM per cm/s around %u%% at 15 cm/s%s\n",
		  f1(b1, (float)(LF_FF)), (unsigned)PID_BASE,
		  (float)(LF_FF) > 0.0f ? "" : " (OFF: flat base, as before)");
	tm_printf((UB*)TAG "tightest radius it can steer: ~%s mm (robot centre)\n",
		  f1(b1, (float)LF_TRACK_MM * (float)SPEED / (2.0f * (float)TURN)));
	tm_printf((UB*)TAG "suggested runs: 1) a STRAIGHT line (wiggle)  2) your tightest "
		  "CURVE (turning)  3) a ramp/barcode section\n");
	tm_printf((UB*)TAG "each GO press = one run; GO during a run = STOP\n");

	for (;;) {
		run_result_t res;
		run_status_t status;

		run++;
		tm_printf((UB*)TAG "---- run %u: place the robot ON the line, then press GO ----\n",
			  run);
		wait_for_go(&reader);
		tm_printf((UB*)TAG "moving in %u s, hands clear\n", START_DELAY_MS / 1000U);
		tk_dly_tsk(START_DELAY_MS);
		er = motor_clear_emergency_stop();
		if (er < E_OK) fail_forever("motor arm", er);

		status = run_once(&reader, &pid, &res);
		(void)motor_emergency_stop();
		report(&res);
#if LF_BARCODE
		bc_report();
#endif
		report_encoder_noise();
		if (status != RUN_OK && status != RUN_LINE_LOST && status != RUN_ABORT_GO &&
		    status != RUN_TIMEOUT) {
			tm_printf((UB*)TAG "MOVE ENDED: %s -- test stopped, reset to retry\n",
				  status_text(status));
			if (status == RUN_ENCODER) report_encoder_fault();
			for (;;) tk_dly_tsk(5000U);
		}
	}
}

#endif /* APP_SENSOR_TEST == 17 */
