/*
 * obs_width_hwtest.c - live obstacle width measurement (SENSOR_TEST=obstacle_width).
 *
 * Motor-free bench test. The robot stands still while the HC-SR04 watches
 * straight ahead. Once an obstacle is CONFIRMED (several consecutive, mutually
 * consistent readings inside the detection distance) the test sends the same
 * SCAN_REQUEST the vehicle controller would, waits for the servo sweep, and
 * prints:
 *   - the raw sweep (angle, range, lateral/forward position per sample)
 *   - full obstacle width
 *   - how far the obstacle extends LEFT and RIGHT of the robot centreline,
 *     and whether each edge was really seen or lies beyond the +/-20 deg sweep
 *   - free space beyond each edge and the profile's advisory bypass
 * It then waits for the obstacle to be removed before re-arming.
 */
#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "../../subsystem6/gpio/gpio_dispatcher.h"
#include "obs_pico_port.h"
#include "obs_profile.h"
#include "obs_task.h"
#include "obs_width_hwtest.h"

/* No <math.h>: its size_t clashes with the kernel's syslib.h once app.h is
 * included, so use the compiler builtins (they still link against libm). */

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 11

/* Distance at or below which an ahead reading counts as an obstacle. Must stay
 * below OBS_BLOCK_MM or the profile will not cluster the scan points. */
#ifndef OBS_WIDTH_TEST_DETECT_MM
#define OBS_WIDTH_TEST_DETECT_MM 400U
#endif
/* Optional known width of the test object; when non-zero the report prints
 * the error and an approximate OBS_BEAM_TRIM_DEG for calibration. */
#ifndef OBS_WIDTH_TEST_TRUE_WIDTH_MM
#define OBS_WIDTH_TEST_TRUE_WIDTH_MM 0U
#endif

_Static_assert(OBS_WIDTH_TEST_DETECT_MM >= OBS_MIN_RANGE_MM &&
	       OBS_WIDTH_TEST_DETECT_MM < OBS_BLOCK_MM,
	       "OBS_WIDTH_TEST_DETECT_MM must be within OBS_MIN_RANGE_MM..OBS_BLOCK_MM-1");

#define TAG "[HWTEST][OBS-WIDTH] "
/* Ahead readings arrive about every 100 ms. Time-based confirmation stops a
 * hand waved past the sensor (a few hundred ms) from triggering a scan. */
#define CONFIRM_MS           1000U /* obstacle must stay this long           */
#define CONFIRM_MIN_READINGS 5U    /* ...and give at least this many readings */
#define CONFIRM_TOLERANCE_MM 60U   /* each reading within this of the first   */
#define CLEAR_MS             500U  /* path must stay clear this long to re-arm */
#define CLEAR_HYSTERESIS_MM  50U   /* clear means beyond detect + this        */
#define STATUS_PRINT_MS      1000U /* live reading print interval while waiting */
/* Readings at or below this are not trusted. The port clamps echoes under
 * OBS_MIN_RANGE_MM (20 mm) up to 20 mm, and on the bench the sensor returned
 * a constant 24 mm (~140 us pulse) at every angle with nothing in front: a
 * no-echo artefact, not a distance. Anything this close would already be
 * touching the sensor, so such readings neither confirm nor clear. */
#ifndef OBS_WIDTH_TEST_MIN_VALID_MM
#define OBS_WIDTH_TEST_MIN_VALID_MM 40U
#endif
#define AT_MIN_RANGE(d)      ((d) != OBS_DIST_NONE && (d) <= OBS_WIDTH_TEST_MIN_VALID_MM)
#define EVENT_WAIT_MS        1000
#define ERROR_DELAY_MS       5000U
#define PI_F                 3.14159265f

typedef enum { ST_ARMED = 0, ST_SCANNING, ST_WAIT_CLEAR } hwtest_state_t;

/* report_scan() compares event and profile sides directly. */
_Static_assert((int)OBS_SIDE_CENTRE == (int)VEHICLE_OBSTACLE_SIDE_CENTRE &&
	       (int)OBS_SIDE_LEFT == (int)VEHICLE_OBSTACLE_SIDE_LEFT &&
	       (int)OBS_SIDE_RIGHT == (int)VEHICLE_OBSTACLE_SIDE_RIGHT,
	       "profile and message side enums must match");

static void fail_forever(const char *operation, ER error)
{
	for (;;) {
		tm_printf((UB*)TAG "%s failed: %d\n", operation, (int)error);
		tk_dly_tsk(ERROR_DELAY_MS);
	}
}

/* An obstacle-task fault stops the sensor permanently; report and halt. */
static void check_faults(void)
{
	fault_message_t fault;
	INT received = tk_rcv_mbf(g_mbf_faults, &fault, TMO_POL);

	if (received == (INT)sizeof(fault)) {
		for (;;) {
			tm_printf((UB*)TAG "obstacle task FAULT code %u "
				  "(19 sensor, 20 command, 21 delivery, 24 scan timeout); "
				  "reset the board\n", (unsigned)fault.code);
			tk_dly_tsk(ERROR_DELAY_MS);
		}
	}
}

/* The obstacle task keeps posting ahead readings (~every 60 ms) into a
 * 4-slot mailbox and faults when it is full. Discard them while the report
 * prints, so a slow USB console cannot trip a delivery fault. */
static void drain_events(void)
{
	obstacle_event_message_t event;

	while (tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) > 0) {
	}
}

static ER send_scan_request(uint32_t *sequence)
{
	obstacle_command_message_t command = {0};
	SYSTIM now;
	ER er = tk_get_otm(&now);

	if (er != E_OK) return er;
	command.header.type = VEHICLE_MESSAGE_OBSTACLE_COMMAND;
	command.header.version = VEHICLE_MESSAGE_VERSION;
	command.header.sequence = ++(*sequence);
	command.header.timestamp_ms = now.lo;
	command.command = VEHICLE_OBSTACLE_SCAN_REQUEST;
	return tk_snd_mbf(g_mbf_obstacle_commands, &command, (INT)sizeof(command), TMO_POL);
}

/* Integer abs for printing signed millimetres with an explicit L/R label. */
static unsigned mag(int v)
{
	return (unsigned)(v < 0 ? -v : v);
}

/* side/other are "LEFT"/"RIGHT". A negative edge means the obstacle never
 * reaches this side of the centreline; its near edge is on the other side. */
static void print_edge(const char *side, const char *other, int16_t edge_mm, bool seen)
{
	const char *note = seen ? "edge seen" : "edge BEYOND sweep: minimum estimate";

	if (edge_mm >= 0)
		tm_printf((UB*)TAG "  %-5s extent: %4u mm %s of centreline  [%s]\n",
			  side, mag(edge_mm), side, note);
	else
		tm_printf((UB*)TAG "  %-5s extent: none, %s edge is %u mm %s of centreline  [%s]\n",
			  side, side, mag(edge_mm), other, note);
}

static void print_raw_scan(const obs_task_sample_t *scan)
{
	uint8_t i;

	tm_printf((UB*)TAG "raw sweep (right -> left), x>0 = right of centre:\n");
	for (i = 0U; i < scan->count; i++) {
		int rel = (int)(scan->angle0_deg + (uint32_t)i * scan->step_deg) -
			  (int)OBS_ANGLE_AHEAD_DEG;
		uint16_t d = scan->distances[i];

		if (d == OBS_DIST_NONE) {
			tm_printf((UB*)TAG "  %c%2u deg :  NONE\n",
				  rel < 0 ? 'R' : (rel > 0 ? 'L' : 'C'), mag(rel));
		} else {
			/* Same geometry as obs_profile_compute(): angle - (ahead - 90). */
			float rad = (90.0f + (float)rel) * (PI_F / 180.0f);
			int x = (int)__builtin_lroundf((float)d * __builtin_cosf(rad));
			int y = (int)__builtin_lroundf((float)d * __builtin_sinf(rad));

			tm_printf((UB*)TAG "  %c%2u deg : %4u mm  x=%+5d y=%4d%s\n",
				  rel < 0 ? 'R' : (rel > 0 ? 'L' : 'C'), mag(rel),
				  (unsigned)d, x, y,
				  AT_MIN_RANGE(d) ? "  <TOO CLOSE TO TRUST: sensor artefact?" :
				  (d < OBS_BLOCK_MM ? "  <obstacle range" : ""));
		}
	}
}

static const char *side_name(uint8_t side)
{
	switch (side) {
	case OBS_SIDE_CENTRE: return "CENTRE";
	case OBS_SIDE_LEFT:   return "LEFT";
	case OBS_SIDE_RIGHT:  return "RIGHT";
	default:              return "UNKNOWN";
	}
}

static const char *action_name(uint8_t action)
{
	switch (action) {
	case OBS_ACT_LEFT:     return "go LEFT";
	case OBS_ACT_RIGHT:    return "go RIGHT";
	case OBS_ACT_REVERSE:  return "REVERSE (neither side fits)";
	case OBS_ACT_CONTINUE: return "continue";
	default:               return "stop";
	}
}

static void report_scan(unsigned scan_no, uint16_t confirmed_mm,
			const obstacle_event_message_t *event)
{
	obs_task_sample_t scan = {0};
	obs_profile_t profile;
	obs_profile_status_t status;
	uint32_t sequence;

	tm_printf((UB*)TAG "================ scan #%u ================\n", scan_no);
	tm_printf((UB*)TAG "confirmed obstacle ahead at %u mm\n", (unsigned)confirmed_mm);
	if (!obs_pico_port_last_scan(&scan, &sequence)) {
		tm_printf((UB*)TAG "no raw scan available\n");
		return;
	}
	print_raw_scan(&scan);
	drain_events();
	{
		unsigned i, at_min = 0U;

		for (i = 0U; i < scan.count; i++)
			if (AT_MIN_RANGE(scan.distances[i])) at_min++;
		if (at_min > 0U)
			tm_printf((UB*)TAG "WARNING: %u/%u points at or below %u mm; with "
				  "nothing that close this is a sensor/power fault\n",
				  at_min, (unsigned)scan.count,
				  (unsigned)OBS_WIDTH_TEST_MIN_VALID_MM);
		if (at_min * 2U > scan.count) {
			tm_printf((UB*)TAG "RESULT: UNRELIABLE, most points are at minimum "
				  "range; no width reported\n");
			return;
		}
	}
	status = obs_profile_compute(scan.distances, scan.count, scan.angle0_deg,
				     scan.step_deg, &profile);
	if (status == OBS_PROFILE_INVALID) {
		tm_printf((UB*)TAG "profile rejected the scan (invalid)\n");
		return;
	}
	if (status == OBS_PROFILE_CLEAR) {
		tm_printf((UB*)TAG "RESULT: nothing closer than %u mm in the sweep "
			  "(obstacle moved, or too narrow / angled for the sensor)\n",
			  (unsigned)OBS_BLOCK_MM);
		return;
	}

	tm_printf((UB*)TAG "RESULT:\n");
	tm_printf((UB*)TAG "  closest point : %4u mm\n", (unsigned)profile.closest_mm);
	if (profile.smeared) {
		/* Beam (>= +/-28 deg at 30 cm) wider than the +/-20 deg sweep: the
		 * numbers would be the sweep's own span, not the object's. */
		tm_printf((UB*)TAG "  FULL WIDTH    : NOT MEASURABLE (heard at every angle: "
			  "narrow object close up, or a wide surface)\n");
	} else {
		tm_printf((UB*)TAG "  FULL WIDTH    : %4u mm%s\n", (unsigned)profile.width_mm,
			  (profile.left_edge_seen && profile.right_edge_seen) ?
			  "" : "  (lower bound: one edge beyond the sweep)");
		print_edge("LEFT", "RIGHT", profile.left_edge_mm, profile.left_edge_seen);
		print_edge("RIGHT", "LEFT", profile.right_edge_mm, profile.right_edge_seen);
	}
	tm_printf((UB*)TAG "  object side   : %s  (%s, range trend %+d mm)\n",
		  side_name(profile.object_side),
		  (profile.left_edge_seen && profile.right_edge_seen) ?
		  "from edges" : "from range trend, weak cue",
		  (int)profile.range_trend_mm);
	tm_printf((UB*)TAG "  free space L  : %4u mm%s\n", (unsigned)profile.left_mm,
		  profile.left_mm == OBS_MAX_RANGE_MM ? " (nothing else seen)" : "");
	tm_printf((UB*)TAG "  free space R  : %4u mm%s\n", (unsigned)profile.right_mm,
		  profile.right_mm == OBS_MAX_RANGE_MM ? " (nothing else seen)" : "");
	tm_printf((UB*)TAG "  advisory      : %s, sideways %u mm, forward %u mm "
		  "(robot needs %u mm free)\n",
		  action_name(profile.action), (unsigned)profile.lateral_mm,
		  (unsigned)profile.advance_mm,
		  (unsigned)(OBS_ROBOT_WIDTH_MM + 2U * OBS_SAFETY_MARGIN_MM));
	tm_printf((UB*)TAG "  CSV,%u,%u,%u,%d,%d,%u,%u,%u,%u,%u,%s,%d\n", scan_no,
		  (unsigned)profile.closest_mm, (unsigned)profile.width_mm,
		  (int)profile.left_edge_mm, (int)profile.right_edge_mm,
		  (unsigned)profile.left_edge_seen, (unsigned)profile.right_edge_seen,
		  (unsigned)profile.left_mm, (unsigned)profile.right_mm,
		  (unsigned)profile.smeared, side_name(profile.object_side),
		  (int)profile.range_trend_mm);

	if (event->obstacle_class != VEHICLE_OBSTACLE_SCAN_FOUND ||
	    event->width_mm != profile.width_mm || event->left_mm != profile.left_mm ||
	    event->right_mm != profile.right_mm ||
	    event->object_side != profile.object_side)
		tm_printf((UB*)TAG "  WARNING: task event (width %u, L %u, R %u) differs "
			  "from raw-scan profile; scan copy may be stale\n",
			  (unsigned)event->width_mm, (unsigned)event->left_mm,
			  (unsigned)event->right_mm);

#if OBS_WIDTH_TEST_TRUE_WIDTH_MM > 0
	if (profile.left_edge_seen && profile.right_edge_seen) {
		int err = (int)profile.width_mm - (int)OBS_WIDTH_TEST_TRUE_WIDTH_MM;
		/* w = w0 - 2*d*tan(trim): solve for the trim that gives the true width. */
		float w0 = (float)profile.width_mm + 2.0f * (float)profile.closest_mm *
			   __builtin_tanf(OBS_BEAM_TRIM_DEG * (PI_F / 180.0f));
		float trim = __builtin_atanf((w0 - (float)OBS_WIDTH_TEST_TRUE_WIDTH_MM) /
				   (2.0f * (float)profile.closest_mm)) * (180.0f / PI_F);
		tm_printf((UB*)TAG "  calibration   : true %u mm, error %+d mm, "
			  "suggested OBS_BEAM_TRIM_DEG ~ %d.%u (approx.)\n",
			  (unsigned)OBS_WIDTH_TEST_TRUE_WIDTH_MM, err,
			  (int)trim, (unsigned)(__builtin_lroundf(__builtin_fabsf(trim) * 10.0f) % 10));
	} else {
		tm_printf((UB*)TAG "  calibration   : skipped, an edge is beyond the sweep; "
			  "move the object further away\n");
	}
#endif
}

void obs_width_hwtest_run(void)
{
	static const obs_task_config_t task_config = {
		50U,	/* period_ms */
		500U,	/* ahead_max_age_ms */
		2500U,	/* scan_max_age_ms: 9 points x 3 polls x 50 ms ~ 1.3 s */
		5000U,	/* scan_timeout_ms */
		500U,	/* command_max_age_ms */
		300U,	/* emergency_distance_mm */
		NULL, NULL, NULL, NULL
	};
	static obs_pico_port_config_t hardware;
	hwtest_state_t state = ST_ARMED;
	uint32_t command_sequence = 0U;
	uint32_t close_start_ms = 0U, clear_start_ms = 0U, last_status_ms = 0U;
	uint16_t first_close = 0U, confirmed_mm = 0U;
	unsigned close_count = 0U, scan_no = 0U, min_range_count = 0U;
	bool clearing = false;
	ER error;

	hardware = obs_pico_port_default_config();
	/* Same measured facts as the vehicle and ultrasonic test profiles. */
	hardware.echo_voltage_verified_3v3_safe = true;
	hardware.servo_limits_verified = true;

	error = obs_pico_port_register();
	if (error != E_OK) fail_forever("GPIO registration", error);
	error = gpio_dispatcher_start();
	if (error != E_OK) fail_forever("GPIO dispatcher", error);
	error = obs_pico_port_task_start(&hardware, &task_config);
	if (error != E_OK) fail_forever("obstacle task start", error);

	tm_printf((UB*)TAG "motor-free obstacle width test; keep the robot still\n");
	tm_printf((UB*)TAG "detect %u..%u mm, confirm after %u ms within %u mm, "
		  "sweep R%u..L%u deg in %u deg steps\n",
		  (unsigned)OBS_WIDTH_TEST_MIN_VALID_MM + 1U,
		  (unsigned)OBS_WIDTH_TEST_DETECT_MM, CONFIRM_MS,
		  CONFIRM_TOLERANCE_MM,
		  (unsigned)(OBS_ANGLE_AHEAD_DEG - hardware.scan_start_deg),
		  (unsigned)(hardware.scan_start_deg +
			     (hardware.scan_count - 1U) * hardware.scan_step_deg -
			     OBS_ANGLE_AHEAD_DEG),
		  (unsigned)hardware.scan_step_deg);
	tm_printf((UB*)TAG "CSV columns: scan,closest,width,left_edge,right_edge,"
		  "left_seen,right_seen,free_left,free_right,smeared,side,trend\n");
	tm_printf((UB*)TAG "ARMED: place an obstacle in front of the sensor\n");

	for (;;) {
		obstacle_event_message_t event;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &event, EVENT_WAIT_MS);

		check_faults();
		if (received == E_TMOUT) {
			if (state == ST_SCANNING)
				tm_printf((UB*)TAG "waiting for sweep to finish...\n");
			continue;
		}
		if (received != (INT)sizeof(event))
			fail_forever("event receive", received < E_OK ? (ER)received : E_IO);
		if (event.header.type != VEHICLE_MESSAGE_OBSTACLE_EVENT ||
		    event.header.version != VEHICLE_MESSAGE_VERSION)
			fail_forever("event validation", E_SYS);

		switch (state) {
		case ST_ARMED: {
			uint16_t d = event.distance_mm;
			uint32_t t = event.header.timestamp_ms;
			bool close;

			if (event.obstacle_class != VEHICLE_OBSTACLE_AHEAD_READING) break;
			if (AT_MIN_RANGE(d)) {
				/* Report the first and then every 10th, not every 100 ms. */
				if (min_range_count++ % 10U == 0U)
					tm_printf((UB*)TAG "reading %u mm ignored (<= %u mm is "
						  "untrusted: sensor artefact), %u so far\n",
						  (unsigned)d,
						  (unsigned)OBS_WIDTH_TEST_MIN_VALID_MM,
						  min_range_count);
			}
			close = d != OBS_DIST_NONE && !AT_MIN_RANGE(d) &&
				d <= OBS_WIDTH_TEST_DETECT_MM &&
				(close_count == 0U ||
				 mag((int)d - (int)first_close) <= CONFIRM_TOLERANCE_MM);
			if (!close) {
				if (close_count > 0U)
					tm_printf((UB*)TAG "candidate lost after %u ms "
						  "(passing object ignored), still armed\n",
						  (unsigned)(t - close_start_ms));
				close_count = 0U;
				/* A far-off reading may itself start a new candidate. */
				if (d == OBS_DIST_NONE || AT_MIN_RANGE(d) ||
				    d > OBS_WIDTH_TEST_DETECT_MM)
					break;
			}
			if (close_count == 0U) {
				first_close = d;
				close_start_ms = t;
				tm_printf((UB*)TAG "candidate at %u mm, holding for %u ms...\n",
					  (unsigned)d, CONFIRM_MS);
			}
			close_count++;
			if (close_count < CONFIRM_MIN_READINGS || t - close_start_ms < CONFIRM_MS)
				break;

			/* Confirmed; the event just received is a fresh ahead reading,
			 * which the obstacle task requires before it accepts a scan. */
			confirmed_mm = d;
			error = send_scan_request(&command_sequence);
			if (error != E_OK) fail_forever("scan request", error);
			tm_printf((UB*)TAG "obstacle CONFIRMED at %u mm (%u readings over %u ms) "
				  "-> sweeping servo\n", (unsigned)d, close_count,
				  (unsigned)(t - close_start_ms));
			close_count = 0U;
			state = ST_SCANNING;
			break;
		}
		case ST_SCANNING:
			if (event.obstacle_class != VEHICLE_OBSTACLE_SCAN_FOUND &&
			    event.obstacle_class != VEHICLE_OBSTACLE_SCAN_CLEAR)
				break;
			report_scan(++scan_no, confirmed_mm, &event);
			drain_events();
			tm_printf((UB*)TAG "remove the obstacle to re-arm (needs > %u mm "
				  "or NONE for %u ms)\n",
				  (unsigned)(OBS_WIDTH_TEST_DETECT_MM + CLEAR_HYSTERESIS_MM),
				  CLEAR_MS);
			clearing = false;
			last_status_ms = 0U;
			state = ST_WAIT_CLEAR;
			break;
		case ST_WAIT_CLEAR: {
			uint16_t d = event.distance_mm;
			uint32_t t = event.header.timestamp_ms;
			bool far;

			if (event.obstacle_class != VEHICLE_OBSTACLE_AHEAD_READING) break;
			far = d == OBS_DIST_NONE ||
			      d > OBS_WIDTH_TEST_DETECT_MM + CLEAR_HYSTERESIS_MM;
			if (!far) {
				clearing = false;
				if (last_status_ms == 0U || t - last_status_ms >= STATUS_PRINT_MS) {
					last_status_ms = t;
					tm_printf((UB*)TAG "still blocked: reading %u mm%s\n",
						  (unsigned)d, AT_MIN_RANGE(d) ?
						  " (untrusted: sensor artefact, nothing is that close)" : "");
				}
				break;
			}
			if (!clearing) {
				clearing = true;
				clear_start_ms = t;
			}
			if (t - clear_start_ms >= CLEAR_MS) {
				tm_printf((UB*)TAG "path clear -> ARMED for next obstacle\n");
				min_range_count = 0U;
				state = ST_ARMED;
			}
			break;
		}
		}
	}
}

#endif /* APP_SENSOR_TEST == 11 */
