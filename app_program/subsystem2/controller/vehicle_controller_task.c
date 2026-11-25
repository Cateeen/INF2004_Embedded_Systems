#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "vehicle_controller_task.h"
#include "straddle_steering.h"
#include "vehicle_bypass.h"
#include "../../subsystem1/wifi/vehicle_telemetry.h"
#include "../../subsystem1/wifi/dashboard_link.h"
#include "../../subsystem3/line_barcode/line_observation.h"
#include "../../subsystem4/imu/imu_types.h"

#define VEHICLE_CONTROLLER_PRIORITY 9
/* Each line-following VELOCITY command lasts this long, so the motors stop by
 * themselves within it if the controller stops sending (period is 50 ms). */
#define VEHICLE_CONTROLLER_DRIVE_MS 200U
/* imu_estimate_message_t.terrain_telemetry bits [2:0] = imu_fsm_state_t,
 * bits [11:6] = the hump's height so far in 0.25 cm steps (imu_types.h). */
#define IMU_TERRAIN_STATE_MASK      0x07U
#define IMU_TERRAIN_HEIGHT_SHIFT    6U
#define IMU_TERRAIN_HEIGHT_MASK     0x3FU
/* Live hump height to the dashboard at most this often (<= 5 Hz). */
#define VEHICLE_HUMP_REPORT_MS      200U

typedef struct {
	controller_state_t controller;
	uint32_t motion_sequence;
	uint32_t status_wait_ms;
	uint32_t status_sequence;
	uint32_t status_timestamp_ms;
	motion_status_message_t motion_status;
	uint8_t imu_fsm_state;		/* latest imu_fsm_state_t, for telemetry */
	uint8_t line_position;		/* latest POS_* from the line observations */
	straddle_t steering;            /* straddle line following */
	/* Motion status may report a moving robot only up to this time. */
	uint32_t motion_allowed_until_ms;
	bool motion_commanded;
	bool steering_stop_sent;
	bool arm_requested;
	bool emergency_sent;
	bool have_motion_status;
	/* Junction manoeuvre (VEHICLE_JUNCTION_* in the header). */
	bool junction_pending;          /* a junction observation arrived */
	bool junction_lead_seen;        /* spinning: the leading sensor is on a tape */
	bool junction_found;            /* spinning: the new line is in the gap */
	bool junction_skip;             /* the lead sensor's tape is the line being left */
	uint8_t junction_phase;         /* JUNCTION_* below */
	uint8_t junction_turn;          /* VEHICLE_JUNCTION_LEFT/RIGHT/UTURN */
	uint8_t junction_lines;         /* lines still to reach while spinning */
	int8_t junction_dir;            /* spinning: +1 left, -1 right */
	int16_t junction_speed;         /* forward speed for the approach */
	uint32_t junction_ms;           /* detection time, then phase start */
	uint32_t junction_forward_ms;   /* approach drive time */
	/* Obstacle bypass (config.obstacle_bypass). */
	uint8_t bypass_phase;           /* BYPASS_* below */
	uint32_t bypass_ms;             /* phase start */
	controller_state_value_t last_state;   /* previous cycle's state */
	/* Dashboard reports. */
	uint16_t ahead_mm;              /* latest ahead reading */
	uint16_t hump_peak_mm;          /* highest point of the current hump */
	uint32_t hump_report_ms;        /* last live height report */
	/* Pick-up (line observation picked_up). */
	bool picked_up;                 /* latest observation says lifted */
	bool pickup_reported;           /* reported and stopped for this lift */
	/* Lent motors are expected while a bypass runs, and in statuses stamped
	 * up to when its end was seen (they may still be queued). */
	uint32_t bypass_done_ms;
	bool have_bypass_done;
	bool previous_go;               /* inputs->go last cycle */
	bool obstacle_off_reported;     /* NO_ULTRASONIC fallback warning printed */
} vehicle_controller_context_t;

enum {
	BYPASS_IDLE = 0,
	BYPASS_STOPPING,        /* STOP sent: wait for the motion task to report idle */
	BYPASS_RUNNING          /* the vehicle_bypass task drives the motors */
};
/* An obstacle first seen this close or closer is not bypassed: it is inside
 * the 120 mm emergency distance, i.e. it appeared suddenly (e.g. a hand)
 * rather than being approached and stopped for at the 200 mm stop distance.
 * The robot just stops. */
#define VEHICLE_BYPASS_MIN_MM      125U
/* Wheels stopped this long, and idle reported, before the bypass starts. */
#define VEHICLE_BYPASS_SETTLE_MS   500U
#define VEHICLE_BYPASS_IDLE_MAX_MS 2000U


enum {
	JUNCTION_IDLE = 0,      /* line following */
	JUNCTION_FORWARD,       /* drive on until the wheels are over the crossing */
	JUNCTION_SETTLE_IN,     /* stopped, wheels settling before the spin */
	JUNCTION_SPIN,          /* spinning until the new line is reached */
	JUNCTION_SETTLE_OUT     /* stopped on the new line, settling */
};

/* Straddle line following (vehicle_controller_task.h). Losing the line on a
 * bend is handled by its own RECOVER pivot and blind SEARCH. */
static const straddle_config_t s_straddle_config = {
	VEHICLE_LINE_SPEED_CM_S, VEHICLE_LINE_TURN_CM_S, VEHICLE_LINE_OVERSHOOT_MS,
	VEHICLE_LINE_PIVOT_SPEED_CM_S, VEHICLE_LINE_PIVOT_TURN_CM_S,
	VEHICLE_LINE_BLIND_MS, VEHICLE_LINE_PIVOT_MAX_MS,
	VEHICLE_LINE_COUNTER_PCT, VEHICLE_LINE_COUNTER_MAX_MS
};
/* The motion task refuses VELOCITY with |turn| >= speed (a wheel would stop
 * or reverse) or a faster wheel over its limit. */
_Static_assert(VEHICLE_LINE_TURN_CM_S >= 1 && VEHICLE_LINE_TURN_CM_S < VEHICLE_LINE_SPEED_CM_S &&
	       VEHICLE_LINE_PIVOT_TURN_CM_S >= 1 &&
	       VEHICLE_LINE_PIVOT_TURN_CM_S < VEHICLE_LINE_PIVOT_SPEED_CM_S,
	       "straddle turns must be >= 1 and below their speed");

static vehicle_controller_task_config_t s_config;
static vehicle_controller_context_t s_context;
static bool s_started;

static bool publish_motion_message(vehicle_controller_context_t *context,
				   uint8_t mode, uint8_t stop_reason,
				   int16_t linear, int16_t angular,
				   uint16_t duration_ms, uint32_t now)
{
	motion_command_message_t message = {0};
	INT result;

	message.header.type = VEHICLE_MESSAGE_MOTION_COMMAND;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = context->motion_sequence + 1U;
	message.header.timestamp_ms = now;
	message.mode = mode;
	message.stop_reason = stop_reason;
	message.linear_velocity = linear;
	message.angular_velocity = angular;
	message.duration_ms = duration_ms;
	result = tk_snd_mbf(g_mbf_motion_commands, &message,
			    (INT)sizeof(message), TMO_POL);
	if (result != E_OK) {
		tm_printf((UB*)"vehicle controller command delivery failed: %d\n",
			  (int)result);
		return false;
	}
	context->motion_sequence = message.header.sequence;
	if (mode == VEHICLE_MOTION_VELOCITY || mode == VEHICLE_MOTION_TURN) {
		/* Status may show motion until this command expires, plus one
		 * status period for the stop to be reported. */
		context->motion_allowed_until_ms =
			now + duration_ms + VEHICLE_MOTION_STATUS_PERIOD_MS;
		context->motion_commanded = true;
	} else if (context->motion_commanded) {
		/* STOP / emergency stop: the motors must be idle within one
		 * status period from now. */
		uint32_t limit = now + VEHICLE_MOTION_STATUS_PERIOD_MS;
		if ((int32_t)(context->motion_allowed_until_ms - limit) > 0)
			context->motion_allowed_until_ms = limit;
	}
	return true;
}

static bool publish_motion_command(vehicle_controller_context_t *context,
				   uint8_t mode, uint8_t stop_reason,
				   uint32_t now)
{
	return publish_motion_message(context, mode, stop_reason, 0, 0, 0U, now);
}

/* Latest line observation as a steering position. A centred line (both
 * sensors see it: no lateral error) drives straight (POS_JUNCTION). The
 * observation's intersection_detected flag is a confirmed junction event,
 * handled by the junction manoeuvre; it does not change the position. */
static uint8_t line_position_of(const line_observation_message_t *message)
{
	if (message->line_detected == 0U) return POS_LOST;
	if (message->lateral_error < 0) return POS_LEFT;
	if (message->lateral_error > 0) return POS_RIGHT;
	return POS_JUNCTION;
}

/* True while a moving robot is expected: inside the window opened by the
 * last VELOCITY command and narrowed by any STOP since. */
static bool motion_expected(const vehicle_controller_context_t *context,
			    uint32_t timestamp_ms)
{
	return context->motion_commanded &&
	       (int32_t)(timestamp_ms - context->motion_allowed_until_ms) <= 0;
}

/*
 * Junction spin, one line observation at a time (they come every 10 ms; the
 * controller only every 50 ms, and a sensor can cross the tape in less).
 * The sensors straddle the line, so at the start of the spin the branch
 * straight ahead sits in the gap: it leaves through the TRAILING sensor and
 * is ignored. A line counts when it arrives at the LEADING sensor (the
 * centre one spinning left, the right one spinning right):
 *   gap seek (VEHICLE_JUNCTION_SEEK_GAP_*): the line is reached once it has
 *     left the leading sensor again, i.e. sits in the gap -- stopping as
 *     soon as the sensor touched it left the robot a few degrees short;
 *   otherwise: reached as soon as the leading sensor sees it.
 * The last line also counts if the trailing sensor goes black while the
 * leading one is on it (past the gap already). A tape already under the
 * leading sensor when the spin starts (junction_skip) is the line being
 * left: it is let through without counting.
 */
static void junction_seek(vehicle_controller_context_t *context, uint8_t position)
{
	const bool centre = position == POS_LEFT || position == POS_JUNCTION;
	const bool right = position == POS_RIGHT || position == POS_JUNCTION;
	const bool lead = context->junction_dir > 0 ? centre : right;
	const bool trail = context->junction_dir > 0 ? right : centre;
	const bool gap = context->junction_dir > 0 ? VEHICLE_JUNCTION_SEEK_GAP_LEFT :
						     VEHICLE_JUNCTION_SEEK_GAP_RIGHT;

	if (context->junction_found || context->junction_lines == 0U) return;
	if (!context->junction_lead_seen) {
		if (!lead) return;
		context->junction_lead_seen = true;
		if (!gap && context->junction_lines == 1U) context->junction_found = true;
		return;
	}
	if (!lead) {                    /* the tape has passed into the gap */
		context->junction_lead_seen = false;
		if (context->junction_skip) context->junction_skip = false;
		else if (--context->junction_lines == 0U) context->junction_found = true;
	} else if (trail && !context->junction_skip && context->junction_lines == 1U) {
		context->junction_found = true;
	}
}

static bool receive_line(vehicle_controller_context_t *context, uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_LINE_CAPACITY; i++) {
		line_observation_message_t message;
		INT received = tk_rcv_mbf(g_mbf_line_observations, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"vehicle controller line receive failed: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		if (!car_safety_line(&context->controller.safety, &message, now))
			return false;
		context->line_position = line_position_of(&message);
		context->picked_up = message.picked_up != 0U;
		if (context->junction_phase == JUNCTION_SPIN)
			junction_seek(context, context->line_position);
		if (message.intersection_detected != 0U) {
			context->junction_pending = true;
			context->junction_ms = message.header.timestamp_ms;
		}
	}
	return true;
}

/* Decoded barcodes: passed to the barcode hook (the dashboard maps the
 * letter to the next junction turn). Always drained, so the line task's
 * queue cannot fill up and fault it. */
static bool receive_barcodes(void)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_BARCODE_CAPACITY; i++) {
		barcode_event_message_t message;
		INT received = tk_rcv_mbf(g_mbf_barcode_events, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message) ||
		    message.header.type != VEHICLE_MESSAGE_BARCODE_EVENT ||
		    message.header.version != VEHICLE_MESSAGE_VERSION ||
		    message.length > sizeof(message.payload)) {
			tm_printf((UB*)"vehicle controller barcode receive failed: %d\n",
				  (int)received);
			return false;
		}
		if (message.valid != 0U && s_config.barcode != NULL)
			s_config.barcode(s_config.barcode_context, message.payload,
					 message.length);
	}
	return true;
}

static bool on_hump(uint8_t imu_fsm_state)
{
	return imu_fsm_state == (uint8_t)IMU_STATE_HUMP_RISING ||
	       imu_fsm_state == (uint8_t)IMU_STATE_HUMP_FALLING;
}

/* Dashboard: live hump height (rate-limited) while on a hump, and its peak
 * once when the robot leaves it. Called before imu_fsm_state is updated. */
static void report_hump(vehicle_controller_context_t *context,
			const imu_estimate_message_t *message, uint32_t now)
{
	uint8_t state = (uint8_t)(message->terrain_telemetry & IMU_TERRAIN_STATE_MASK);
	uint16_t height_mm = (uint16_t)((((message->terrain_telemetry >> IMU_TERRAIN_HEIGHT_SHIFT) &
					  IMU_TERRAIN_HEIGHT_MASK) * 25U) / 10U);

	if (on_hump(state)) {
		if (!on_hump(context->imu_fsm_state)) {
			context->hump_peak_mm = 0U;
			context->hump_report_ms = now - VEHICLE_HUMP_REPORT_MS;
		}
		if (height_mm > context->hump_peak_mm) context->hump_peak_mm = height_mm;
		if ((uint32_t)(now - context->hump_report_ms) >= VEHICLE_HUMP_REPORT_MS) {
			(void)dashboard_report_hump_height(height_mm);
			context->hump_report_ms = now;
		}
	} else if (on_hump(context->imu_fsm_state)) {
		(void)dashboard_report_hump_peak(context->hump_peak_mm);
	}
}

static bool receive_imu(vehicle_controller_context_t *context, uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_IMU_CAPACITY; i++) {
		imu_estimate_message_t message;
		INT received = tk_rcv_mbf(g_mbf_imu_estimates, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"vehicle controller IMU receive failed: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		report_hump(context, &message, now);
		context->imu_fsm_state =
			(uint8_t)(message.terrain_telemetry & IMU_TERRAIN_STATE_MASK);
		if (!car_safety_imu(&context->controller.safety, &message, now))
			return false;
	}
	return true;
}

/*
 * Line state for the dashboard with straddle line sensors. Centred on the
 * tape reads both white -- the same as no tape at all -- so "no tape under a
 * sensor" is NOT a lost line:
 *   following : FOUND while the steering tracks the tape (straight,
 *               correcting, pivoting at a bend or searching), LOST only once
 *               it has given up (stop latched);
 *   otherwise : FOUND if a sensor sees tape; LOST if the steering gave up
 *               and nothing has been seen since; else UNKNOWN (both white:
 *               the tape may be between the sensors, or not there);
 *   lifted    : UNKNOWN (no floor under the sensors).
 * Pivots, searches and the give-up are also sent as LINE events.
 */
static uint8_t straddle_line_state(const vehicle_controller_context_t *context,
				   controller_state_value_t state)
{
	if (context->picked_up) return WIFI_TELEMETRY_LINE_UNKNOWN;
	if (state == CONTROLLER_FOLLOWING && context->arm_requested &&
	    context->steering.initialized)
		return context->steering.stop_latched ? WIFI_TELEMETRY_LINE_LOST :
							WIFI_TELEMETRY_LINE_FOUND;
	if (context->line_position != POS_LOST) return WIFI_TELEMETRY_LINE_FOUND;
	return context->steering.stop_latched ? WIFI_TELEMETRY_LINE_LOST :
						WIFI_TELEMETRY_LINE_UNKNOWN;
}

/* Dashboard snapshot of what the controller currently believes. Read-only
 * with respect to control: it only reads context and never changes state. */
static void report_telemetry(const vehicle_controller_context_t *context,
			     controller_state_value_t state, uint32_t now)
{
	const car_safety_t *safety = &context->controller.safety;
	wifi_telemetry_data_t data = {0};

	switch (state) {
	case CONTROLLER_FOLLOWING:
	case CONTROLLER_SCANNING:
		data.vehicle_state = WIFI_TELEMETRY_VEHICLE_RUNNING;
		break;
	case CONTROLLER_BLOCKED:
		/* Driving round an obstacle is running, not paused. */
		data.vehicle_state = context->bypass_phase != BYPASS_IDLE ?
				     WIFI_TELEMETRY_VEHICLE_RUNNING : WIFI_TELEMETRY_VEHICLE_PAUSED;
		break;
	case CONTROLLER_FAULTED:
		data.vehicle_state = WIFI_TELEMETRY_VEHICLE_FAULT;
		break;
	case CONTROLLER_WAITING:
	default:
		data.vehicle_state = WIFI_TELEMETRY_VEHICLE_IDLE;
		break;
	}
	if (!safety->line.have_message) {
		data.line_state = WIFI_TELEMETRY_LINE_UNKNOWN;
	} else if (context->junction_phase != JUNCTION_IDLE) {
		/* Leaving the line to turn at a junction is not losing it. */
		data.line_state = WIFI_TELEMETRY_LINE_FOUND;
	} else if (safety->config.straddle_line) {
		data.line_state = straddle_line_state(context, state);
	} else {
		data.line_state = safety->line_clear ? WIFI_TELEMETRY_LINE_FOUND
						     : WIFI_TELEMETRY_LINE_LOST;
	}
	if (!safety->have_ahead || safety->obstacle_disabled) {
		/* No ultrasonic (NO_ULTRASONIC fallback): never claim "clear". */
		data.obstacle_state = WIFI_TELEMETRY_OBSTACLE_UNKNOWN;
	} else {
		data.obstacle_state = safety->ahead_clear ? WIFI_TELEMETRY_OBSTACLE_CLEAR
							  : WIFI_TELEMETRY_OBSTACLE_DETECTED;
	}
	if (!safety->imu.have_message) {
		data.terrain_state = WIFI_TELEMETRY_TERRAIN_UNKNOWN;
	} else if ((context->imu_fsm_state == (uint8_t)IMU_STATE_HUMP_RISING) ||
		   (context->imu_fsm_state == (uint8_t)IMU_STATE_HUMP_FALLING)) {
		data.terrain_state = WIFI_TELEMETRY_TERRAIN_HUMP;
	} else {
		data.terrain_state = WIFI_TELEMETRY_TERRAIN_FLAT;
	}
	if ((safety->faults & CAR_SAFETY_FAULT_IMU) != 0U) {
		data.fault_flags |= WIFI_TELEMETRY_FAULT_IMU;
	}
	if ((safety->faults & CAR_SAFETY_FAULT_OBSTACLE) != 0U) {
		data.fault_flags |= WIFI_TELEMETRY_FAULT_NO_ECHO;
	}
	if (context->have_motion_status && (context->motion_status.faulted != 0U)) {
		data.fault_flags |= WIFI_TELEMETRY_FAULT_MOTOR_STALL;
	}
	data.battery_pct = 0U;	/* no battery sensor: the dashboard shows none */
	vehicle_telemetry_report(&data, now);
}

static bool receive_obstacle(vehicle_controller_context_t *context, uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_OBSTACLE_CAPACITY; i++) {
		obstacle_event_message_t message;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"vehicle controller obstacle receive failed: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		if (!car_safety_obstacle(&context->controller.safety, &message, now))
			return false;
		/* The bypass task reads the ahead stream through the controller
		 * (it is this task's buffer). */
		if (message.obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING) {
			context->ahead_mm = message.distance_mm;
			if (s_config.obstacle_bypass)
				vehicle_bypass_feed(message.distance_mm,
						    message.header.timestamp_ms);
		}
	}
	return true;
}

static bool receive_faults(vehicle_controller_context_t *context)
{
	for (;;) {
		fault_message_t message;
		INT received = tk_rcv_mbf(g_mbf_faults, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message) ||
		    message.header.type != VEHICLE_MESSAGE_FAULT ||
		    message.header.version != VEHICLE_MESSAGE_VERSION ||
		    message.source < VEHICLE_FAULT_SENSOR ||
		    message.source > VEHICLE_FAULT_SYSTEM ||
		    message.recoverable > 1U || message.code == 0U) {
			tm_printf((UB*)"vehicle controller invalid fault message: %d\n",
				  (int)received);
		} else {
			/* NO_ULTRASONIC build: an obstacle task that failed before
			 * its first reading switches obstacle detection off. */
			if (message.code >= VEHICLE_FAULT_CODE_OBSTACLE_SENSOR &&
			    message.code <= VEHICLE_FAULT_CODE_OBSTACLE_SCAN_TIMEOUT &&
			    car_safety_disable_obstacle(&context->controller.safety)) {
				tm_printf((UB*)"vehicle controller: ultrasonic failed before its first "
					  "reading (code %u)\n", (unsigned)message.code);
				continue;
			}
			tm_printf((UB*)"vehicle controller subsystem fault: source=%u code=%u\n",
				  (unsigned)message.source, (unsigned)message.code);
		}
		car_safety_latch_fault(&context->controller.safety);
		return false;
	}
}

static bool lent_expected(const vehicle_controller_context_t *context, uint32_t timestamp)
{
	return context->bypass_phase == BYPASS_RUNNING ||
	       (context->have_bypass_done &&
		(int32_t)(timestamp - context->bypass_done_ms) <= 0);
}

static bool receive_motion_status(vehicle_controller_context_t *context,
				  uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_STATUS_CAPACITY; i++) {
		motion_status_message_t message;
		uint32_t age;
		uint32_t future;
		INT received = tk_rcv_mbf(g_mbf_motion_status, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message) ||
		    message.header.type != VEHICLE_MESSAGE_MOTION_STATUS ||
		    message.header.version != VEHICLE_MESSAGE_VERSION ||
		    message.active > 1U || message.faulted > 1U || message.lent > 1U ||
		    (message.faulted != 0U && message.fault_code == 0U) ||
		    (message.faulted == 0U && message.fault_code != 0U)) {
			tm_printf((UB*)"vehicle controller invalid motion status: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		age = now - message.header.timestamp_ms;
		future = message.header.timestamp_ms - now;
		if (age > s_config.motion_status_max_age_ms ||
		    (future != 0U && future <= INT32_MAX) ||
		    (context->have_motion_status &&
		     (message.header.sequence - context->status_sequence != 1U ||
		      message.header.timestamp_ms - context->status_timestamp_ms >
			      INT32_MAX))) {
			tm_printf((UB*)"vehicle controller stale or out-of-order motion status\n");
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		context->motion_status = message;
		context->status_sequence = message.header.sequence;
		context->status_timestamp_ms = message.header.timestamp_ms;
		context->have_motion_status = true;
		/* Moving is only acceptable while this controller commands it,
		 * or while a bypass run has the motors on loan. */
		if (message.lent != 0U && !lent_expected(context, message.header.timestamp_ms)) {
			tm_printf((UB*)"vehicle controller: motors lent outside an obstacle bypass\n");
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		if (message.faulted != 0U ||
		    (message.lent == 0U &&
		     (message.active != 0U || message.motor_1_pwm != 0 ||
		      message.motor_2_pwm != 0) &&
		     !motion_expected(context, message.header.timestamp_ms))) {
			tm_printf((UB*)"vehicle controller motion status is not safely idle\n");
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
	}
	return true;
}

static void latch_controller_fault(vehicle_controller_context_t *context,
				   uint32_t now)
{
	car_safety_latch_fault(&context->controller.safety);
	(void)controller_state_step(&context->controller, now, false, false);
}

/* The robot stopped by itself with GO still on: turn a toggle GO off. */
static void go_reset(void)
{
	if (s_config.go_reset != NULL) s_config.go_reset(s_config.input_context);
}

/* Why a GO press did not arm (the first failing check). */
static const char *arm_refusal(const car_safety_t *safety)
{
	if (safety->faults != 0U)
		return "a fault is latched -- restart the robot";
	if (safety->scanning) return "an obstacle scan is running";
	if (!safety->line.have_message || !safety->imu.have_message || !safety->have_ahead)
		return "still waiting for sensor data";
	if (!safety->imu_clear)
		return "IMU: tilted, or still calibrating -- put it down flat and keep it still ~1 s";
	if (!safety->ahead_clear) return "something ahead within the stop distance";
	if (!safety->line_clear)
		return safety->config.straddle_line ? "the robot is lifted (no floor under the sensors)" :
						      "the line is not under the sensors";
	return "a safety check";
}

/* Why a following robot stopped by itself (once per stop). */
static void print_stop_reason(const vehicle_controller_context_t *context,
			      controller_state_value_t state)
{
	const car_safety_t *safety = &context->controller.safety;

	if (state == CONTROLLER_FOLLOWING) {
		/* Still armed, but the steering latched its stop. */
		tm_printf((UB*)"stopped: %s\n",
			  context->picked_up ? "picked up" :
			  "line lost (the pivot / search for the tape found none)");
	} else if (state == CONTROLLER_FAULTED) {
		tm_printf((UB*)"stopped: fault -- %s\n",
			  safety->fault_reason != NULL ? safety->fault_reason : "unknown");
	} else if (safety->have_ahead && !safety->ahead_clear) {
		tm_printf((UB*)"stopped: something ahead at %u mm%s\n",
			  (unsigned)context->ahead_mm,
			  context->ahead_mm <= VEHICLE_BYPASS_MIN_MM ?
			  " (too close to bypass: appeared suddenly)" : "");
	} else {
		tm_printf((UB*)"stopped: %s\n", arm_refusal(safety));
	}
}

static void junction_reset(vehicle_controller_context_t *context)
{
	context->junction_phase = JUNCTION_IDLE;
	context->junction_pending = false;
}

static void junction_done(uint8_t turn)
{
	if (s_config.junction_done != NULL)
		s_config.junction_done(s_config.junction_context, turn);
}

/*
 * One cycle of the junction manoeuvre (see the header). Returns 0 when the
 * steering should run this cycle (no junction in progress, or it just
 * finished), 1 when a junction command was handled, -1 if a command could
 * not be sent.
 */
static int junction_step(vehicle_controller_context_t *context, uint32_t now)
{
	bool sent = true;

	if (context->junction_phase == JUNCTION_IDLE) {
		uint8_t turn;
		uint32_t mm;

		if (!context->junction_pending) return 0;
		context->junction_pending = false;
		if (context->steering.stop_latched || s_config.junction_turn == NULL)
			return 0;
		turn = s_config.junction_turn(s_config.junction_context);
		if (turn != VEHICLE_JUNCTION_LEFT && turn != VEHICLE_JUNCTION_RIGHT &&
		    turn != VEHICLE_JUNCTION_UTURN) {
			junction_done(VEHICLE_JUNCTION_STRAIGHT);
			return 0;       /* straight on: keep following */
		}
		context->junction_turn = turn;
		context->junction_speed = context->steering.config.speed_cm_s;
		mm = turn == VEHICLE_JUNCTION_RIGHT ? VEHICLE_JUNCTION_FORWARD_RIGHT_MM :
						      VEHICLE_JUNCTION_FORWARD_MM;
		/* mm at speed cm/s takes mm * 100 / speed ms, from the detection. */
		context->junction_forward_ms = mm * 100U / (uint32_t)context->junction_speed;
		context->junction_phase = JUNCTION_FORWARD;
		tm_printf((UB*)"junction: turning %s\n",
			  turn == VEHICLE_JUNCTION_LEFT ? "left" :
			  (turn == VEHICLE_JUNCTION_RIGHT ? "right" : "around"));
	}

	switch (context->junction_phase) {
	case JUNCTION_FORWARD:
		if ((uint32_t)(now - context->junction_ms) < context->junction_forward_ms) {
			sent = publish_motion_message(context, VEHICLE_MOTION_VELOCITY, 0U,
						      context->junction_speed, 0,
						      VEHICLE_CONTROLLER_DRIVE_MS, now);
			break;
		}
		sent = publish_motion_command(context, VEHICLE_MOTION_STOP, 0U, now);
		context->junction_phase = JUNCTION_SETTLE_IN;
		context->junction_ms = now;
		break;
	case JUNCTION_SETTLE_IN:
		if ((uint32_t)(now - context->junction_ms) < VEHICLE_JUNCTION_SETTLE_MS) break;
		context->junction_phase = JUNCTION_SPIN;
		context->junction_ms = now;
		context->junction_lines =
			context->junction_turn == VEHICLE_JUNCTION_UTURN ? 2U : 1U;
		context->junction_dir = context->junction_turn == VEHICLE_JUNCTION_RIGHT ? -1 : 1;
		/* Stopped and settled: a tape under the leading sensor now is
		 * the line being left, not the new one. */
		context->junction_skip =
			context->junction_dir > 0 ?
			(context->line_position == POS_LEFT || context->line_position == POS_JUNCTION) :
			(context->line_position == POS_RIGHT || context->line_position == POS_JUNCTION);
		context->junction_lead_seen = context->junction_skip;
		context->junction_found = false;
		/* fall through */
	case JUNCTION_SPIN:
		/* junction_seek() follows the lines, one observation at a time. */
		if (context->junction_found) {
			sent = publish_motion_command(context, VEHICLE_MOTION_STOP, 0U, now);
			context->junction_phase = JUNCTION_SETTLE_OUT;
			context->junction_ms = now;
		} else if ((uint32_t)(now - context->junction_ms) > VEHICLE_JUNCTION_SPIN_MAX_MS) {
			/* No line: stop and stay stopped, as for a lost line. */
			tm_printf((UB*)"junction: no line found while turning; stopped\n");
			(void)dashboard_report_line_event(DASHBOARD_LINE_LOST);
			sent = publish_motion_command(context, VEHICLE_MOTION_STOP, 0U, now);
			context->steering.stop_latched = true;
			context->steering_stop_sent = true;
			junction_reset(context);
		} else {
			sent = publish_motion_message(context, VEHICLE_MOTION_TURN, 0U, 0,
				(int16_t)(context->junction_dir * VEHICLE_JUNCTION_SPIN_CM_S),
				VEHICLE_CONTROLLER_DRIVE_MS, now);
		}
		break;
	case JUNCTION_SETTLE_OUT:
		if ((uint32_t)(now - context->junction_ms) < VEHICLE_JUNCTION_SETTLE_MS) break;
		/* On the new line: follow it from scratch. A junction seen while
		 * turning is not a new one. */
		straddle_reset(&context->steering);
		context->steering_stop_sent = false;
		junction_reset(context);
		junction_done(context->junction_turn);
		return 0;
	default:
		junction_reset(context);
		return 0;
	}
	return sent ? 1 : -1;
}

/* Stopped only because something is ahead: every other check is fine. */
static bool obstacle_blocked(const car_safety_t *safety)
{
	return safety->faults == 0U && !safety->scanning && safety->have_ahead &&
	       !safety->ahead_clear && safety->line.have_message &&
	       safety->imu.have_message && safety->imu_clear;
}

/*
 * One cycle of the obstacle bypass (config.obstacle_bypass). FOLLOWING ->
 * BLOCKED by an obstacle starts it: the normal BLOCKED path sends STOP, this
 * waits until the motion task reports idle, then hands the motors to the
 * vehicle_bypass task. Every safety check keeps running meanwhile; leaving
 * BLOCKED (a fault, or GO pressed to stop) aborts it. When it reports OK the
 * robot is re-armed with the same checks as a GO press (line seen, ahead
 * clear, ...) and the FOLLOWING path sends ARM and resumes the steering.
 */
static void bypass_step(vehicle_controller_context_t *context,
			controller_state_value_t state, uint32_t now,
			const vehicle_controller_inputs_t *inputs)
{
	car_safety_t *safety = &context->controller.safety;

	if (!s_config.obstacle_bypass) return;
	switch (context->bypass_phase) {
	case BYPASS_IDLE:
		if (state == CONTROLLER_BLOCKED && context->last_state == CONTROLLER_FOLLOWING &&
		    obstacle_blocked(safety) && context->junction_phase == JUNCTION_IDLE &&
		    context->ahead_mm > VEHICLE_BYPASS_MIN_MM) {
			context->bypass_phase = BYPASS_STOPPING;
			context->bypass_ms = now;
			tm_printf((UB*)"obstacle ahead: stopping to scan it\n");
		}
		break;
	case BYPASS_STOPPING:
		if (state != CONTROLLER_BLOCKED || context->picked_up) {
			context->bypass_phase = BYPASS_IDLE;
			tm_printf((UB*)"obstacle bypass cancelled before it started\n");
			break;
		}
		if ((uint32_t)(now - context->bypass_ms) < VEHICLE_BYPASS_SETTLE_MS) break;
		if (!context->have_motion_status || context->motion_status.active != 0U ||
		    context->motion_status.motor_1_pwm != 0 ||
		    context->motion_status.motor_2_pwm != 0) {
			if ((uint32_t)(now - context->bypass_ms) > VEHICLE_BYPASS_IDLE_MAX_MS) {
				context->bypass_phase = BYPASS_IDLE;
				tm_printf((UB*)"obstacle bypass: motion task not idle; stopped (press GO to retry)\n");
				go_reset();
			}
			break;
		}
		if (vehicle_bypass_request()) {
			context->bypass_phase = BYPASS_RUNNING;
		} else {
			context->bypass_phase = BYPASS_IDLE;
			tm_printf((UB*)"obstacle bypass unavailable; stopped (press GO to retry)\n");
			go_reset();
		}
		break;
	case BYPASS_RUNNING: {
		obstacle_bypass_status_t result = OBSTACLE_BYPASS_ABORTED;
		vehicle_bypass_state_t bypass = vehicle_bypass_state(&result);

		if (state != CONTROLLER_BLOCKED) vehicle_bypass_abort();
		if (bypass != VEHICLE_BYPASS_DONE) break;
		vehicle_bypass_ack();
		context->bypass_phase = BYPASS_IDLE;
		context->bypass_done_ms = now;
		context->have_bypass_done = true;
		/* The rejoin crosses the tape: not a junction. */
		junction_reset(context);
		if (result != OBSTACLE_BYPASS_OK || state != CONTROLLER_BLOCKED) {
			tm_printf((UB*)"obstacle bypass ended: %s; stopped (press GO to resume)\n",
				  obstacle_bypass_status_text(result));
			go_reset();
		} else if (car_safety_resume(safety, now, inputs->go, inputs->emergency_stopped)) {
			tm_printf((UB*)"obstacle bypassed: back on the line, following\n");
		} else {
			tm_printf((UB*)"obstacle bypassed, but cannot resume (%s); press GO to resume\n",
				  !safety->line_clear ? "line not under the sensors" :
				  (!safety->ahead_clear ? "something ahead" : "a safety check"));
			go_reset();
		}
		break;
	}
	default:
		context->bypass_phase = BYPASS_IDLE;
		break;
	}
}

/*
 * Lifted while running (following, turning at a junction or bypassing):
 * stop as for a lost line (latched: press GO to start again; it stops
 * again at once while still lifted), end any bypass, and tell the dashboard
 * once per lift (it ends the run timer).
 */
static void pickup_step(vehicle_controller_context_t *context,
			controller_state_value_t state)
{
	bool running = state == CONTROLLER_FOLLOWING ||
		       context->bypass_phase != BYPASS_IDLE;

	if (!context->picked_up) {
		context->pickup_reported = false;
		return;
	}
	if (!running) return;
	if (context->bypass_phase == BYPASS_RUNNING) vehicle_bypass_abort();
	if (state == CONTROLLER_FOLLOWING) {
		junction_reset(context);
		context->steering.stop_latched = true;
	}
	if (!context->pickup_reported) {
		context->pickup_reported = true;
		tm_printf((UB*)"picked up: motors stopped (press GO to start again)\n");
		/* End of the run: the last barcode's turn does not carry over. */
		dashboard_junction_reset();
		(void)dashboard_report_pickup();
	}
}

/*
 * Straddle steering edges to the dashboard (one event each, never per
 * cycle): a pivot at a bend, a blind search's outcome (the tape had escaped
 * and was found again, or it was between the sensors all along), and the
 * line given up. before = the steering before this cycle's update.
 */
static void report_line_events(const straddle_t *before, const straddle_t *after)
{
	if (after->recoveries != before->recoveries)
		(void)dashboard_report_line_event(DASHBOARD_LINE_PIVOT);
	if (before->state == STRADDLE_SEARCH && after->state != STRADDLE_SEARCH &&
	    !after->stop_latched)
		(void)dashboard_report_line_event(after->false_searches != before->false_searches ?
						  DASHBOARD_LINE_SEARCH_CENTRED :
						  DASHBOARD_LINE_SEARCH_FOUND);
	if (after->stop_latched && !before->stop_latched)
		(void)dashboard_report_line_event(DASHBOARD_LINE_LOST);
}

static bool controller_cycle(vehicle_controller_context_t *context,
			     uint32_t now,
			     const vehicle_controller_inputs_t *inputs)
{
	controller_state_value_t state;
	bool messages_valid = true;

	if (!receive_faults(context)) messages_valid = false;
	if (!receive_motion_status(context, now)) messages_valid = false;
	if (!receive_line(context, now)) messages_valid = false;
	if (!receive_barcodes()) messages_valid = false;
	if (!receive_imu(context, now)) messages_valid = false;
	if (!receive_obstacle(context, now)) messages_valid = false;
	if ((!context->have_motion_status &&
	     now - context->status_wait_ms > s_config.motion_status_max_age_ms) ||
	    (context->have_motion_status &&
	     now - context->status_timestamp_ms > s_config.motion_status_max_age_ms)) {
		tm_printf((UB*)"vehicle controller motion status deadline expired\n");
		messages_valid = false;
	}
	if (!messages_valid) car_safety_latch_fault(&context->controller.safety);
	if (context->controller.safety.obstacle_disabled && !context->obstacle_off_reported) {
		context->obstacle_off_reported = true;
		tm_printf((UB*)"WARNING: no ultrasonic sensor -- obstacle detection and bypass "
			  "are OFF (NO_ULTRASONIC build); the robot will NOT stop for obstacles\n");
	}

	state = controller_state_step(&context->controller, now,
				      inputs->go, inputs->emergency_stopped);
	if (state == CONTROLLER_BLOCKED && context->last_state == CONTROLLER_FOLLOWING &&
	    obstacle_blocked(&context->controller.safety))
		(void)dashboard_report_obstacle(context->ahead_mm);
	pickup_step(context, state);
	/* A GO press that did not arm: say why and turn a toggle GO back off,
	 * so the next press is another real attempt (not a press that only
	 * switches GO off). */
	if (inputs->go && !context->previous_go && state != CONTROLLER_FOLLOWING &&
	    state != CONTROLLER_FAULTED) {
		const car_safety_t *safety = &context->controller.safety;

		if (safety->faults == 0U && safety->have_ahead && !safety->ahead_clear)
			/* Say how close: 20 mm means the sensor returned an echo
			 * shorter than its minimum range (a glitch or a stuck
			 * sensor), not a real object at 2 cm. */
			tm_printf((UB*)"GO refused: something ahead at %u mm (stop distance %u mm)%s\n",
				  (unsigned)context->ahead_mm,
				  (unsigned)safety->config.obstacle_stop_mm,
				  context->ahead_mm <= 20U ?
				  " -- 20 mm is the sensor's floor: the ultrasonic sensor is stuck "
				  "(usually after RST); unplug USB AND switch the battery off for 5 s, "
				  "then power on" : "");
		else
			tm_printf((UB*)"GO refused: %s\n", arm_refusal(safety));
		go_reset();
	} else if (inputs->go && !context->previous_go && state == CONTROLLER_FAULTED) {
		/* Faults latch and the failed task has stopped: GO restarts. */
		if (s_config.restart != NULL) {
			tm_printf((UB*)"GO while faulted: restarting the robot\n");
			s_config.restart(s_config.input_context);
		} else {
			tm_printf((UB*)"GO refused: fault latched -- power-cycle the robot\n");
			go_reset();
		}
	}
	context->previous_go = inputs->go;
	bypass_step(context, state, now, inputs);
	/* Stopped by itself with GO still on (not handed to the bypass): a
	 * toggle GO turns off, so one press -- not two -- starts it again. */
	if (inputs->go && context->bypass_phase == BYPASS_IDLE &&
	    ((context->last_state == CONTROLLER_FOLLOWING && state != CONTROLLER_FOLLOWING) ||
	     (state == CONTROLLER_FOLLOWING && context->steering.stop_latched))) {
		print_stop_reason(context, state);
		go_reset();
	}
	context->last_state = state;
	report_telemetry(context, state, now);
	if (state == CONTROLLER_FAULTED) {
		if (!context->emergency_sent) {
			/* Say why: a missed sensor deadline latches silently. */
			uint32_t f = context->controller.safety.faults;

			tm_printf((UB*)"vehicle controller FAULTED (0x%02x):%s%s%s%s%s%s -- first cause: %s\n",
				  (unsigned)f,
				  (f & CAR_SAFETY_FAULT_LINE) ? " line" : "",
				  (f & CAR_SAFETY_FAULT_IMU) ? " imu" : "",
				  (f & CAR_SAFETY_FAULT_OBSTACLE) ? " obstacle" : "",
				  (f & CAR_SAFETY_FAULT_CLOCK) ? " clock" : "",
				  (f & CAR_SAFETY_FAULT_EXTERNAL) ? " subsystem/message" : "",
				  (f & CAR_SAFETY_FAULT_EMERGENCY) ? " emergency" : "",
				  context->controller.safety.fault_reason != NULL ?
				  context->controller.safety.fault_reason : "unknown");
		}
		if (!context->emergency_sent &&
		    publish_motion_command(context, VEHICLE_MOTION_EMERGENCY_STOP,
					   (uint8_t)context->controller.safety.faults,
					   now))
			context->emergency_sent = true;
		return false;
	}

	if (state == CONTROLLER_FOLLOWING) {
		line_steering_command_t steer;
		bool sent;
		int junction;

		if (!context->have_motion_status) return true;
		/* A bypass run still holds the motors (it was aborted and is
		 * stopping): ARM now would be refused by the driver. Wait for it. */
		if (context->bypass_phase != BYPASS_IDLE) return true;
		if (!context->arm_requested) {
			if (publish_motion_command(context, VEHICLE_MOTION_ARM, 0U, now)) {
				context->arm_requested = true;
				(void)straddle_init(&context->steering, &s_straddle_config);
				context->steering_stop_sent = false;
				junction_reset(context);
				/* The motion task ignores VELOCITY stamped in the
				 * same millisecond as ARM; drive from next cycle. */
				return true;
			}
			latch_controller_fault(context, now);
			if (publish_motion_command(context,
						   VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)context->controller.safety.faults,
						   now))
				context->emergency_sent = true;
			return false;
		}
		/* No blind search while the IMU has the robot on a ramp (rising
		 * covers the climb, top and descent; falling holds 300 ms of level
		 * ground after it): over the crest the tape is often out of sight
		 * and straight on is right. */
		straddle_set_hump(&context->steering, VEHICLE_LINE_HUMP_NO_SEARCH &&
				  on_hump(context->imu_fsm_state));
		junction = junction_step(context, now);
		if (junction > 0) return true;
		if (junction < 0) {
			sent = false;
		} else {
			const straddle_t before = context->steering;

			steer = straddle_update(&context->steering, context->line_position, now);
			report_line_events(&before, &context->steering);
			if (steer.stop_latched) {
				if (context->steering_stop_sent) return true;
				sent = publish_motion_command(context, VEHICLE_MOTION_STOP, 0U, now);
				context->steering_stop_sent = sent;
			} else {
				sent = publish_motion_message(context, VEHICLE_MOTION_VELOCITY, 0U,
							      steer.linear_cm_s, steer.turn_cm_s,
							      VEHICLE_CONTROLLER_DRIVE_MS, now);
			}
		}
		if (!sent) {
			latch_controller_fault(context, now);
			if (publish_motion_command(context,
						   VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)context->controller.safety.faults,
						   now))
				context->emergency_sent = true;
			return false;
		}
		return true;
	}

	junction_reset(context);        /* left FOLLOWING: abandon any turn */
	if (context->arm_requested) {
		if (publish_motion_command(context, VEHICLE_MOTION_STOP, 0U, now)) {
			context->arm_requested = false;
		} else {
			latch_controller_fault(context, now);
			if (publish_motion_command(context, VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)context->controller.safety.faults,
						   now))
				context->emergency_sent = true;
			return false;
		}
	}
	return true;
}

static void vehicle_controller_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	for (;;) {
		vehicle_controller_inputs_t inputs = {0};
		SYSTIM now;
		ER er = tk_get_otm(&now);

		if (er != E_OK) {
			tm_printf((UB*)"vehicle controller clock failed: %d\n", (int)er);
			latch_controller_fault(&s_context,
					       s_context.controller.safety.last_clock_ms);
			if (!s_context.emergency_sent &&
			    publish_motion_command(&s_context,
						   VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)s_context.controller.safety.faults,
						   s_context.controller.safety.last_clock_ms))
				s_context.emergency_sent = true;
		} else {
			bool inputs_valid = s_config.read_inputs(s_config.input_context, &inputs);
			if (!inputs_valid) {
				tm_printf((UB*)"vehicle controller local input read failed\n");
				latch_controller_fault(&s_context, now.lo);
			}
			(void)controller_cycle(&s_context, now.lo, &inputs);
		}

		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) {
			tm_printf((UB*)"vehicle controller schedule failed: %d\n", (int)er);
			latch_controller_fault(&s_context,
					       s_context.controller.safety.last_clock_ms);
			if (!s_context.emergency_sent &&
			    publish_motion_command(&s_context,
						   VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)s_context.controller.safety.faults,
						   s_context.controller.safety.last_clock_ms))
				s_context.emergency_sent = true;
			for (;;) {
				er = tk_slp_tsk(TMO_FEVR);
				if (er != E_OK)
					tm_printf((UB*)"vehicle controller sleep failed: %d\n",
						  (int)er);
			}
		}
	}
}

ER vehicle_controller_task_start(const vehicle_controller_task_config_t *config)
{
	controller_state_t validation = {0};
	controller_state_t initial = {0};
	T_CTSK task = {0};
	SYSTIM now;
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->read_inputs == NULL ||
	    config->period_ms == 0U ||
	    config->period_ms > VEHICLE_MOTION_STATUS_PERIOD_MS / 2U ||
	    config->motion_status_max_age_ms < 100U ||
	    config->motion_status_max_age_ms > INT32_MAX ||
	    config->period_ms > config->motion_status_max_age_ms ||
	    config->period_ms > config->safety.line_max_age_ms ||
	    config->period_ms > config->safety.imu_max_age_ms ||
	    config->period_ms > config->safety.ahead_max_age_ms ||
	    !controller_state_init(&validation, &config->safety, 0U))
		return E_PAR;
	if (g_mbf_line_observations <= 0 || g_mbf_imu_estimates <= 0 ||
	    g_mbf_obstacle_events <= 0 || g_mbf_faults <= 0 ||
	    g_mbf_motion_commands <= 0 || g_mbf_motion_status <= 0)
		return E_OBJ;
	er = tk_get_otm(&now);
	if (er != E_OK) return er < E_OK ? er : E_SYS;
	if (!controller_state_init(&initial, &config->safety, now.lo))
		return E_PAR;

	s_config = *config;
	s_context = (vehicle_controller_context_t){0};
	s_context.controller = initial;
	s_context.status_wait_ms = now.lo;
	task.task = (FP)vehicle_controller_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = VEHICLE_CONTROLLER_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
	/* Core 1 (TP_PRC2): core 0 runs the radio (priority 4) and USB console
	 * (3), which pre-empt any task there for milliseconds at a time. */
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC2;
#endif
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK)
			tm_printf((UB*)"vehicle controller cleanup failed: %d\n", (int)cleanup);
		else
			s_started = false;
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

#if APP_TEST
#include "../../test_report.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"

static BOOL drain_controller_test_buffer(ID id)
{
	union {
		line_observation_message_t line;
		imu_estimate_message_t imu;
		obstacle_event_message_t obstacle;
		fault_message_t fault;
		motion_status_message_t status;
		motion_command_message_t command;
	} message;
	INT received;

	do {
		received = tk_rcv_mbf(id, &message, TMO_POL);
	} while (received > 0);
	return received == E_TMOUT;
}

static BOOL drain_controller_test_buffers(void)
{
	BOOL ok = drain_controller_test_buffer(g_mbf_line_observations);
	ok = drain_controller_test_buffer(g_mbf_imu_estimates) && ok;
	ok = drain_controller_test_buffer(g_mbf_obstacle_events) && ok;
	ok = drain_controller_test_buffer(g_mbf_faults) && ok;
	ok = drain_controller_test_buffer(g_mbf_motion_status) && ok;
	ok = drain_controller_test_buffer(g_mbf_motion_commands) && ok;
	return ok;
}

static BOOL queue_motion_status(uint32_t now, uint32_t sequence,
				uint8_t active, int16_t motor_1, int16_t motor_2)
{
	motion_status_message_t status = {0};

	status.header.type = VEHICLE_MESSAGE_MOTION_STATUS;
	status.header.version = VEHICLE_MESSAGE_VERSION;
	status.header.sequence = sequence;
	status.header.timestamp_ms = now;
	status.active = active;
	status.motor_1_pwm = motor_1;
	status.motor_2_pwm = motor_2;
	return tk_snd_mbf(g_mbf_motion_status, &status, (INT)sizeof(status),
			  TMO_POL) == E_OK;
}

static BOOL queue_safe_sensor_messages(uint32_t now, uint32_t sequence)
{
	line_observation_message_t line = {0};
	imu_estimate_message_t imu = {0};
	obstacle_event_message_t obstacle = {0};
	BOOL ok;

	line.header = (vehicle_message_header_t){VEHICLE_MESSAGE_LINE_OBSERVATION,
						  VEHICLE_MESSAGE_VERSION, sequence, now};
	line.line_detected = 1U;
	line.confidence = 1000U;
	imu.header = (vehicle_message_header_t){VEHICLE_MESSAGE_IMU_ESTIMATE,
						 VEHICLE_MESSAGE_VERSION, sequence, now};
	imu.validity = VEHICLE_IMU_VALID_ROLL_PITCH;
	obstacle.header = (vehicle_message_header_t){VEHICLE_MESSAGE_OBSTACLE_EVENT,
						      VEHICLE_MESSAGE_VERSION, sequence, now};
	obstacle.obstacle_class = VEHICLE_OBSTACLE_AHEAD_READING;
	obstacle.distance_mm = OBS_DIST_NONE;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) == E_OK;
	ok = tk_snd_mbf(g_mbf_imu_estimates, &imu, (INT)sizeof(imu), TMO_POL) == E_OK && ok;
	ok = tk_snd_mbf(g_mbf_obstacle_events, &obstacle, (INT)sizeof(obstacle),
			TMO_POL) == E_OK && ok;
	return ok;
}

static BOOL receive_test_command(uint8_t expected_mode, uint32_t expected_sequence)
{
	motion_command_message_t command;
	INT received = tk_rcv_mbf(g_mbf_motion_commands, &command, TMO_POL);

	return received == (INT)sizeof(command) &&
	       command.header.type == VEHICLE_MESSAGE_MOTION_COMMAND &&
	       command.header.version == VEHICLE_MESSAGE_VERSION &&
	       command.header.sequence == expected_sequence &&
	       command.mode == expected_mode &&
	       command.linear_velocity == 0 && command.angular_velocity == 0 &&
	       command.duration_ms == 0U;
}

static const vehicle_controller_task_config_t test_config = {
	{200U, 200U, 200U, 200U, 500U, 300U},
	10U, 100U, NULL, NULL
};

static BOOL controller_arm_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	motion_command_message_t unexpected;
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = controller_cycle(&context, 100U, &inputs) &&
	     context.controller.state == CONTROLLER_WAITING &&
	     tk_rcv_mbf(g_mbf_motion_commands, &unexpected, TMO_POL) == E_TMOUT && ok;
	inputs.go = true;
	ok = controller_cycle(&context, 101U, &inputs) &&
	     context.controller.state == CONTROLLER_FOLLOWING &&
	     !context.arm_requested &&
	     tk_rcv_mbf(g_mbf_motion_commands, &unexpected, TMO_POL) == E_TMOUT && ok;
	ok = queue_motion_status(101U, 1U, 0U, 0, 0) && ok;
	ok = controller_cycle(&context, 102U, &inputs) &&
	     context.controller.state == CONTROLLER_FOLLOWING &&
	     context.arm_requested &&
	     receive_test_command(VEHICLE_MOTION_ARM, 1U) && ok;
	inputs.go = false;
	ok = controller_cycle(&context, 103U, &inputs) &&
	     context.controller.state == CONTROLLER_WAITING &&
	     !context.arm_requested &&
	     receive_test_command(VEHICLE_MOTION_STOP, 2U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL receive_test_velocity(int16_t linear, int16_t angular,
				  uint32_t expected_sequence)
{
	motion_command_message_t command;
	INT received = tk_rcv_mbf(g_mbf_motion_commands, &command, TMO_POL);

	return received == (INT)sizeof(command) &&
	       command.header.sequence == expected_sequence &&
	       command.mode == VEHICLE_MOTION_VELOCITY &&
	       command.linear_velocity == linear && command.angular_velocity == angular &&
	       command.duration_ms == VEHICLE_CONTROLLER_DRIVE_MS;
}

static BOOL queue_test_line(uint32_t now, uint32_t sequence, int16_t lateral_error)
{
	line_observation_message_t line = {0};

	line.header = (vehicle_message_header_t){VEHICLE_MESSAGE_LINE_OBSERVATION,
						  VEHICLE_MESSAGE_VERSION, sequence, now};
	line.line_detected = 1U;
	line.confidence = 1000U;
	line.lateral_error = lateral_error;
	return tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line),
			  TMO_POL) == E_OK;
}

/* Line following: after ARM the controller sends a steering VELOCITY every
 * cycle, accepts a moving status while it commands motion, stops on GO off
 * and faults if motion is still reported after the stop window. */
static BOOL controller_drive_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	motion_command_message_t unexpected;
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = controller_cycle(&context, 100U, &inputs) && ok;
	inputs.go = true;
	ok = controller_cycle(&context, 101U, &inputs) && ok;
	ok = queue_motion_status(101U, 1U, 0U, 0, 0) && ok;
	ok = controller_cycle(&context, 102U, &inputs) &&
	     receive_test_command(VEHICLE_MOTION_ARM, 1U) &&
	     tk_rcv_mbf(g_mbf_motion_commands, &unexpected, TMO_POL) == E_TMOUT && ok;
	/* Tape under the left (centre) sensor: gentle turn left. */
	ok = queue_test_line(110U, 2U, -1) && ok;
	ok = controller_cycle(&context, 110U, &inputs) &&
	     receive_test_velocity(VEHICLE_LINE_SPEED_CM_S, VEHICLE_LINE_TURN_CM_S, 2U) && ok;
	ok = queue_motion_status(120U, 2U, 1U, 22, -26) && ok;
	ok = controller_cycle(&context, 120U, &inputs) &&
	     context.controller.state == CONTROLLER_FOLLOWING &&
	     receive_test_velocity(VEHICLE_LINE_SPEED_CM_S, VEHICLE_LINE_TURN_CM_S, 3U) && ok;
	/* Both black (a crossing tape): straight. */
	ok = queue_test_line(130U, 3U, 0) && ok;
	ok = controller_cycle(&context, 130U, &inputs) &&
	     receive_test_velocity(VEHICLE_LINE_SPEED_CM_S, 0, 4U) && ok;
	inputs.go = false;
	ok = controller_cycle(&context, 140U, &inputs) &&
	     context.controller.state == CONTROLLER_WAITING &&
	     receive_test_command(VEHICLE_MOTION_STOP, 5U) && ok;
	/* Still reported moving 60 ms after STOP: inside the window. */
	ok = queue_motion_status(200U, 3U, 1U, 22, -22) && ok;
	ok = controller_cycle(&context, 200U, &inputs) &&
	     context.controller.state == CONTROLLER_WAITING && ok;
	/* 110 ms after STOP the motors must be idle. */
	ok = queue_motion_status(250U, 4U, 1U, 22, -22) && ok;
	ok = !controller_cycle(&context, 250U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED && context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 6U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_fault_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_motion_status(301U, 1U, 0U, 0, 0) && ok;
	ok = !controller_cycle(&context, 301U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_local_emergency_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {true, true};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = queue_motion_status(100U, 1U, 0U, 0, 0) && ok;
	ok = !controller_cycle(&context, 100U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_nonidle_status_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = queue_motion_status(100U, 1U, 0U, 20, -20) && ok;
	ok = !controller_cycle(&context, 100U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_stale_status_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = queue_motion_status(100U, 1U, 0U, 0, 0) && ok;
	ok = controller_cycle(&context, 100U, &inputs) && ok;
	ok = queue_safe_sensor_messages(201U, 2U) && ok;
	ok = !controller_cycle(&context, 201U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

/* ---- Junction manoeuvre ---------------------------------------------------- */

#define TEST_NO_COMMAND 0xFFU

static uint8_t s_test_turn;
static uint8_t s_test_done_turn;
static unsigned s_test_done_count;

static uint8_t test_junction_turn(void *context)
{
	(void)context;
	return s_test_turn;
}

static void test_junction_done(void *context, uint8_t turn)
{
	(void)context;
	s_test_done_turn = turn;
	s_test_done_count++;
}

typedef struct {
	vehicle_controller_context_t context;
	vehicle_controller_inputs_t inputs;
	uint32_t line_seq, imu_seq, obstacle_seq, status_seq;
	uint32_t now;
	uint8_t terrain;                    /* imu_fsm_state_t for the IMU messages */
	motion_command_message_t command;   /* mode TEST_NO_COMMAND if none sent */
} junction_test_t;

/* Advance 10 ms and run one cycle with fresh IMU/obstacle data, an idle
 * motion status and the given line observation; keep the command sent. */
static BOOL junction_cycle(junction_test_t *t, uint8_t detected, int16_t lateral,
			   uint8_t intersection)
{
	line_observation_message_t line = {0};
	imu_estimate_message_t imu = {0};
	obstacle_event_message_t obstacle = {0};
	INT received;
	BOOL ok;

	t->now += 10U;
	line.header = (vehicle_message_header_t){VEHICLE_MESSAGE_LINE_OBSERVATION,
						  VEHICLE_MESSAGE_VERSION, ++t->line_seq, t->now};
	line.line_detected = detected;
	line.confidence = detected ? 1000U : 0U;
	line.lateral_error = lateral;
	line.intersection_detected = intersection;
	imu.header = (vehicle_message_header_t){VEHICLE_MESSAGE_IMU_ESTIMATE,
						 VEHICLE_MESSAGE_VERSION, ++t->imu_seq, t->now};
	imu.validity = VEHICLE_IMU_VALID_ROLL_PITCH;
	imu.terrain_telemetry = t->terrain;
	obstacle.header = (vehicle_message_header_t){VEHICLE_MESSAGE_OBSTACLE_EVENT,
						      VEHICLE_MESSAGE_VERSION, ++t->obstacle_seq,
						      t->now};
	obstacle.obstacle_class = VEHICLE_OBSTACLE_AHEAD_READING;
	obstacle.distance_mm = OBS_DIST_NONE;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) == E_OK;
	ok = tk_snd_mbf(g_mbf_imu_estimates, &imu, (INT)sizeof(imu), TMO_POL) == E_OK && ok;
	ok = tk_snd_mbf(g_mbf_obstacle_events, &obstacle, (INT)sizeof(obstacle),
			TMO_POL) == E_OK && ok;
	ok = queue_motion_status(t->now, ++t->status_seq, 0U, 0, 0) && ok;
	ok = controller_cycle(&t->context, t->now, &t->inputs) && ok;
	t->command = (motion_command_message_t){0};
	t->command.mode = TEST_NO_COMMAND;
	received = tk_rcv_mbf(g_mbf_motion_commands, &t->command, TMO_POL);
	if (received == E_TMOUT) t->command.mode = TEST_NO_COMMAND;
	else if (received != (INT)sizeof(t->command)) ok = FALSE;
	return ok;
}

static BOOL is_motion(const junction_test_t *t, uint8_t mode, int16_t linear,
		      int16_t angular)
{
	return t->command.mode == mode && t->command.linear_velocity == linear &&
	       t->command.angular_velocity == angular &&
	       t->command.duration_ms == VEHICLE_CONTROLLER_DRIVE_MS;
}

/* Arm and follow until the first steering VELOCITY. */
static BOOL junction_arm(junction_test_t *t)
{
	BOOL ok;

	*t = (junction_test_t){0};
	t->now = 90U;
	ok = controller_state_init(&t->context.controller, &test_config.safety, 100U);
	t->context.status_wait_ms = 100U;
	ok = junction_cycle(t, 1U, -1, 0U) && ok;               /* 100: WAITING */
	t->inputs.go = true;
	ok = junction_cycle(t, 1U, -1, 0U) && t->command.mode == VEHICLE_MOTION_ARM && ok;
	ok = junction_cycle(t, 1U, -1, 0U) &&
	     is_motion(t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_SPEED_CM_S,
		       VEHICLE_LINE_TURN_CM_S) && ok;
	return ok;
}

/* Line observations as positions: tape in the gap, under the centre (left)
 * sensor, under the right one. */
#define SEEN_GAP     0U, 0
#define SEEN_CENTRE  1U, -1
#define SEEN_RIGHT   1U, 1

/* n cycles with the same observation, each still spinning. */
static BOOL spin_cycles(junction_test_t *t, uint8_t detected, int16_t lateral, unsigned n,
			int16_t spin)
{
	BOOL ok = TRUE;

	for (unsigned i = 0U; i < n; i++)
		ok = junction_cycle(t, detected, lateral, 0U) &&
		     is_motion(t, VEHICLE_MOTION_TURN, 0, spin) && ok;
	return ok;
}

/*
 * A junction with the given turn: drive on for the forward distance, stop,
 * settle, spin (+ left) until the new line is reached (a U-turn: the
 * second), stop, settle, then follow again. Spinning, the branch straight
 * ahead leaves through the trailing sensor first (ignored); a line counts at
 * the leading sensor -- as soon as the sensor sees it, or (gap seek,
 * VEHICLE_JUNCTION_SEEK_GAP_*) once it has passed into the gap. start_on_lead: a tape is already
 * under the leading sensor when the spin starts (the branch being left):
 * it passes without counting.
 */
static BOOL junction_turn_test(uint8_t turn, bool start_on_lead)
{
	junction_test_t t;
	const int16_t speed = VEHICLE_LINE_SPEED_CM_S;
	const int16_t spin = turn == VEHICLE_JUNCTION_RIGHT ?
			     (int16_t)-VEHICLE_JUNCTION_SPIN_CM_S :
			     (int16_t)VEHICLE_JUNCTION_SPIN_CM_S;
	const uint32_t forward_ms = (turn == VEHICLE_JUNCTION_RIGHT ?
				     VEHICLE_JUNCTION_FORWARD_RIGHT_MM :
				     VEHICLE_JUNCTION_FORWARD_MM) * 100U / (uint32_t)speed;
	uint32_t detected_at, stopped_at;
	BOOL ok;

	s_test_turn = turn;
	s_test_done_count = 0U;
	s_test_done_turn = TEST_NO_COMMAND;
	ok = junction_arm(&t);

	/* Junction observation: the approach starts at once. */
	ok = junction_cycle(&t, 1U, -1, 1U) &&
	     is_motion(&t, VEHICLE_MOTION_VELOCITY, speed, 0) && ok;
	detected_at = t.now;
	while (t.now + 10U - detected_at < forward_ms && ok)
		ok = junction_cycle(&t, 1U, -1, 0U) &&
		     is_motion(&t, VEHICLE_MOTION_VELOCITY, speed, 0);
	ok = junction_cycle(&t, 1U, -1, 0U) && t.command.mode == VEHICLE_MOTION_STOP &&
	     t.now - detected_at >= forward_ms && ok;
	stopped_at = t.now;
	while (t.now + 10U - stopped_at < VEHICLE_JUNCTION_SETTLE_MS && ok)
		ok = junction_cycle(&t, 1U, -1, 0U) && t.command.mode == TEST_NO_COMMAND;

	/* Spin: the branch ahead starts in the gap and leaves through the
	 * trailing sensor (ignored) -- or starts under the leading sensor. */
	if (!start_on_lead) {
		ok = junction_cycle(&t, SEEN_GAP, 0U) &&
		     is_motion(&t, VEHICLE_MOTION_TURN, 0, spin) && ok;
	} else if (turn == VEHICLE_JUNCTION_RIGHT) {
		ok = junction_cycle(&t, SEEN_RIGHT, 0U) &&
		     is_motion(&t, VEHICLE_MOTION_TURN, 0, spin) && ok;
		ok = spin_cycles(&t, SEEN_RIGHT, 2U, spin) && ok;
		ok = spin_cycles(&t, SEEN_GAP, 2U, spin) && ok;
	} else {
		ok = junction_cycle(&t, SEEN_CENTRE, 0U) &&
		     is_motion(&t, VEHICLE_MOTION_TURN, 0, spin) && ok;
		ok = spin_cycles(&t, SEEN_CENTRE, 2U, spin) && ok;
		ok = spin_cycles(&t, SEEN_GAP, 2U, spin) && ok;
	}
	if (turn == VEHICLE_JUNCTION_RIGHT) {
		ok = spin_cycles(&t, SEEN_CENTRE, 3U, spin) && ok;      /* trailing */
		ok = spin_cycles(&t, SEEN_GAP, 5U, spin) && ok;
		/* The new line at the leading (right) sensor: stop at once. */
		ok = junction_cycle(&t, SEEN_RIGHT, 0U) && t.command.mode == VEHICLE_MOTION_STOP &&
		     s_test_done_count == 0U && ok;
	} else {
		ok = spin_cycles(&t, SEEN_RIGHT, 3U, spin) && ok;       /* trailing */
		ok = spin_cycles(&t, SEEN_GAP, 5U, spin) && ok;
		if (turn == VEHICLE_JUNCTION_UTURN) {
			/* The side arm: in at the leading sensor, through the
			 * gap, out at the trailing one; the spin goes on. */
			ok = spin_cycles(&t, SEEN_CENTRE, 3U, spin) && ok;
			ok = spin_cycles(&t, SEEN_GAP, 3U, spin) && ok;
			ok = spin_cycles(&t, SEEN_RIGHT, 3U, spin) && ok;
			ok = spin_cycles(&t, SEEN_GAP, 5U, spin) && ok;
		}
#if VEHICLE_JUNCTION_SEEK_GAP_LEFT
		/* The new line at the leading (centre) sensor: not yet ... */
		ok = spin_cycles(&t, SEEN_CENTRE, 3U, spin) && ok;
		/* ... it stops once the line has passed into the gap. */
		ok = junction_cycle(&t, SEEN_GAP, 0U) && t.command.mode == VEHICLE_MOTION_STOP &&
		     s_test_done_count == 0U && ok;
#else
		/* The new line at the leading (centre) sensor: stop at once. */
		ok = junction_cycle(&t, SEEN_CENTRE, 0U) && t.command.mode == VEHICLE_MOTION_STOP &&
		     s_test_done_count == 0U && ok;
#endif
	}
	stopped_at = t.now;
	while (t.now + 10U - stopped_at < VEHICLE_JUNCTION_SETTLE_MS && ok)
		ok = junction_cycle(&t, 1U, -1, 0U) && t.command.mode == TEST_NO_COMMAND;

	/* Following again from scratch, and the turn is reported done. */
	ok = junction_cycle(&t, 1U, -1, 0U) &&
	     is_motion(&t, VEHICLE_MOTION_VELOCITY, speed, VEHICLE_LINE_TURN_CM_S) &&
	     s_test_done_count == 1U && s_test_done_turn == turn && ok;
	return drain_controller_test_buffers() && ok;
}

/* STRAIGHT keeps following (and is reported); no line within the spin limit
 * stops the robot and keeps it stopped; no callback ignores junctions. */
static BOOL junction_other_test(void)
{
	junction_test_t t;
	uint32_t spin_from;
	BOOL ok;

	s_test_turn = VEHICLE_JUNCTION_STRAIGHT;
	s_test_done_count = 0U;
	ok = junction_arm(&t);
	ok = junction_cycle(&t, 1U, -1, 1U) &&
	     is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_SPEED_CM_S,
		       VEHICLE_LINE_TURN_CM_S) &&
	     s_test_done_count == 1U && s_test_done_turn == VEHICLE_JUNCTION_STRAIGHT && ok;
	ok = drain_controller_test_buffers() && ok;

	/* Left turn, but the sensors never find a line once off it. */
	s_test_turn = VEHICLE_JUNCTION_LEFT;
	s_test_done_count = 0U;
	ok = junction_arm(&t) && ok;
	ok = junction_cycle(&t, 1U, -1, 1U) && ok;
	while (t.command.mode != VEHICLE_MOTION_TURN && t.now < 5000U && ok)
		ok = junction_cycle(&t, 1U, -1, 0U);
	spin_from = t.now;
	while (t.command.mode == VEHICLE_MOTION_TURN && ok)
		ok = junction_cycle(&t, 0U, 0, 0U);
	ok = t.command.mode == VEHICLE_MOTION_STOP &&
	     t.now - spin_from > VEHICLE_JUNCTION_SPIN_MAX_MS - 20U &&
	     t.now - spin_from <= VEHICLE_JUNCTION_SPIN_MAX_MS + 20U && ok;
	ok = junction_cycle(&t, 1U, -1, 0U) && t.command.mode == TEST_NO_COMMAND &&
	     s_test_done_count == 0U && ok;
	ok = drain_controller_test_buffers() && ok;

	/* Without a turn callback a junction changes nothing. */
	s_config.junction_turn = NULL;
	ok = junction_arm(&t) && ok;
	ok = junction_cycle(&t, 1U, -1, 1U) &&
	     is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_SPEED_CM_S,
		       VEHICLE_LINE_TURN_CM_S) && ok;
	s_config.junction_turn = test_junction_turn;
	return drain_controller_test_buffers() && ok;
}

/* On a hump (IMU rising/falling) the straddle steering never starts its
 * blind search, however long the tape stays out of sight; level again, it
 * searches towards the last-seen side after VEHICLE_LINE_BLIND_MS. */
static BOOL controller_hump_test(void)
{
	junction_test_t t;
	uint32_t from;
	BOOL ok;

#if !VEHICLE_LINE_HUMP_NO_SEARCH
	/* Hump search inhibit off (as the bench test): a hump changes nothing,
	 * the robot still searches after VEHICLE_LINE_BLIND_MS. */
	s_config.junction_turn = NULL;
	ok = junction_arm(&t);
	t.terrain = (uint8_t)IMU_STATE_HUMP_RISING;
	from = t.now;
	while (t.now - from <= VEHICLE_LINE_BLIND_MS + 20U && ok &&
	       !is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_PIVOT_SPEED_CM_S,
			  VEHICLE_LINE_PIVOT_TURN_CM_S))
		ok = junction_cycle(&t, SEEN_GAP, 0U);
	ok = is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_PIVOT_SPEED_CM_S,
		       VEHICLE_LINE_PIVOT_TURN_CM_S) &&
	     t.now - from >= VEHICLE_LINE_BLIND_MS - 20U && ok;
	s_config = test_config;
	return drain_controller_test_buffers() && ok;
#endif
	s_config.junction_turn = NULL;
	ok = junction_arm(&t);                  /* tape last seen: centre (left) */
	t.terrain = (uint8_t)IMU_STATE_HUMP_RISING;
	from = t.now;
	while (t.now - from < VEHICLE_LINE_BLIND_MS + 200U && ok)
		ok = junction_cycle(&t, SEEN_GAP, 0U) && t.command.mode == VEHICLE_MOTION_VELOCITY &&
		     t.command.linear_velocity == VEHICLE_LINE_SPEED_CM_S;
	ok = is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_SPEED_CM_S, 0) && ok;
	t.terrain = (uint8_t)IMU_STATE_MONITORING;
	from = t.now;
	while (t.now - from <= VEHICLE_LINE_BLIND_MS + 20U && ok &&
	       !is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_PIVOT_SPEED_CM_S,
			  VEHICLE_LINE_PIVOT_TURN_CM_S))
		ok = junction_cycle(&t, SEEN_GAP, 0U);
	ok = is_motion(&t, VEHICLE_MOTION_VELOCITY, VEHICLE_LINE_PIVOT_SPEED_CM_S,
		       VEHICLE_LINE_PIVOT_TURN_CM_S) &&
	     t.now - from >= VEHICLE_LINE_BLIND_MS - 20U && ok;
	s_config = test_config;
	return drain_controller_test_buffers() && ok;
}

static char s_test_barcode[8];
static unsigned s_test_barcode_count;

static void test_barcode(void *context, const char *payload, uint8_t length)
{
	(void)context;
	if (length >= 1U && s_test_barcode_count < sizeof(s_test_barcode))
		s_test_barcode[s_test_barcode_count++] = payload[0];
}

static BOOL queue_test_barcode(uint32_t now, uint32_t sequence, char letter)
{
	barcode_event_message_t barcode = {0};

	barcode.header = (vehicle_message_header_t){VEHICLE_MESSAGE_BARCODE_EVENT,
						     VEHICLE_MESSAGE_VERSION, sequence, now};
	barcode.payload[0] = letter;
	barcode.length = 1U;
	barcode.valid = 1U;
	return tk_snd_mbf(g_mbf_barcode_events, &barcode, (INT)sizeof(barcode),
			  TMO_POL) == E_OK;
}

/* Decoded barcodes reach the hook in order; without a hook they are still
 * drained, so the line task's queue never fills. */
static BOOL controller_barcode_test(void)
{
	junction_test_t t;
	barcode_event_message_t left;
	BOOL ok;

	s_config.barcode = test_barcode;
	s_test_barcode_count = 0U;
	ok = junction_arm(&t);
	ok = queue_test_barcode(t.now, 1U, 'A') && queue_test_barcode(t.now, 2U, 'B') && ok;
	ok = junction_cycle(&t, 1U, -1, 0U) && s_test_barcode_count == 2U &&
	     s_test_barcode[0] == 'A' && s_test_barcode[1] == 'B' && ok;
	s_config.barcode = NULL;
	for (unsigned i = 0U; i < VEHICLE_MB_BARCODE_CAPACITY; i++)
		ok = queue_test_barcode(t.now, 3U + i, 'C') && ok;
	ok = junction_cycle(&t, 1U, -1, 0U) && s_test_barcode_count == 2U &&
	     tk_rcv_mbf(g_mbf_barcode_events, &left, TMO_POL) == E_TMOUT && ok;
	s_config = test_config;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_junction_test(void)
{
	BOOL ok;

	s_config.junction_turn = test_junction_turn;
	s_config.junction_done = test_junction_done;
	ok = junction_turn_test(VEHICLE_JUNCTION_LEFT, false);
	ok = junction_turn_test(VEHICLE_JUNCTION_RIGHT, false) && ok;
	ok = junction_turn_test(VEHICLE_JUNCTION_UTURN, false) && ok;
	ok = junction_turn_test(VEHICLE_JUNCTION_LEFT, true) && ok;
	ok = junction_turn_test(VEHICLE_JUNCTION_RIGHT, true) && ok;
	ok = junction_turn_test(VEHICLE_JUNCTION_UTURN, true) && ok;
	ok = junction_other_test() && ok;
	s_config = test_config;
	return ok;
}

/* Lent motors are accepted only while a bypass runs, and in statuses
 * stamped up to the moment its end was seen; a pick-up while following
 * latches the steering stop, once reported, and clears with the floor. */
static BOOL controller_lent_pickup_test(void)
{
	vehicle_controller_context_t context = {0};
	BOOL ok;

	ok = !lent_expected(&context, 100U);
	context.bypass_phase = BYPASS_RUNNING;
	ok = lent_expected(&context, 100U) && ok;
	context.bypass_phase = BYPASS_IDLE;
	context.bypass_done_ms = 200U;
	context.have_bypass_done = true;
	ok = lent_expected(&context, 199U) && lent_expected(&context, 200U) &&
	     !lent_expected(&context, 201U) && ok;

	context = (vehicle_controller_context_t){0};
	context.picked_up = true;
	pickup_step(&context, CONTROLLER_WAITING);
	ok = !context.pickup_reported && !context.steering.stop_latched && ok;
	context.junction_phase = JUNCTION_SPIN;
	pickup_step(&context, CONTROLLER_FOLLOWING);
	ok = context.pickup_reported && context.steering.stop_latched &&
	     context.junction_phase == JUNCTION_IDLE && ok;
	context.picked_up = false;
	pickup_step(&context, CONTROLLER_FOLLOWING);
	return !context.pickup_reported && ok;
}

/* Straddle line state for the dashboard: both white while following is on
 * the line (also pivoting or searching); LOST only once the steering gave
 * up; idle with both white is UNKNOWN; lifted is UNKNOWN. */
static BOOL controller_line_state_test(void)
{
	vehicle_controller_context_t c = {0};
	BOOL ok;

	c.line_position = POS_LOST;
	ok = straddle_line_state(&c, CONTROLLER_WAITING) == WIFI_TELEMETRY_LINE_UNKNOWN;
	c.line_position = POS_RIGHT;
	ok = straddle_line_state(&c, CONTROLLER_WAITING) == WIFI_TELEMETRY_LINE_FOUND && ok;
	c.line_position = POS_LOST;
	ok = straddle_init(&c.steering, &s_straddle_config) && ok;
	c.arm_requested = true;
	ok = straddle_line_state(&c, CONTROLLER_FOLLOWING) == WIFI_TELEMETRY_LINE_FOUND && ok;
	c.steering.state = STRADDLE_SEARCH;
	ok = straddle_line_state(&c, CONTROLLER_FOLLOWING) == WIFI_TELEMETRY_LINE_FOUND && ok;
	c.steering.stop_latched = true;
	ok = straddle_line_state(&c, CONTROLLER_FOLLOWING) == WIFI_TELEMETRY_LINE_LOST &&
	     straddle_line_state(&c, CONTROLLER_WAITING) == WIFI_TELEMETRY_LINE_LOST && ok;
	c.line_position = POS_LEFT;
	ok = straddle_line_state(&c, CONTROLLER_WAITING) == WIFI_TELEMETRY_LINE_FOUND && ok;
	c.picked_up = true;
	ok = straddle_line_state(&c, CONTROLLER_FOLLOWING) == WIFI_TELEMETRY_LINE_UNKNOWN && ok;
	return ok;
}

BOOL vehicle_controller_task_test(void)
{
	BOOL lent_ok;
	BOOL line_state_ok;
	BOOL arm_ok;
	BOOL fault_ok;
	BOOL emergency_ok;
	BOOL status_ok;
	BOOL stale_status_ok;
	BOOL drive_ok;
	BOOL hump_ok;
	BOOL junction_ok;
	BOOL barcode_ok;
	TEST_INIT();

	/* controller_cycle() reads the module config (s_config) for the motion
	   status deadline.  The task is not started during the boot-time test
	   run, so s_config would otherwise still be all zeros (max age 0 ms)
	   and every advancing-clock cycle would fault.  Install the test
	   config; vehicle_controller_task_start() overwrites it later. */
	s_config = test_config;

	arm_ok = controller_arm_test();
	fault_ok = controller_fault_test();
	emergency_ok = controller_local_emergency_test();
	status_ok = controller_nonidle_status_test();
	stale_status_ok = controller_stale_status_test();
	drive_ok = controller_drive_test();
	hump_ok = controller_hump_test();
	junction_ok = controller_junction_test();
	barcode_ok = controller_barcode_test();
	lent_ok = controller_lent_pickup_test();
	line_state_ok = controller_line_state_test();
	TEST_CHECK("VC-U13", arm_ok, "fresh safety inputs and local GO gate ARM/STOP commands");
	TEST_CHECK("VC-U14", emergency_ok, "local emergency input sends a latched emergency stop");
	TEST_CHECK("VC-U15", fault_ok, "missing sensor deadlines latch fault and request emergency stop");
	TEST_CHECK("VC-U16", status_ok, "nonzero commanded output status blocks arming and requests emergency stop");
	TEST_CHECK("VC-U17", stale_status_ok, "stale motion status faults despite fresh sensor observations");
	TEST_CHECK("VC-U22", drive_ok, "line following sends steering VELOCITY, accepts commanded motion, stops and faults on unexpected motion");
	TEST_CHECK("VC-U24", hump_ok, VEHICLE_LINE_HUMP_NO_SEARCH ?
		   "no blind search while the IMU reports a hump; level again, it searches towards the last-seen side" :
		   "hump search inhibit off: on a hump it still searches towards the last-seen side after the blind time");
	TEST_CHECK("VC-U26", junction_ok, "junction: approach, stop, spin until the new line reaches the leading sensor (left: into the gap; U-turn: second line), stop, resume; straight, timeout, no callback");
	TEST_CHECK("VC-U27", barcode_ok, "decoded barcodes are passed to the barcode hook in order and always drained");
	TEST_CHECK("VC-U28", lent_ok, "lent motors accepted only during a bypass; pick-up while following stops once per lift");
	TEST_CHECK("VC-U36", line_state_ok, "straddle line state: on the line while following (also pivoting or searching), lost only once given up, unknown idle with both white or lifted");
	TEST_SUMMARY("vehicle controller task");
	return arm_ok && fault_ok && emergency_ok && status_ok && stale_status_ok &&
	       drive_ok && hump_ok && junction_ok && barcode_ok && lent_ok && line_state_ok;
}
#endif
