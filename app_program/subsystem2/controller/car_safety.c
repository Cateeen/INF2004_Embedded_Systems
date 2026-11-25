#include "car_safety.h"
#include "car_gate.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"

#include <stddef.h>

static bool interval_valid(uint32_t value)
{
	return value != 0U && value <= INT32_MAX;
}

static bool fail(car_safety_t *safety, uint32_t fault, const char *why)
{
	if (safety->fault_reason == NULL) safety->fault_reason = why;
	safety->faults |= fault;
	safety->armed = false;
	return false;
}

static bool clock_valid(car_safety_t *safety, uint32_t now)
{
	if (now - safety->last_clock_ms > INT32_MAX)
		return fail(safety, CAR_SAFETY_FAULT_CLOCK, "clock went backwards");
	safety->last_clock_ms = now;
	return true;
}

/* Before a stream's first message, its deadline is the start-up grace. */
static uint32_t first_deadline(const car_safety_t *safety, bool have_message,
			       uint32_t max_age)
{
	if (have_message || safety->config.startup_grace_ms <= max_age) return max_age;
	return safety->config.startup_grace_ms;
}

static bool check_deadlines(car_safety_t *safety, uint32_t now)
{
	if (now - (safety->line.have_message ? safety->line.timestamp_ms :
		   safety->line_wait_ms) >
	    first_deadline(safety, safety->line.have_message, safety->config.line_max_age_ms))
		(void)fail(safety, CAR_SAFETY_FAULT_LINE,
			   safety->line.have_message ? "line: observations stopped (deadline)" :
			   "line: no first observation in time");
	if (now - (safety->imu.have_message ? safety->imu.timestamp_ms :
		   safety->imu_wait_ms) >
	    first_deadline(safety, safety->imu.have_message, safety->config.imu_max_age_ms))
		(void)fail(safety, CAR_SAFETY_FAULT_IMU,
			   safety->imu.have_message ? "imu: estimates stopped (deadline)" :
			   "imu: no first estimate in time");
	if (safety->obstacle_disabled) {
		/* No ultrasonic: no obstacle deadlines. */
	} else if (safety->scanning) {
		if (now - safety->scan_start_ms > safety->config.scan_timeout_ms)
			(void)fail(safety, CAR_SAFETY_FAULT_OBSTACLE, "obstacle: scan timed out");
	} else if (now - (safety->have_ahead ? safety->obstacle.timestamp_ms :
			  safety->ahead_wait_ms) >
		   first_deadline(safety, safety->obstacle.have_message,
				  safety->config.ahead_max_age_ms)) {
		if (!car_safety_disable_obstacle(safety))
			(void)fail(safety, CAR_SAFETY_FAULT_OBSTACLE,
				   safety->obstacle.have_message ?
				   "obstacle: ahead readings stopped (deadline)" :
				   "obstacle: no first ahead reading in time");
	}
	return safety->faults == 0U;
}

static bool accept(car_safety_t *safety, car_safety_stream_t *stream,
		   const vehicle_message_header_t *header, uint16_t type,
		   uint32_t now, uint32_t max_age, uint32_t not_before, uint32_t fault)
{
	if (header == NULL || header->type != type ||
	    header->version != VEHICLE_MESSAGE_VERSION)
		return fail(safety, fault, fault == CAR_SAFETY_FAULT_LINE ? "line: wrong message type" :
			    fault == CAR_SAFETY_FAULT_IMU ? "imu: wrong message type" :
			    "obstacle: wrong message type");
	if (now - header->timestamp_ms > max_age)
		return fail(safety, fault, fault == CAR_SAFETY_FAULT_LINE ? "line: message too old" :
			    fault == CAR_SAFETY_FAULT_IMU ? "imu: message too old" :
			    "obstacle: message too old");
	if (!stream->have_message && header->timestamp_ms - not_before > INT32_MAX)
		return fail(safety, fault, fault == CAR_SAFETY_FAULT_LINE ? "line: first message from before start" :
			    fault == CAR_SAFETY_FAULT_IMU ? "imu: first message from before start" :
			    "obstacle: first message from before start");
	if (stream->have_message &&
	    (header->sequence - stream->sequence != 1U ||
	     header->timestamp_ms - stream->timestamp_ms > INT32_MAX))
		return fail(safety, fault, fault == CAR_SAFETY_FAULT_LINE ? "line: message out of sequence" :
			    fault == CAR_SAFETY_FAULT_IMU ? "imu: message out of sequence" :
			    "obstacle: message out of sequence");
	stream->sequence = header->sequence;
	stream->timestamp_ms = header->timestamp_ms;
	stream->have_message = true;
	return true;
}

bool car_safety_init(car_safety_t *safety, const car_safety_config_t *config,
		     uint32_t now_ms)
{
	car_safety_t initial = {0};

	if (safety == NULL) return false;
	if (config == NULL ||
	    !interval_valid(config->line_max_age_ms) ||
	    !interval_valid(config->imu_max_age_ms) ||
	    !interval_valid(config->ahead_max_age_ms) ||
	    !interval_valid(config->scan_timeout_ms) ||
	    config->min_line_confidence > 1000U ||
	    config->obstacle_stop_mm < OBS_MIN_RANGE_MM ||
	    config->obstacle_stop_mm > OBS_MAX_RANGE_MM ||
	    config->startup_grace_ms > INT32_MAX) {
		*safety = (car_safety_t){.faults = CAR_SAFETY_FAULT_EXTERNAL};
		return false;
	}
	initial.config = *config;
	initial.last_clock_ms = now_ms;
	initial.line_wait_ms = now_ms;
	initial.imu_wait_ms = now_ms;
	initial.ahead_wait_ms = now_ms;
	initial.initialized = true;
	*safety = initial;
	return true;
}

bool car_safety_line(car_safety_t *safety, const line_observation_message_t *message,
		     uint32_t now_ms)
{
	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms) || !check_deadlines(safety, now_ms)) return false;
	if (message == NULL || message->line_detected > 1U ||
	    message->intersection_detected > 1U || message->picked_up > 1U ||
	    message->confidence > 1000U)
		return fail(safety, CAR_SAFETY_FAULT_LINE, "line: invalid field");
	if (!accept(safety, &safety->line, &message->header, VEHICLE_MESSAGE_LINE_OBSERVATION,
		    now_ms, safety->config.line_max_age_ms, safety->line_wait_ms,
		    CAR_SAFETY_FAULT_LINE))
		return false;
	/* The line is required to ARM (car_safety_follow); once following, a
	 * lost line is handled by the steering (300 ms hold, then a latched
	 * stop) and is expected during a junction turn, so it does not disarm. */
	safety->line_clear = safety->config.straddle_line ?
			     message->picked_up == 0U :
			     message->line_detected == 1U &&
			     message->confidence >= safety->config.min_line_confidence;
	return true;
}

bool car_safety_imu(car_safety_t *safety, const imu_estimate_message_t *message,
		    uint32_t now_ms)
{
	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms) || !check_deadlines(safety, now_ms)) return false;
	if (message == NULL || !(message->validity & VEHICLE_IMU_VALID_ROLL_PITCH) ||
	    message->shock_detected > 1U || message->tilt_detected > 1U)
		return fail(safety, CAR_SAFETY_FAULT_IMU, "imu: invalid estimate or flags");
	if (!accept(safety, &safety->imu, &message->header, VEHICLE_MESSAGE_IMU_ESTIMATE,
		    now_ms, safety->config.imu_max_age_ms, safety->imu_wait_ms,
		    CAR_SAFETY_FAULT_IMU))
		return false;
	safety->imu_clear = !message->shock_detected && !message->tilt_detected;
	if (!safety->imu_clear) safety->armed = false;
	return true;
}

bool car_safety_obstacle(car_safety_t *safety, const obstacle_event_message_t *message,
			 uint32_t now_ms)
{
	uint32_t max_age;

	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms) || !check_deadlines(safety, now_ms)) return false;
	if (safety->obstacle_disabled) return true;     /* switched off: ignored */
	if (message == NULL || message->emergency > 1U ||
	    (message->distance_mm != OBS_DIST_NONE &&
	     (message->distance_mm < OBS_MIN_RANGE_MM || message->distance_mm > OBS_MAX_RANGE_MM)))
		return fail(safety, CAR_SAFETY_FAULT_OBSTACLE, "obstacle: invalid distance or flag");
	max_age = safety->scanning ? safety->config.scan_timeout_ms :
				    safety->config.ahead_max_age_ms;
	if (!accept(safety, &safety->obstacle, &message->header, VEHICLE_MESSAGE_OBSTACLE_EVENT,
		    now_ms, max_age, safety->ahead_wait_ms, CAR_SAFETY_FAULT_OBSTACLE))
		return false;
	if (message->emergency) safety->armed = false;      /* stop at once; no latch */
	if (message->obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING && !safety->scanning) {
		if (!safety->have_ahead &&
		    message->header.timestamp_ms - safety->ahead_wait_ms > INT32_MAX)
			return fail(safety, CAR_SAFETY_FAULT_OBSTACLE,
				    "obstacle: ahead reading from before the wait began");
		safety->have_ahead = true;
		safety->ahead_clear = message->distance_mm == OBS_DIST_NONE ||
				      message->distance_mm > safety->config.obstacle_stop_mm;
		if (!safety->ahead_clear) safety->armed = false;
		return true;
	}
	if (safety->scanning &&
	    (message->obstacle_class == VEHICLE_OBSTACLE_SCAN_CLEAR ||
	     message->obstacle_class == VEHICLE_OBSTACLE_SCAN_FOUND)) {
		if (message->header.timestamp_ms - safety->scan_start_ms > INT32_MAX)
			return fail(safety, CAR_SAFETY_FAULT_OBSTACLE,
				    "obstacle: scan result from before the scan");
		safety->scanning = false;
		safety->have_ahead = false;
		safety->ahead_clear = false;
		safety->ahead_wait_ms = now_ms;
		safety->armed = false;
		return true;
	}
	return fail(safety, CAR_SAFETY_FAULT_OBSTACLE, "obstacle: unexpected message class");
}

bool car_safety_begin_scan(car_safety_t *safety, uint32_t now_ms)
{
	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms) || !check_deadlines(safety, now_ms)) return false;
	safety->armed = false;
	if (safety->faults != 0U || safety->obstacle_disabled ||
	    safety->scanning || !safety->have_ahead ||
	    now_ms - safety->obstacle.timestamp_ms > safety->config.ahead_max_age_ms)
		return false;
	safety->scanning = true;
	safety->scan_start_ms = now_ms;
	safety->have_ahead = false;
	safety->ahead_clear = false;
	return true;
}

bool car_safety_disable_obstacle(car_safety_t *safety)
{
	if (safety == NULL || !safety->initialized || !safety->config.obstacle_optional ||
	    safety->obstacle.have_message)
		return false;
	safety->obstacle_disabled = true;
	safety->scanning = false;
	safety->have_ahead = true;      /* ahead counts as clear from now on */
	safety->ahead_clear = true;
	return true;
}

void car_safety_latch_fault(car_safety_t *safety)
{
	if (safety == NULL || !safety->initialized) return;
	(void)fail(safety, CAR_SAFETY_FAULT_EXTERNAL,
		   "latched by the controller (subsystem fault, bad message or deadline)");
}

bool car_safety_follow(car_safety_t *safety, uint32_t now_ms,
		       bool go, bool go_rising, bool emergency_stopped)
{
	bool blocked;
	bool arm_blocked;

	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms)) return false;
	(void)check_deadlines(safety, now_ms);
	if (emergency_stopped)
		(void)fail(safety, CAR_SAFETY_FAULT_EMERGENCY, "emergency-stop input");
	/* Hazards stop a following robot at once. The line must be seen to
	 * (re)arm, but losing it while following is the steering's job. */
	blocked = safety->scanning || !safety->line.have_message ||
		  !safety->imu.have_message || !safety->imu_clear ||
		  !safety->have_ahead || !safety->ahead_clear;
	arm_blocked = blocked || !safety->line_clear;
	if (!car_gate_follow(go, emergency_stopped, safety->faults, blocked)) {
		safety->armed = false;
	} else if (!safety->armed &&
		   car_gate_rearm(go_rising, emergency_stopped, safety->faults, arm_blocked)) {
		safety->armed = true;
	}
	return safety->armed;
}

bool car_safety_resume(car_safety_t *safety, uint32_t now_ms,
		       bool go, bool emergency_stopped)
{
	/* The same gate as a GO press (go_rising), only without the press. */
	if (!go) return false;
	return car_safety_follow(safety, now_ms, go, true, emergency_stopped);
}
