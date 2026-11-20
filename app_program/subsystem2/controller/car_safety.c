#include "car_safety.h"
#include "car_gate.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"

#include <stddef.h>

static bool interval_valid(uint32_t value)
{
	return value != 0U && value <= INT32_MAX;
}

static bool fail(car_safety_t *safety, uint32_t fault)
{
	safety->faults |= fault;
	safety->armed = false;
	return false;
}

static bool clock_valid(car_safety_t *safety, uint32_t now)
{
	if (now - safety->last_clock_ms > INT32_MAX)
		return fail(safety, CAR_SAFETY_FAULT_CLOCK);
	safety->last_clock_ms = now;
	return true;
}

static bool check_deadlines(car_safety_t *safety, uint32_t now)
{
	if (now - (safety->line.have_message ? safety->line.timestamp_ms :
		   safety->line_wait_ms) > safety->config.line_max_age_ms)
		(void)fail(safety, CAR_SAFETY_FAULT_LINE);
	if (now - (safety->imu.have_message ? safety->imu.timestamp_ms :
		   safety->imu_wait_ms) > safety->config.imu_max_age_ms)
		(void)fail(safety, CAR_SAFETY_FAULT_IMU);
	if (safety->scanning) {
		if (now - safety->scan_start_ms > safety->config.scan_timeout_ms)
			(void)fail(safety, CAR_SAFETY_FAULT_OBSTACLE);
	} else if (now - (safety->have_ahead ? safety->obstacle.timestamp_ms :
			  safety->ahead_wait_ms) > safety->config.ahead_max_age_ms) {
		(void)fail(safety, CAR_SAFETY_FAULT_OBSTACLE);
	}
	return safety->faults == 0U;
}

static bool accept(car_safety_t *safety, car_safety_stream_t *stream,
		   const vehicle_message_header_t *header, uint16_t type,
		   uint32_t now, uint32_t max_age, uint32_t not_before, uint32_t fault)
{
	if (header == NULL || header->type != type ||
	    header->version != VEHICLE_MESSAGE_VERSION ||
	    now - header->timestamp_ms > max_age ||
	    (!stream->have_message && header->timestamp_ms - not_before > INT32_MAX) ||
	    (stream->have_message &&
	     (header->sequence - stream->sequence != 1U ||
	      header->timestamp_ms - stream->timestamp_ms > INT32_MAX)))
		return fail(safety, fault);
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
	    config->obstacle_stop_mm > OBS_MAX_RANGE_MM) {
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
	    message->intersection_detected > 1U || message->confidence > 1000U)
		return fail(safety, CAR_SAFETY_FAULT_LINE);
	if (!accept(safety, &safety->line, &message->header, VEHICLE_MESSAGE_LINE_OBSERVATION,
		    now_ms, safety->config.line_max_age_ms, safety->line_wait_ms,
		    CAR_SAFETY_FAULT_LINE))
		return false;
	safety->line_clear = message->line_detected == 1U &&
			     message->confidence >= safety->config.min_line_confidence;
	if (!safety->line_clear) safety->armed = false;
	return true;
}

bool car_safety_imu(car_safety_t *safety, const imu_estimate_message_t *message,
		    uint32_t now_ms)
{
	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms) || !check_deadlines(safety, now_ms)) return false;
	if (message == NULL || !(message->validity & VEHICLE_IMU_VALID_ROLL_PITCH) ||
	    message->shock_detected > 1U || message->tilt_detected > 1U)
		return fail(safety, CAR_SAFETY_FAULT_IMU);
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
	if (message == NULL || message->emergency > 1U ||
	    (message->distance_mm != OBS_DIST_NONE &&
	     (message->distance_mm < OBS_MIN_RANGE_MM || message->distance_mm > OBS_MAX_RANGE_MM)))
		return fail(safety, CAR_SAFETY_FAULT_OBSTACLE);
	max_age = safety->scanning ? safety->config.scan_timeout_ms :
				    safety->config.ahead_max_age_ms;
	if (!accept(safety, &safety->obstacle, &message->header, VEHICLE_MESSAGE_OBSTACLE_EVENT,
		    now_ms, max_age, safety->ahead_wait_ms, CAR_SAFETY_FAULT_OBSTACLE))
		return false;
	if (message->emergency) return fail(safety, CAR_SAFETY_FAULT_EMERGENCY);
	if (message->obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING && !safety->scanning) {
		if (!safety->have_ahead &&
		    message->header.timestamp_ms - safety->ahead_wait_ms > INT32_MAX)
			return fail(safety, CAR_SAFETY_FAULT_OBSTACLE);
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
			return fail(safety, CAR_SAFETY_FAULT_OBSTACLE);
		safety->scanning = false;
		safety->have_ahead = false;
		safety->ahead_clear = false;
		safety->ahead_wait_ms = now_ms;
		safety->armed = false;
		return true;
	}
	return fail(safety, CAR_SAFETY_FAULT_OBSTACLE);
}

bool car_safety_begin_scan(car_safety_t *safety, uint32_t now_ms)
{
	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms) || !check_deadlines(safety, now_ms)) return false;
	safety->armed = false;
	if (safety->faults != 0U || safety->scanning || !safety->have_ahead ||
	    now_ms - safety->obstacle.timestamp_ms > safety->config.ahead_max_age_ms)
		return false;
	safety->scanning = true;
	safety->scan_start_ms = now_ms;
	safety->have_ahead = false;
	safety->ahead_clear = false;
	return true;
}

void car_safety_latch_fault(car_safety_t *safety)
{
	if (safety == NULL || !safety->initialized) return;
	(void)fail(safety, CAR_SAFETY_FAULT_EXTERNAL);
}

bool car_safety_follow(car_safety_t *safety, uint32_t now_ms,
		       bool go, bool go_rising, bool emergency_stopped)
{
	bool blocked;

	if (safety == NULL || !safety->initialized) return false;
	if (!clock_valid(safety, now_ms)) return false;
	(void)check_deadlines(safety, now_ms);
	if (emergency_stopped) (void)fail(safety, CAR_SAFETY_FAULT_EMERGENCY);
	blocked = safety->scanning || !safety->line.have_message || !safety->line_clear ||
		  !safety->imu.have_message || !safety->imu_clear ||
		  !safety->have_ahead || !safety->ahead_clear;
	if (!car_gate_follow(go, emergency_stopped, safety->faults, blocked)) {
		safety->armed = false;
	} else if (!safety->armed &&
		   car_gate_rearm(go_rising, emergency_stopped, safety->faults, blocked)) {
		safety->armed = true;
	}
	return safety->armed;
}
