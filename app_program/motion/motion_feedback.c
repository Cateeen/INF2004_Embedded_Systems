#include "motion_feedback.h"

#include <stddef.h>

bool motion_feedback_init(motion_feedback_t *feedback,
			  const motion_feedback_config_t *config)
{
	motion_feedback_t initial = {0};
	double distance;

	if (feedback == NULL || config == NULL || config->max_age_ms == 0U ||
	    config->max_age_ms > INT32_MAX || config->max_interval_ms == 0U ||
	    config->max_interval_ms > INT32_MAX ||
	    !wheel_odometry_distance_precise(&config->left, 0, &distance) ||
	    !wheel_odometry_distance_precise(&config->right, 0, &distance))
		return false;
	initial.config = *config;
	initial.initialized = true;
	*feedback = initial;
	return true;
}

motion_feedback_result_t motion_feedback_update(motion_feedback_t *feedback,
				 const wheel_odometry_message_t *message,
				 uint32_t now_ms, motion_wheel_speeds_t *speeds)
{
	motion_wheel_speeds_t next;
	wheel_odometry_result_t left, right;
	uint32_t left_count, right_count, elapsed;
	int32_t left_delta, right_delta;

	if (feedback == NULL || message == NULL || speeds == NULL ||
	    !feedback->initialized ||
	    message->header.type != VEHICLE_MESSAGE_WHEEL_ODOMETRY ||
	    message->header.version != VEHICLE_MESSAGE_VERSION || message->valid != 1U ||
	    now_ms - message->header.timestamp_ms > feedback->config.max_age_ms)
		return MOTION_FEEDBACK_INVALID;
	left_count = (uint32_t)message->left_count;
	right_count = (uint32_t)message->right_count;
	if (feedback->have_baseline) {
		elapsed = message->header.timestamp_ms - feedback->last_timestamp_ms;
		if (message->header.sequence - feedback->last_sequence != 1U ||
		    elapsed == 0U || elapsed > feedback->config.max_interval_ms ||
		    !wheel_odometry_count_delta(left_count, feedback->last_left, &left_delta) ||
		    !wheel_odometry_count_delta(right_count, feedback->last_right, &right_delta))
			return MOTION_FEEDBACK_INVALID;
		next.elapsed_seconds = (float)elapsed / 1000.0f;
		if (!wheel_odometry_calculate(&feedback->config.left, left_delta,
					       next.elapsed_seconds, &left) ||
		    !wheel_odometry_calculate(&feedback->config.right, right_delta,
					       next.elapsed_seconds, &right))
			return MOTION_FEEDBACK_INVALID;
		next.timestamp_ms = message->header.timestamp_ms;
		next.left_cm_per_second = left.speed_cm_per_second;
		next.right_cm_per_second = right.speed_cm_per_second;
	}
	feedback->last_left = left_count;
	feedback->last_right = right_count;
	feedback->last_sequence = message->header.sequence;
	feedback->last_timestamp_ms = message->header.timestamp_ms;
	if (!feedback->have_baseline) {
		feedback->have_baseline = true;
		return MOTION_FEEDBACK_BASELINE;
	}
	*speeds = next;
	return MOTION_FEEDBACK_READY;
}
