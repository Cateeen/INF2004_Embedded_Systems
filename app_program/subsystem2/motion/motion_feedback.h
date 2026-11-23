#ifndef MOTION_FEEDBACK_H
#define MOTION_FEEDBACK_H

#include "../../subsystem6/odometry/wheel_odometry.h"
#include "../../vehicle_messages.h"

typedef struct {
	wheel_odometry_config_t left;
	wheel_odometry_config_t right;
	uint32_t max_age_ms;
	uint32_t max_interval_ms;
} motion_feedback_config_t;

typedef struct {
	motion_feedback_config_t config;
	uint32_t last_left;
	uint32_t last_right;
	uint32_t last_sequence;
	uint32_t last_timestamp_ms;
	bool initialized;
	bool have_baseline;
} motion_feedback_t;

typedef struct {
	uint32_t timestamp_ms;
	float elapsed_seconds;
	/* Signed speeds after odometry's per-wheel polarity normalization. */
	float left_cm_per_second;
	float right_cm_per_second;
} motion_wheel_speeds_t;

typedef enum {
	MOTION_FEEDBACK_INVALID = 0,
	MOTION_FEEDBACK_BASELINE,
	MOTION_FEEDBACK_READY
} motion_feedback_result_t;

/* Calibration must be independently verified; the owner is the motion task. */
bool motion_feedback_init(motion_feedback_t *feedback,
			  const motion_feedback_config_t *config);

/* Messages must be fresh, valid and consecutive; timestamps and wheel counts
 * may wrap, but differences of half a counter range are ambiguous. The first
 * valid message establishes a baseline without changing speeds. On failure,
 * state and speeds are unchanged. The caller must stop on failure, check
 * direction and freshness, and reset the baseline when restarting control. */
motion_feedback_result_t motion_feedback_update(motion_feedback_t *feedback,
				 const wheel_odometry_message_t *message,
				 uint32_t now_ms, motion_wheel_speeds_t *speeds);

#endif
