#include "motion_control.h"

#include <stddef.h>

#define MOTION_MAX_CONTROL_INTERVAL_MS 250U
/* Backward speed (cm/s) over one odometry sample that still counts as a
 * stopped wheel rather than a fault. One count in 50 ms is 0.16 cm/s; a
 * reversed motor or encoder at line speed is ~10 cm/s. */
#define MOTION_REVERSE_TOLERANCE_CM_S 2.0f

static bool start(motion_control_t *control, const motion_control_config_t *config,
		  int8_t direction, int8_t left, int8_t right, uint32_t now_ms)
{
	motion_control_t initial = {0};

	if (control == NULL) return false;
	if (config == NULL || (left != 1 && left != -1) || (right != 1 && right != -1) ||
	    config->feedback.max_interval_ms > MOTION_MAX_CONTROL_INTERVAL_MS ||
	    config->feedback.max_age_ms > MOTION_MAX_CONTROL_INTERVAL_MS ||
	    !motion_feedback_init(&initial.feedback, &config->feedback) ||
	    !wheel_speed_control_init(&initial.speed, &config->speed)) {
		*control = (motion_control_t){.faulted = true};
		return false;
	}
	initial.start_ms = now_ms;
	initial.last_check_ms = now_ms;
	initial.direction = direction;
	initial.left_direction = left;
	initial.right_direction = right;
	initial.running = true;
	*control = initial;
	return true;
}

bool motion_control_start(motion_control_t *control,
			  const motion_control_config_t *config,
			  int8_t direction, uint32_t now_ms)
{
	if (direction != 1 && direction != -1) {
		if (control != NULL) *control = (motion_control_t){.faulted = true};
		return false;
	}
	return start(control, config, direction, direction, direction, now_ms);
}

bool motion_control_start_spin(motion_control_t *control,
			       const motion_control_config_t *config,
			       int8_t turn_direction, uint32_t now_ms)
{
	if (turn_direction != 1 && turn_direction != -1) {
		if (control != NULL) *control = (motion_control_t){.faulted = true};
		return false;
	}
	return start(control, config, 0, (int8_t)-turn_direction, turn_direction, now_ms);
}

bool motion_control_set_targets(motion_control_t *control,
				float left_cm_s, float right_cm_s)
{
	bool changed;
	wheel_speed_output_t output;

	if (control == NULL || !control->running || control->faulted) return false;
	changed = left_cm_s != control->speed.left_target_cm_s ||
		  right_cm_s != control->speed.right_target_cm_s;
	if (!wheel_speed_control_set_targets(&control->speed, left_cm_s, right_cm_s))
		return false;
	/* Apply a steering change now, not at the next odometry sample. */
	if (changed && control->have_output && control->have_speeds &&
	    wheel_speed_control_retarget_output(&control->speed, control->left_speed_cm_s,
						control->right_speed_cm_s, &output))
		control->output = output;
	return true;
}

void motion_control_stop(motion_control_t *control)
{
	if (control == NULL) return;
	control->output = (wheel_speed_output_t){0};
	if (control->speed.initialized)
		(void)wheel_speed_control_reset(&control->speed);
	control->have_output = false;
	control->running = false;
}

static motion_control_result_t fault(motion_control_t *control,
				     wheel_speed_output_t *output)
{
	if (output != NULL) *output = (wheel_speed_output_t){0};
	if (control != NULL) {
		motion_control_stop(control);
		control->faulted = true;
	}
	return MOTION_CONTROL_FAULT;
}

motion_control_result_t motion_control_step(motion_control_t *control,
			 const wheel_odometry_message_t *message,
			 uint32_t now_ms, wheel_speed_output_t *output)
{
	motion_feedback_result_t result;
	motion_wheel_speeds_t speeds;
	uint32_t last_sample;
	float left, right;

	if (control == NULL || output == NULL || !control->running || control->faulted) {
		if (control != NULL && control->fault_reason == NULL)
			control->fault_reason = "control not running";
		return fault(control, output);
	}
	last_sample = control->feedback.have_baseline ?
		      control->feedback.last_timestamp_ms : control->start_ms;
	if (now_ms - control->last_check_ms > INT32_MAX) {
		control->fault_reason = "clock went backwards";
		return fault(control, output);
	}
	if (now_ms - last_sample > control->feedback.config.max_age_ms) {
		control->fault_reason = "no new odometry in time (ms since last)";
		control->fault_value = now_ms - last_sample;
		return fault(control, output);
	}
	control->last_check_ms = now_ms;
	if (message != NULL) {
		if (message->header.timestamp_ms - control->start_ms > INT32_MAX) {
			control->fault_reason = "odometry sample from before the motion started";
			return fault(control, output);
		}
		result = motion_feedback_update(&control->feedback, message, now_ms, &speeds);
		if (result == MOTION_FEEDBACK_INVALID) {
			control->fault_reason = control->feedback.reject_reason != NULL ?
						control->feedback.reject_reason :
						"odometry rejected";
			control->fault_value = control->feedback.reject_value;
			return fault(control, output);
		}
		if (result == MOTION_FEEDBACK_READY) {
			left = speeds.left_cm_per_second * control->left_direction;
			right = speeds.right_cm_per_second * control->right_direction;
			/* A wheel measured slightly backwards over one sample (the
			 * near-stopped inner wheel of a pivot rocking back, or one
			 * noisy encoder count) is treated as stopped, as the
			 * line_follow bench test does. Clearly backwards is still a
			 * fault: wrong polarity or the robot being pushed. */
			if (left < -MOTION_REVERSE_TOLERANCE_CM_S ||
			    right < -MOTION_REVERSE_TOLERANCE_CM_S) {
				control->fault_reason = "a wheel ran backwards";
				control->fault_left_cm_s = left;
				control->fault_right_cm_s = right;
				return fault(control, output);
			}
			if (left < 0.0f) left = 0.0f;
			if (right < 0.0f) right = 0.0f;
			if (!wheel_speed_control_update(&control->speed, left, right,
							speeds.elapsed_seconds, &control->output)) {
				control->fault_reason = "wheel speed PID update failed";
				return fault(control, output);
			}
			control->left_speed_cm_s = left;
			control->right_speed_cm_s = right;
			control->have_speeds = true;
			control->have_output = true;
		}
	}
	*output = control->output;
	return control->have_output ? MOTION_CONTROL_READY : MOTION_CONTROL_WAITING;
}
