#include "motion_control.h"

#include <stddef.h>

#define MOTION_MAX_CONTROL_INTERVAL_MS 250U

bool motion_control_start(motion_control_t *control,
			  const motion_control_config_t *config,
			  int8_t direction, uint32_t now_ms)
{
	motion_control_t initial = {0};

	if (control == NULL) return false;
	if (config == NULL || (direction != 1 && direction != -1) ||
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
	initial.running = true;
	*control = initial;
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

	if (control == NULL || output == NULL || !control->running || control->faulted)
		return fault(control, output);
	last_sample = control->feedback.have_baseline ?
		      control->feedback.last_timestamp_ms : control->start_ms;
	if (now_ms - control->last_check_ms > INT32_MAX ||
	    now_ms - last_sample > control->feedback.config.max_age_ms)
		return fault(control, output);
	control->last_check_ms = now_ms;
	if (message != NULL) {
		if (message->header.timestamp_ms - control->start_ms > INT32_MAX)
			return fault(control, output);
		result = motion_feedback_update(&control->feedback, message, now_ms, &speeds);
		if (result == MOTION_FEEDBACK_INVALID) return fault(control, output);
		if (result == MOTION_FEEDBACK_READY) {
			left = speeds.left_cm_per_second * control->direction;
			right = speeds.right_cm_per_second * control->direction;
			if (left < 0.0f || right < 0.0f ||
			    !wheel_speed_control_update(&control->speed, left, right,
							speeds.elapsed_seconds, &control->output))
				return fault(control, output);
			control->have_output = true;
		}
	}
	*output = control->output;
	return control->have_output ? MOTION_CONTROL_READY : MOTION_CONTROL_WAITING;
}
