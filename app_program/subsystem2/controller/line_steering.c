#include "line_steering.h"
#include "../../subsystem3/line_barcode/line_observation.h"

#include <stddef.h>

#define FORWARD_PWM 25
#define CORRECTION_PWM 10
#define LOST_CONTINUE_MS 300U

void line_steering_reset(line_steering_t *steering)
{
	if (steering == NULL) return;
	*steering = (line_steering_t){0};
}

line_steering_command_t line_steering_update(
	line_steering_t *steering, uint8_t position, uint32_t now_ms)
{
	line_steering_command_t stop = {0, 0, true};
	line_steering_command_t command = {0, 0, false};

	if (steering == NULL || steering->stop_latched) return stop;

	if (position == POS_LOST) {
		if (!steering->line_lost) {
			steering->line_lost = true;
			steering->lost_since_ms = now_ms;
		}
		if ((uint32_t)(now_ms - steering->lost_since_ms) >= LOST_CONTINUE_MS) {
			steering->stop_latched = true;
			return stop;
		}
		command.motor_1_pwm = steering->last_motor_1_pwm;
		command.motor_2_pwm = steering->last_motor_2_pwm;
		return command;
	}

	steering->line_lost = false;
	switch (position) {
	case POS_JUNCTION:
		command.motor_1_pwm = FORWARD_PWM;
		command.motor_2_pwm = -FORWARD_PWM;
		break;
	case POS_LEFT:
		command.motor_1_pwm = FORWARD_PWM - CORRECTION_PWM;
		command.motor_2_pwm = -FORWARD_PWM;
		break;
	case POS_RIGHT:
		command.motor_1_pwm = FORWARD_PWM;
		command.motor_2_pwm = -(FORWARD_PWM - CORRECTION_PWM);
		break;
	default:
		steering->stop_latched = true;
		return stop;
	}

	steering->last_motor_1_pwm = command.motor_1_pwm;
	steering->last_motor_2_pwm = command.motor_2_pwm;
	return command;
}
