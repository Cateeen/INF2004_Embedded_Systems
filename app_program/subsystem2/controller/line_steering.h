#ifndef LINE_STEERING_H
#define LINE_STEERING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	int16_t motor_1_pwm;
	int16_t motor_2_pwm;
	bool stop_latched;
} line_steering_command_t;

typedef struct {
	uint32_t lost_since_ms;
	int16_t last_motor_1_pwm;
	int16_t last_motor_2_pwm;
	bool line_lost;
	bool stop_latched;
} line_steering_t;

void line_steering_reset(line_steering_t *steering);
line_steering_command_t line_steering_update(
	line_steering_t *steering, uint8_t position, uint32_t now_ms);

#endif
