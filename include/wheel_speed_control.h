
#ifndef WHEEL_SPEED_CONTROL_H
#define WHEEL_SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "pid_controller.h"

/*
 * Buddy 2 - Closed-Loop Wheel Speed Control
 *
 * Calculates PWM commands using two PID
 * controllers and measured wheel speeds.
 *
 * No direct motor hardware access.
 *
 * Target and measured speeds are expressed
 * as positive forward-equivalent magnitudes
 * in cm/s.
 *
 * Physical motor direction is handled by
 * the motion controller separately.
 */

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target_speed_cm_s;

    uint8_t base_pwm_percent;
    uint8_t max_pwm_percent;
} wheel_speed_config_t;

typedef struct
{
    pid_controller_t left_pid;
    pid_controller_t right_pid;

    wheel_speed_config_t config;

    bool initialized;
} wheel_speed_control_t;

typedef struct
{
    uint8_t left_pwm_percent;
    uint8_t right_pwm_percent;
} wheel_speed_output_t;

/*
 * Initialise both wheel PID controllers.
 *
 * Rejects invalid or non-finite settings.
 */
bool wheel_speed_control_init(
    wheel_speed_control_t *controller,
    const wheel_speed_config_t *config);

/*
 * Compute corrected PWM values.
 *
 * Measured speeds must be valid
 * non-negative wheel-speed magnitudes.
 *
 * dt_seconds must be greater than zero.
 *
 * Returns false without issuing motor
 * commands if input validation fails.
 */
bool wheel_speed_control_update(
    wheel_speed_control_t *controller,
    float left_speed_cm_s,
    float right_speed_cm_s,
    float dt_seconds,
    wheel_speed_output_t *output);

/*
 * Reset both PID controllers.
 *
 * Call whenever movement stops,
 * changes direction or is restarted.
 */
void wheel_speed_control_reset(
    wheel_speed_control_t *controller);

#endif
