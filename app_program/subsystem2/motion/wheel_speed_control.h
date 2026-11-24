
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

    /*
     * Optional per-wheel feed-forward. Each wheel's PID corrects around
     *   base_pwm_percent + ff_pwm_per_cm_s * (that wheel's target - ff_ref_speed_cm_s)
     * (clamped to 0..max), i.e. base_pwm_percent is the PWM that holds
     * ff_ref_speed_cm_s. Steering then lands immediately: the inner wheel of
     * a turn starts lower and the outer higher, instead of both starting at
     * base and waiting for the integral. 0 (the default when omitted) keeps
     * the flat base for both wheels. ff_ref_speed_cm_s must be > 0 when
     * ff_pwm_per_cm_s > 0.
     */
    float ff_pwm_per_cm_s;
    float ff_ref_speed_cm_s;
} wheel_speed_config_t;

typedef struct
{
    pid_controller_t left_pid;
    pid_controller_t right_pid;

    wheel_speed_config_t config;

    /* Per-wheel targets; both start at config.target_speed_cm_s. */
    float left_target_cm_s;
    float right_target_cm_s;

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
 * commands if input validation fails. A failed
 * update zeros the output and resets both PIDs;
 * the caller must stop the motors.
 */
bool wheel_speed_control_update(
    wheel_speed_control_t *controller,
    float left_speed_cm_s,
    float right_speed_cm_s,
    float dt_seconds,
    wheel_speed_output_t *output);

/*
 * Set separate target speeds for steering
 * (cm/s magnitudes, both finite and > 0).
 *
 * PID state is kept so steering changes are
 * smooth. Returns false and changes nothing
 * if a target is invalid.
 */
bool wheel_speed_control_set_targets(
    wheel_speed_control_t *controller,
    float left_target_cm_s,
    float right_target_cm_s);

/*
 * Reset both PID controllers.
 *
 * Call whenever movement stops,
 * changes direction or is restarted.
 */
bool wheel_speed_control_reset(
    wheel_speed_control_t *controller);

#endif
