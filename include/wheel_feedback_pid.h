
#ifndef WHEEL_FEEDBACK_PID_H
#define WHEEL_FEEDBACK_PID_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_control.h"
#include "wheel_feedback.h"
#include "wheel_speed_control.h"

/*
 * Buddy 2 - Encoder Feedback and PID Integration
 *
 * Calculates signed PWM requests from encoder feedback.
 * Does NOT command the motor hardware directly.
 */

typedef struct
{
    wheel_feedback_t *left;
    wheel_feedback_t *right;

    wheel_speed_control_t speed_controller;

    bool left_positive_forward;
    bool right_positive_forward;

    uint64_t previous_update_us;
    bool initialized;
} wheel_feedback_pid_t;

typedef struct
{
    int16_t motor_1_pwm;
    int16_t motor_2_pwm;
} wheel_feedback_pid_output_t;

/*
 * The two wheel-feedback objects must already have
 * valid calibration and remain alive throughout use.
 *
 * Encoder polarity flags indicate whether increasing
 * counts correspond to forward vehicle wheel travel.
 */
bool wheel_feedback_pid_init(
    wheel_feedback_pid_t *controller,
    wheel_feedback_t *left,
    wheel_feedback_t *right,
    const wheel_speed_config_t *speed_config,
    bool left_positive_forward,
    bool right_positive_forward,
    uint64_t current_time_us);

/*
 * Calculate one feedback update.
 *
 * On failure, both output PWM values are zero.
 * The caller must brake or emergency-stop the motors
 * when this function returns false.
 */
bool wheel_feedback_pid_update(
    wheel_feedback_pid_t *controller,
    motion_direction_t direction,
    uint64_t current_time_us,
    wheel_feedback_pid_output_t *output);

/* Reset the PID accumulated errors. */
void wheel_feedback_pid_reset(
    wheel_feedback_pid_t *controller);

#endif
