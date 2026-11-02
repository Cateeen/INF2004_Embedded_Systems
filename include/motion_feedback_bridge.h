
#ifndef MOTION_FEEDBACK_BRIDGE_H
#define MOTION_FEEDBACK_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_control.h"
#include "wheel_feedback_pid.h"

/*
 * Buddy 2 - Motion Feedback Bridge
 *
 * Connects the wheel-feedback PID adapter
 * to the motion-control callback.
 *
 * The callback does not directly command motors.
 */

typedef struct
{
    wheel_feedback_pid_t *controller;

    uint64_t last_left_pulse_us;
    uint64_t last_right_pulse_us;

    bool initialized;
} motion_feedback_bridge_t;

bool motion_feedback_bridge_init(
    motion_feedback_bridge_t *bridge,
    wheel_feedback_pid_t *controller);

/*
 * Pass this function to motion_set_feedback().
 *
 * If it returns false, motion_control.c
 * emergency-stops both motors.
 */
bool motion_feedback_bridge_update(
    void *context,
    motion_direction_t direction,
    uint64_t current_time_us,
    bool starting,
    int16_t *motor_1_pwm,
    int16_t *motor_2_pwm);

#endif
