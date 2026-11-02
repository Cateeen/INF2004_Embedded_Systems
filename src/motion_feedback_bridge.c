
#include "motion_feedback_bridge.h"

#include <stddef.h>
#include <stdint.h>

#include "encoder.h"
#include "wheel_feedback.h"
#include "wheel_speed_control.h"

/*
 * Maximum allowed interval with no encoder
 * movement while a motor is commanded to run.
 *
 * This is an initial SOFTWARE safeguard,
 * not a calibrated physical stall threshold.
 */
#define MOTION_NO_ENCODER_TIMEOUT_US 500000ULL

static bool motor_signs(
    motion_direction_t direction,
    int16_t *motor_1_sign,
    int16_t *motor_2_sign)
{
    switch (direction)
    {
        case MOTION_FORWARD:
            *motor_1_sign = 1;
            *motor_2_sign = -1;
            return true;

        case MOTION_BACKWARD:
            *motor_1_sign = -1;
            *motor_2_sign = 1;
            return true;

        case MOTION_LEFT:
            *motor_1_sign = -1;
            *motor_2_sign = -1;
            return true;

        case MOTION_RIGHT:
            *motor_1_sign = 1;
            *motor_2_sign = 1;
            return true;

        default:
            return false;
    }
}

bool motion_feedback_bridge_init(
    motion_feedback_bridge_t *bridge,
    wheel_feedback_pid_t *controller)
{
    if (bridge == NULL)
    {
        return false;
    }

    bridge->initialized = false;
    bridge->controller = NULL;
    bridge->last_left_pulse_us = 0ULL;
    bridge->last_right_pulse_us = 0ULL;

    if ((controller == NULL) ||
        (!controller->initialized) ||
        (controller->left == NULL) ||
        (controller->right == NULL))
    {
        return false;
    }

    if ((!controller->left->initialized) ||
        (!controller->right->initialized) ||
        (!controller->speed_controller.initialized))
    {
        return false;
    }

    bridge->controller = controller;
    bridge->initialized = true;

    return true;
}

bool motion_feedback_bridge_update(
    void *context,
    motion_direction_t direction,
    uint64_t current_time_us,
    bool starting,
    int16_t *motor_1_pwm,
    int16_t *motor_2_pwm)
{
    if ((motor_1_pwm == NULL) ||
        (motor_2_pwm == NULL))
    {
        return false;
    }

    *motor_1_pwm = 0;
    *motor_2_pwm = 0;

    motion_feedback_bridge_t *bridge =
        (motion_feedback_bridge_t *)context;

    if ((bridge == NULL) ||
        (!bridge->initialized) ||
        (bridge->controller == NULL))
    {
        return false;
    }

    wheel_feedback_pid_t *controller =
        bridge->controller;

    int16_t sign_1 = 0;
    int16_t sign_2 = 0;

    if (!motor_signs(
            direction, &sign_1, &sign_2))
    {
        return false;
    }

    if (starting)
    {
        /*
         * Rebase count sampling to the start
         * of the new motion without erasing
         * accumulated wheel distance.
         *
         * This is safe only when the encoder
         * and motion task share a synchronised
         * execution context.
         */
        controller->left->previous_count =
            encoder_get_count(
                &controller->left->encoder);

        controller->right->previous_count =
            encoder_get_count(
                &controller->right->encoder);

        controller->left->previous_sample_us =
            current_time_us;

        controller->right->previous_sample_us =
            current_time_us;

        controller->previous_update_us =
            current_time_us;

        wheel_feedback_pid_reset(controller);

        bridge->last_left_pulse_us =
            current_time_us;

        bridge->last_right_pulse_us =
            current_time_us;

        uint8_t base =
            controller->speed_controller.config
                .base_pwm_percent;

        *motor_1_pwm =
            sign_1 * (int16_t)base;

        *motor_2_pwm =
            sign_2 * (int16_t)base;

        return true;
    }

    wheel_feedback_pid_output_t output = {0};

    if (!wheel_feedback_pid_update(
            controller,
            direction,
            current_time_us,
            &output))
    {
        return false;
    }

    /*
     * The underlying wheel feedback module
     * records movement since its last sample.
     */
    if (controller->left->latest_result.distance_cm
        != 0.0f)
    {
        bridge->last_left_pulse_us =
            current_time_us;
    }

    if (controller->right->latest_result.distance_cm
        != 0.0f)
    {
        bridge->last_right_pulse_us =
            current_time_us;
    }

    if (((current_time_us -
          bridge->last_left_pulse_us) >=
         MOTION_NO_ENCODER_TIMEOUT_US) ||
        ((current_time_us -
          bridge->last_right_pulse_us) >=
         MOTION_NO_ENCODER_TIMEOUT_US))
    {
        return false;
    }

    *motor_1_pwm = output.motor_1_pwm;
    *motor_2_pwm = output.motor_2_pwm;

    return true;
}
