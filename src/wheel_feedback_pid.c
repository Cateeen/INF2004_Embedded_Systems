
#include "wheel_feedback_pid.h"

#include <math.h>
#include <stddef.h>

#define MAX_FEEDBACK_INTERVAL_US 250000ULL

static bool direction_signs(
    motion_direction_t direction,
    int8_t *left_wheel_sign,
    int8_t *right_wheel_sign,
    int8_t *motor_1_sign,
    int8_t *motor_2_sign)
{
    switch (direction)
    {
        case MOTION_FORWARD:
            *left_wheel_sign = 1;
            *right_wheel_sign = 1;
            *motor_1_sign = 1;
            *motor_2_sign = -1;
            return true;

        case MOTION_BACKWARD:
            *left_wheel_sign = -1;
            *right_wheel_sign = -1;
            *motor_1_sign = -1;
            *motor_2_sign = 1;
            return true;

        case MOTION_LEFT:
            *left_wheel_sign = -1;
            *right_wheel_sign = 1;
            *motor_1_sign = -1;
            *motor_2_sign = -1;
            return true;

        case MOTION_RIGHT:
            *left_wheel_sign = 1;
            *right_wheel_sign = -1;
            *motor_1_sign = 1;
            *motor_2_sign = 1;
            return true;

        default:
            return false;
    }
}

bool wheel_feedback_pid_init(
    wheel_feedback_pid_t *controller,
    wheel_feedback_t *left,
    wheel_feedback_t *right,
    const wheel_speed_config_t *speed_config,
    bool left_positive_forward,
    bool right_positive_forward,
    uint64_t current_time_us)
{
    if (controller == NULL)
    {
        return false;
    }

    controller->initialized = false;

    if ((left == NULL) ||
        (right == NULL) ||
        (left == right) ||
        (!left->initialized) ||
        (!right->initialized))
    {
        return false;
    }

    if (!wheel_speed_control_init(
            &controller->speed_controller,
            speed_config))
    {
        return false;
    }

    controller->left = left;
    controller->right = right;

    controller->left_positive_forward =
        left_positive_forward;

    controller->right_positive_forward =
        right_positive_forward;

    controller->previous_update_us =
        current_time_us;

    controller->initialized = true;

    return true;
}

bool wheel_feedback_pid_update(
    wheel_feedback_pid_t *controller,
    motion_direction_t direction,
    uint64_t current_time_us,
    wheel_feedback_pid_output_t *output)
{
    if (output == NULL)
    {
        return false;
    }

    output->motor_1_pwm = 0;
    output->motor_2_pwm = 0;

    if ((controller == NULL) ||
        (!controller->initialized))
    {
        return false;
    }

    int8_t left_wheel_sign = 0;
    int8_t right_wheel_sign = 0;
    int8_t motor_1_sign = 0;
    int8_t motor_2_sign = 0;

    if (!direction_signs(
            direction,
            &left_wheel_sign,
            &right_wheel_sign,
            &motor_1_sign,
            &motor_2_sign))
    {
        return false;
    }

    if (current_time_us <=
        controller->previous_update_us)
    {
        return false;
    }

    uint64_t elapsed_us =
        current_time_us -
        controller->previous_update_us;

    if (elapsed_us > MAX_FEEDBACK_INTERVAL_US)
    {
        return false;
    }

    /*
     * Sample both encoders.
     * A failed sample means motor output must be
     * stopped by the calling motion task.
     */
    if (!wheel_feedback_sample(
            controller->left,
            current_time_us))
    {
        return false;
    }

    if (!wheel_feedback_sample(
            controller->right,
            current_time_us))
    {
        return false;
    }

    wheel_odometry_result_t left_result = {0};
    wheel_odometry_result_t right_result = {0};

    if (!wheel_feedback_get_result(
            controller->left, &left_result) ||
        !wheel_feedback_get_result(
            controller->right, &right_result))
    {
        return false;
    }

    int left_encoder_sign =
        controller->left_positive_forward ? 1 : -1;

    int right_encoder_sign =
        controller->right_positive_forward ? 1 : -1;

    float left_speed =
        left_result.speed_cm_per_second *
        (float)left_encoder_sign *
        (float)left_wheel_sign;

    float right_speed =
        right_result.speed_cm_per_second *
        (float)right_encoder_sign *
        (float)right_wheel_sign;

    /*
     * A speed in the opposite direction may
     * indicate incorrect polarity or unexpected
     * wheel motion.
     */
    if (!isfinite(left_speed) ||
        !isfinite(right_speed) ||
        (left_speed < 0.0f) ||
        (right_speed < 0.0f))
    {
        return false;
    }

    float dt_seconds =
        (float)elapsed_us / 1000000.0f;

    wheel_speed_output_t pwm = {0};

    if (!wheel_speed_control_update(
            &controller->speed_controller,
            left_speed,
            right_speed,
            dt_seconds,
            &pwm))
    {
        return false;
    }

    output->motor_1_pwm =
        (int16_t)(
            motor_1_sign *
            (int16_t)pwm.left_pwm_percent);

    output->motor_2_pwm =
        (int16_t)(
            motor_2_sign *
            (int16_t)pwm.right_pwm_percent);

    controller->previous_update_us =
        current_time_us;

    return true;
}

void wheel_feedback_pid_reset(
    wheel_feedback_pid_t *controller)
{
    if ((controller == NULL) ||
        (!controller->initialized))
    {
        return;
    }

    wheel_speed_control_reset(
        &controller->speed_controller);
}
