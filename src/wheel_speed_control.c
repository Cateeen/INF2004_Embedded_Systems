
#include "wheel_speed_control.h"

#include <math.h>
#include <stddef.h>

/*
 * Buddy 2 - Dual Wheel PID Speed Controller
 *
 * Produces unsigned PWM magnitudes.
 * Motor directions and emergency stopping remain
 * the responsibility of the motion-control task.
 */

#define MAX_CONTROL_INTERVAL_SECONDS 0.25f

static bool valid_config(
    const wheel_speed_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if (!isfinite(config->kp) ||
        !isfinite(config->ki) ||
        !isfinite(config->kd) ||
        !isfinite(config->target_speed_cm_s))
    {
        return false;
    }

    if ((config->kp < 0.0f) ||
        (config->ki < 0.0f) ||
        (config->kd < 0.0f) ||
        (config->target_speed_cm_s <= 0.0f))
    {
        return false;
    }

    if ((config->base_pwm_percent == 0U) ||
        (config->max_pwm_percent == 0U) ||
        (config->max_pwm_percent > 100U) ||
        (config->base_pwm_percent >
         config->max_pwm_percent))
    {
        return false;
    }

    return true;
}

static uint8_t calculate_pwm(
    float base_pwm,
    float correction,
    uint8_t maximum)
{
    float command = base_pwm + correction;

    if (command < 0.0f)
    {
        command = 0.0f;
    }

    if (command > (float)maximum)
    {
        command = (float)maximum;
    }

    return (uint8_t)(command + 0.5f);
}

bool wheel_speed_control_init(
    wheel_speed_control_t *controller,
    const wheel_speed_config_t *config)
{
    if (controller == NULL)
    {
        return false;
    }

    controller->initialized = false;

    if (!valid_config(config))
    {
        return false;
    }

    controller->config = *config;

    /*
     * Limit PID correction so base + correction
     * always remains within the configured
     * motor PWM range.
     */
    float minimum_correction =
        -(float)config->base_pwm_percent;

    float maximum_correction =
        (float)config->max_pwm_percent -
        (float)config->base_pwm_percent;

    pid_init(
        &controller->left_pid,
        config->kp,
        config->ki,
        config->kd,
        minimum_correction,
        maximum_correction);

    pid_init(
        &controller->right_pid,
        config->kp,
        config->ki,
        config->kd,
        minimum_correction,
        maximum_correction);

    controller->initialized = true;

    return true;
}

bool wheel_speed_control_update(
    wheel_speed_control_t *controller,
    float left_speed_cm_s,
    float right_speed_cm_s,
    float dt_seconds,
    wheel_speed_output_t *output)
{
    if (output == NULL)
    {
        return false;
    }

    /* Safe result whenever an update fails. */
    output->left_pwm_percent = 0U;
    output->right_pwm_percent = 0U;

    if ((controller == NULL) ||
        (!controller->initialized))
    {
        return false;
    }

    if (!isfinite(left_speed_cm_s) ||
        !isfinite(right_speed_cm_s) ||
        !isfinite(dt_seconds))
    {
        return false;
    }

    if ((left_speed_cm_s < 0.0f) ||
        (right_speed_cm_s < 0.0f) ||
        (dt_seconds <= 0.0f) ||
        (dt_seconds > MAX_CONTROL_INTERVAL_SECONDS))
    {
        return false;
    }

    float target =
        controller->config.target_speed_cm_s;

    float left_correction = pid_update(
        &controller->left_pid,
        target,
        left_speed_cm_s,
        dt_seconds);

    float right_correction = pid_update(
        &controller->right_pid,
        target,
        right_speed_cm_s,
        dt_seconds);

    if (!isfinite(left_correction) ||
        !isfinite(right_correction))
    {
        wheel_speed_control_reset(controller);
        return false;
    }

    float base =
        (float)controller->config.base_pwm_percent;

    uint8_t maximum =
        controller->config.max_pwm_percent;

    output->left_pwm_percent = calculate_pwm(
        base, left_correction, maximum);

    output->right_pwm_percent = calculate_pwm(
        base, right_correction, maximum);

    return true;
}

void wheel_speed_control_reset(
    wheel_speed_control_t *controller)
{
    if ((controller == NULL) ||
        (!controller->initialized))
    {
        return;
    }

    pid_reset(&controller->left_pid);
    pid_reset(&controller->right_pid);
}
