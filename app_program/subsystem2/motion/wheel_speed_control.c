
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

    if (!isfinite(config->ff_pwm_per_cm_s) ||
        !isfinite(config->ff_ref_speed_cm_s) ||
        (config->ff_pwm_per_cm_s < 0.0f) ||
        (config->ff_ref_speed_cm_s < 0.0f) ||
        ((config->ff_pwm_per_cm_s > 0.0f) &&
         (config->ff_ref_speed_cm_s <= 0.0f)))
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

/*
 * PWM a wheel starts from for its target: base at the reference speed,
 * plus the feed-forward slope for the difference. With a zero slope this
 * is exactly base_pwm_percent, as before feed-forward existed.
 */
static float feed_forward_pwm(
    const wheel_speed_config_t *config,
    float target_cm_s)
{
    float pwm = (float)config->base_pwm_percent;

    if (config->ff_pwm_per_cm_s > 0.0f)
    {
        pwm += config->ff_pwm_per_cm_s *
               (target_cm_s - config->ff_ref_speed_cm_s);
    }

    if (pwm < 0.0f)
    {
        pwm = 0.0f;
    }

    if (pwm > (float)config->max_pwm_percent)
    {
        pwm = (float)config->max_pwm_percent;
    }

    return pwm;
}

/*
 * Keep feed-forward + correction inside 0..max, so the PID's conditional
 * integration stops winding up exactly where the motor saturates.
 */
static void set_correction_limits(
    pid_controller_t *pid,
    float feed_forward,
    uint8_t maximum)
{
    pid->output_min = -feed_forward;
    pid->output_max = (float)maximum - feed_forward;
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

    if (!pid_init(
        &controller->left_pid,
        config->kp,
        config->ki,
        config->kd,
        minimum_correction,
        maximum_correction))
    {
        return false;
    }

    if (!pid_init(
        &controller->right_pid,
        config->kp,
        config->ki,
        config->kd,
        minimum_correction,
        maximum_correction))
    {
        return false;
    }

    controller->left_target_cm_s = config->target_speed_cm_s;
    controller->right_target_cm_s = config->target_speed_cm_s;
    controller->initialized = true;

    return true;
}

bool wheel_speed_control_set_targets(
    wheel_speed_control_t *controller,
    float left_target_cm_s,
    float right_target_cm_s)
{
    if ((controller == NULL) ||
        (!controller->initialized) ||
        !isfinite(left_target_cm_s) ||
        !isfinite(right_target_cm_s) ||
        (left_target_cm_s <= 0.0f) ||
        (right_target_cm_s <= 0.0f))
    {
        return false;
    }

    controller->left_target_cm_s = left_target_cm_s;
    controller->right_target_cm_s = right_target_cm_s;

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
        if (controller != NULL && controller->initialized)
        {
            (void)wheel_speed_control_reset(controller);
        }
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
        (void)wheel_speed_control_reset(controller);
        return false;
    }

    if ((left_speed_cm_s < 0.0f) ||
        (right_speed_cm_s < 0.0f) ||
        (dt_seconds <= 0.0f) ||
        (dt_seconds > MAX_CONTROL_INTERVAL_SECONDS))
    {
        (void)wheel_speed_control_reset(controller);
        return false;
    }

    float left_correction;
    float right_correction;

    uint8_t maximum =
        controller->config.max_pwm_percent;

    float left_base = feed_forward_pwm(
        &controller->config, controller->left_target_cm_s);

    float right_base = feed_forward_pwm(
        &controller->config, controller->right_target_cm_s);

    set_correction_limits(&controller->left_pid, left_base, maximum);
    set_correction_limits(&controller->right_pid, right_base, maximum);

    if (!pid_update(
        &controller->left_pid,
        controller->left_target_cm_s,
        left_speed_cm_s,
        dt_seconds,
        &left_correction) ||
        !pid_update(
        &controller->right_pid,
        controller->right_target_cm_s,
        right_speed_cm_s,
        dt_seconds,
        &right_correction))
    {
        (void)wheel_speed_control_reset(controller);
        return false;
    }

    output->left_pwm_percent = calculate_pwm(
        left_base, left_correction, maximum);

    output->right_pwm_percent = calculate_pwm(
        right_base, right_correction, maximum);

    return true;
}

/*
 * PWM for the current targets from the last measured speeds, without
 * advancing the PIDs (no integration, derivative or stored-error change):
 * feed-forward + kp * error + ki * integral, clamped as in an update.
 */
bool wheel_speed_control_retarget_output(
    const wheel_speed_control_t *controller,
    float left_speed_cm_s,
    float right_speed_cm_s,
    wheel_speed_output_t *output)
{
    if ((controller == NULL) ||
        (!controller->initialized) ||
        (output == NULL) ||
        !isfinite(left_speed_cm_s) ||
        !isfinite(right_speed_cm_s) ||
        (left_speed_cm_s < 0.0f) ||
        (right_speed_cm_s < 0.0f))
    {
        return false;
    }

    uint8_t maximum = controller->config.max_pwm_percent;
    float left_base = feed_forward_pwm(
        &controller->config, controller->left_target_cm_s);
    float right_base = feed_forward_pwm(
        &controller->config, controller->right_target_cm_s);
    float left_correction =
        controller->left_pid.kp *
            (controller->left_target_cm_s - left_speed_cm_s) +
        controller->left_pid.ki * controller->left_pid.integral;
    float right_correction =
        controller->right_pid.kp *
            (controller->right_target_cm_s - right_speed_cm_s) +
        controller->right_pid.ki * controller->right_pid.integral;

    if (!isfinite(left_correction) || !isfinite(right_correction))
    {
        return false;
    }

    /* calculate_pwm clamps base + correction to 0..max, which is the
     * same as the update's correction limits. */
    output->left_pwm_percent = calculate_pwm(
        left_base, left_correction, maximum);
    output->right_pwm_percent = calculate_pwm(
        right_base, right_correction, maximum);
    return true;
}

bool wheel_speed_control_reset(
    wheel_speed_control_t *controller)
{
    if ((controller == NULL) ||
        (!controller->initialized))
    {
        return false;
    }

    return pid_reset(&controller->left_pid) &&
           pid_reset(&controller->right_pid);
}
