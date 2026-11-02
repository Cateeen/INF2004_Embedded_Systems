
#include "motion_control.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_driver.h"
#include "pico/time.h"

/*
 * Buddy 2 - Motion Control
 *
 * Vehicle direction and physical motor polarity:
 *
 * Forward:  M1 positive, M2 negative
 * Backward: M1 negative, M2 positive
 * Left:     M1 negative, M2 negative
 * Right:    M1 positive, M2 positive
 *
 * Feedback is optional. The controller must
 * be explicitly armed before motor operation.
 *
 * All functions must run in one designated
 * control-task context. This is not thread-safe.
 */

#define MOTION_MAX_DURATION_MS 10000U
#define MOTION_MAX_TURN_DEG 180.0f

#define MOTION_BRAKE_WAIT_US 60000ULL
#define MOTION_FEEDBACK_PERIOD_US 100000ULL
#define MOTION_UPDATE_TIMEOUT_US 250000ULL

static motion_config_t settings =
{
    0U,
    0U,
    1000U,
    1000U,
    20U
};

static bool initialized = false;
static bool armed = false;

static motion_status_t status = MOTION_IDLE;

static motion_direction_t direction =
    MOTION_FORWARD;

static uint32_t duration_ms = 0U;
static uint64_t waiting_since_us = 0ULL;
static uint64_t running_since_us = 0ULL;
static uint64_t last_update_us = 0ULL;
static uint64_t last_feedback_us = 0ULL;

static motion_feedback_callback_t
    feedback_callback = NULL;

static void *feedback_context = NULL;

static void enter_error(void)
{
    motor_emergency_stop();

    armed = false;
    duration_ms = 0U;
    status = MOTION_ERROR;
}

static bool calculate_duration(
    motion_direction_t requested_direction,
    float amount,
    uint32_t *result_ms)
{
    float milliseconds = 0.0f;

    if ((result_ms == NULL) ||
        !isfinite(amount) ||
        (amount <= 0.0f))
    {
        return false;
    }

    switch (requested_direction)
    {
        case MOTION_FORWARD:
            if (settings.forward_cm_per_second_x100 == 0U)
            {
                return false;
            }

            milliseconds =
                (amount * 100000.0f) /
                (float)settings.forward_cm_per_second_x100;
            break;

        case MOTION_BACKWARD:
            if (settings.backward_cm_per_second_x100 == 0U)
            {
                return false;
            }

            milliseconds =
                (amount * 100000.0f) /
                (float)settings.backward_cm_per_second_x100;
            break;

        case MOTION_LEFT:
            if (amount > MOTION_MAX_TURN_DEG)
            {
                return false;
            }

            milliseconds =
                (amount / 90.0f) *
                (float)settings.left_turn_ms_per_90;
            break;

        case MOTION_RIGHT:
            if (amount > MOTION_MAX_TURN_DEG)
            {
                return false;
            }

            milliseconds =
                (amount / 90.0f) *
                (float)settings.right_turn_ms_per_90;
            break;

        default:
            return false;
    }

    if (!isfinite(milliseconds) ||
        (milliseconds < 1.0f) ||
        (milliseconds > (float)MOTION_MAX_DURATION_MS))
    {
        return false;
    }

    *result_ms =
        (uint32_t)(milliseconds + 0.5f);

    return (*result_ms > 0U) &&
           (*result_ms <= MOTION_MAX_DURATION_MS);
}

static bool valid_pwm_for_direction(
    motion_direction_t requested_direction,
    int16_t motor_1,
    int16_t motor_2)
{
    if ((motor_1 < -100) || (motor_1 > 100) ||
        (motor_2 < -100) || (motor_2 > 100))
    {
        return false;
    }

    switch (requested_direction)
    {
        case MOTION_FORWARD:
            return (motor_1 >= 0) &&
                   (motor_2 <= 0);

        case MOTION_BACKWARD:
            return (motor_1 <= 0) &&
                   (motor_2 >= 0);

        case MOTION_LEFT:
            return (motor_1 <= 0) &&
                   (motor_2 <= 0);

        case MOTION_RIGHT:
            return (motor_1 >= 0) &&
                   (motor_2 >= 0);

        default:
            return false;
    }
}

static void apply_open_loop_pwm(void)
{
    int16_t pwm = (int16_t)settings.pwm_percent;

    switch (direction)
    {
        case MOTION_FORWARD:
            motor_set_speed(MOTOR_1, pwm);
            motor_set_speed(MOTOR_2, -pwm);
            break;

        case MOTION_BACKWARD:
            motor_set_speed(MOTOR_1, -pwm);
            motor_set_speed(MOTOR_2, pwm);
            break;

        case MOTION_LEFT:
            motor_set_speed(MOTOR_1, -pwm);
            motor_set_speed(MOTOR_2, -pwm);
            break;

        case MOTION_RIGHT:
            motor_set_speed(MOTOR_1, pwm);
            motor_set_speed(MOTOR_2, pwm);
            break;

        default:
            enter_error();
            break;
    }
}

static bool apply_feedback_pwm(
    uint64_t now_us,
    bool starting)
{
    int16_t motor_1 = 0;
    int16_t motor_2 = 0;

    if (feedback_callback == NULL)
    {
        return false;
    }

    if (!feedback_callback(
            feedback_context,
            direction,
            now_us,
            starting,
            &motor_1,
            &motor_2))
    {
        return false;
    }

    if (!valid_pwm_for_direction(
            direction, motor_1, motor_2))
    {
        return false;
    }

    if (motor_is_emergency_stopped())
    {
        return false;
    }

    motor_set_speed(MOTOR_1, motor_1);
    motor_set_speed(MOTOR_2, motor_2);

    return !motor_is_emergency_stopped();
}

void motion_init(void)
{
    motor_driver_init();
    motor_emergency_stop();

    settings.forward_cm_per_second_x100 = 0U;
    settings.backward_cm_per_second_x100 = 0U;
    settings.left_turn_ms_per_90 = 1000U;
    settings.right_turn_ms_per_90 = 1000U;
    settings.pwm_percent = 20U;

    feedback_callback = NULL;
    feedback_context = NULL;

    direction = MOTION_FORWARD;
    duration_ms = 0U;

    waiting_since_us = 0ULL;
    running_since_us = 0ULL;
    last_update_us = 0ULL;
    last_feedback_us = 0ULL;

    status = MOTION_IDLE;
    armed = false;
    initialized = true;
}

bool motion_arm(void)
{
    if (!initialized || motion_is_busy())
    {
        return false;
    }

    motor_clear_emergency_stop();

    if (motor_is_emergency_stopped())
    {
        armed = false;
        status = MOTION_ERROR;
        return false;
    }

    armed = true;
    status = MOTION_IDLE;
    duration_ms = 0U;

    return true;
}

bool motion_configure(
    const motion_config_t *config)
{
    if (!initialized ||
        (config == NULL) ||
        motion_is_busy())
    {
        return false;
    }

    if ((config->pwm_percent == 0U) ||
        (config->pwm_percent > 100U) ||
        (config->left_turn_ms_per_90 == 0U) ||
        (config->right_turn_ms_per_90 == 0U))
    {
        return false;
    }

    settings = *config;
    return true;
}

bool motion_set_feedback(
    motion_feedback_callback_t callback,
    void *context)
{
    if (!initialized ||
        motion_is_armed() ||
        motion_is_busy())
    {
        return false;
    }

    if (((callback == NULL) && (context != NULL)) ||
        ((callback != NULL) && (context == NULL)))
    {
        return false;
    }

    feedback_callback = callback;
    feedback_context = context;

    return true;
}

bool motion_start(
    motion_direction_t requested_direction,
    float amount)
{
    uint32_t requested_ms = 0U;

    if (!initialized ||
        !motion_is_armed() ||
        motion_is_busy())
    {
        return false;
    }

    if (!calculate_duration(
            requested_direction,
            amount,
            &requested_ms))
    {
        return false;
    }

    motor_brake_all();

    direction = requested_direction;
    duration_ms = requested_ms;
    waiting_since_us = time_us_64();
    running_since_us = 0ULL;

    status = MOTION_WAITING;
    return true;
}

void motion_update(void)
{
    if (!initialized)
    {
        return;
    }

    if (armed && motor_is_emergency_stopped())
    {
        enter_error();
        return;
    }

    if (!armed)
    {
        return;
    }

    uint64_t now_us = time_us_64();

    /*
     * A delayed feedback-control update is unsafe.
     *
     * This is a cooperative timeout check:
     * it is evaluated only when motion_update()
     * actually runs.
     */
    if ((feedback_callback != NULL) &&
        (status == MOTION_RUNNING) &&
        ((now_us - last_update_us) >
         MOTION_UPDATE_TIMEOUT_US))
    {
        enter_error();
        return;
    }

    last_update_us = now_us;

    if (status == MOTION_WAITING)
    {
        if ((now_us - waiting_since_us) <
            MOTION_BRAKE_WAIT_US)
        {
            return;
        }

        if (motor_is_emergency_stopped())
        {
            enter_error();
            return;
        }

        if (feedback_callback != NULL)
        {
            if (!apply_feedback_pwm(now_us, true))
            {
                enter_error();
                return;
            }

            last_feedback_us = now_us;
        }
        else
        {
            apply_open_loop_pwm();
        }

        if (status == MOTION_ERROR)
        {
            return;
        }

        running_since_us = now_us;
        status = MOTION_RUNNING;
    }

    if (status == MOTION_RUNNING)
    {
        uint64_t elapsed_us =
            now_us - running_since_us;

        /*
         * Stop before issuing any further PWM
         * when the requested duration has elapsed.
         */
        if (elapsed_us >=
            ((uint64_t)duration_ms * 1000ULL))
        {
            motor_brake_all();
            duration_ms = 0U;
            status = MOTION_COMPLETE;
            return;
        }

        if ((feedback_callback != NULL) &&
            ((now_us - last_feedback_us) >=
             MOTION_FEEDBACK_PERIOD_US))
        {
            if (!apply_feedback_pwm(now_us, false))
            {
                enter_error();
                return;
            }

            last_feedback_us = now_us;
        }
    }
}

void motion_stop(void)
{
    if (!initialized)
    {
        return;
    }

    motor_brake_all();
    duration_ms = 0U;
    status = MOTION_STOPPED;

    if (motor_is_emergency_stopped())
    {
        armed = false;
    }
}

void motion_emergency_stop(void)
{
    motor_emergency_stop();

    armed = false;
    duration_ms = 0U;
    status = MOTION_STOPPED;
}

motion_status_t motion_get_status(void)
{
    return status;
}

bool motion_is_busy(void)
{
    return (status == MOTION_WAITING) ||
           (status == MOTION_RUNNING);
}

bool motion_is_armed(void)
{
    return initialized &&
           armed &&
           !motor_is_emergency_stopped();
}

bool motion_move_forward(float distance_cm)
{
    return motion_start(
        MOTION_FORWARD, distance_cm);
}

bool motion_move_backward(float distance_cm)
{
    return motion_start(
        MOTION_BACKWARD, distance_cm);
}

bool motion_turn_left(float angle_degrees)
{
    return motion_start(
        MOTION_LEFT, angle_degrees);
}

bool motion_turn_right(float angle_degrees)
{
    return motion_start(
        MOTION_RIGHT, angle_degrees);
}
