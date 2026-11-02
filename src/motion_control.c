
#include "motion_control.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/time.h"
#include "motor_driver.h"

/*
 * Buddy 2 - Non-Blocking Motion Control
 *
 * Physical motor orientation:
 *
 * Forward:
 *   Motor 1 = positive
 *   Motor 2 = negative
 *
 * Backward:
 *   Motor 1 = negative
 *   Motor 2 = positive
 *
 * Left turn:
 *   Both motors = negative
 *
 * Right turn:
 *   Both motors = positive
 *
 * This implementation uses open-loop timing.
 * Actual movement accuracy requires hardware
 * calibration and encoder/IMU feedback.
 */

#define MOTION_MAX_DURATION_MS 10000U
#define MOTION_MAX_TURN_DEG 180.0f
#define MOTION_BRAKE_WAIT_US 60000ULL

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

static motion_status_t current_status = MOTION_IDLE;

static motion_direction_t current_direction =
    MOTION_FORWARD;

static uint32_t requested_duration_ms = 0U;

static uint64_t wait_started_us = 0ULL;
static uint64_t movement_started_us = 0ULL;

/*
 * Convert requested distance or angle
 * into a movement duration in milliseconds.
 */
static bool calculate_duration(
    motion_direction_t direction,
    float amount,
    uint32_t *duration_ms)
{
    float milliseconds = 0.0f;

    if ((duration_ms == NULL) ||
        (!isfinite(amount)) ||
        (amount <= 0.0f))
    {
        return false;
    }

    switch (direction)
    {
        case MOTION_FORWARD:
        {
            if (settings.forward_cm_per_second_x100 == 0U)
            {
                return false;
            }

            milliseconds =
                (amount * 100000.0f) /
                (float)settings.forward_cm_per_second_x100;

            break;
        }

        case MOTION_BACKWARD:
        {
            if (settings.backward_cm_per_second_x100 == 0U)
            {
                return false;
            }

            milliseconds =
                (amount * 100000.0f) /
                (float)settings.backward_cm_per_second_x100;

            break;
        }

        case MOTION_LEFT:
        {
            if (amount > MOTION_MAX_TURN_DEG)
            {
                return false;
            }

            milliseconds =
                (amount / 90.0f) *
                (float)settings.left_turn_ms_per_90;

            break;
        }

        case MOTION_RIGHT:
        {
            if (amount > MOTION_MAX_TURN_DEG)
            {
                return false;
            }

            milliseconds =
                (amount / 90.0f) *
                (float)settings.right_turn_ms_per_90;

            break;
        }

        default:
        {
            return false;
        }
    }

    if ((!isfinite(milliseconds)) ||
        (milliseconds < 1.0f) ||
        (milliseconds >
         (float)MOTION_MAX_DURATION_MS))
    {
        return false;
    }

    *duration_ms =
        (uint32_t)(milliseconds + 0.5f);

    return (*duration_ms > 0U) &&
           (*duration_ms <= MOTION_MAX_DURATION_MS);
}

/*
 * Apply the motor commands required for
 * the current vehicle movement direction.
 */
static void apply_motor_direction(void)
{
    int16_t speed =
        (int16_t)settings.pwm_percent;

    switch (current_direction)
    {
        case MOTION_FORWARD:
        {
            motor_set_speed(MOTOR_1, speed);
            motor_set_speed(MOTOR_2, -speed);
            break;
        }

        case MOTION_BACKWARD:
        {
            motor_set_speed(MOTOR_1, -speed);
            motor_set_speed(MOTOR_2, speed);
            break;
        }

        case MOTION_LEFT:
        {
            motor_set_speed(MOTOR_1, -speed);
            motor_set_speed(MOTOR_2, -speed);
            break;
        }

        case MOTION_RIGHT:
        {
            motor_set_speed(MOTOR_1, speed);
            motor_set_speed(MOTOR_2, speed);
            break;
        }

        default:
        {
            motion_emergency_stop();
            current_status = MOTION_ERROR;
            break;
        }
    }
}

/*
 * Initialise the controller.
 *
 * Motor output remains disabled.
 */
void motion_init(void)
{
    motor_driver_init();
    motor_emergency_stop();

    settings.forward_cm_per_second_x100 = 0U;
    settings.backward_cm_per_second_x100 = 0U;
    settings.left_turn_ms_per_90 = 1000U;
    settings.right_turn_ms_per_90 = 1000U;
    settings.pwm_percent = 20U;

    requested_duration_ms = 0U;
    wait_started_us = 0ULL;
    movement_started_us = 0ULL;

    current_direction = MOTION_FORWARD;
    current_status = MOTION_IDLE;

    armed = false;
    initialized = true;
}

/*
 * Explicitly arm the motion controller.
 *
 * The caller must be an authorised
 * safety-control task that has verified
 * it is safe to enable movement.
 */
bool motion_arm(void)
{
    if ((!initialized) || motion_is_busy())
    {
        return false;
    }

    /*
     * Clearing the motor emergency stop is
     * permitted only through this explicit API.
     */
    motor_clear_emergency_stop();

    if (motor_is_emergency_stopped())
    {
        armed = false;
        current_status = MOTION_ERROR;
        return false;
    }

    armed = true;
    current_status = MOTION_IDLE;

    requested_duration_ms = 0U;
    wait_started_us = 0ULL;
    movement_started_us = 0ULL;

    return true;
}

/*
 * Configure calibrated movement parameters.
 *
 * The controller must not be moving.
 */
bool motion_configure(const motion_config_t *config)
{
    if ((!initialized) ||
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

/*
 * Start a non-blocking movement.
 *
 * Requires the motion controller to be armed
 * and the motor emergency stop to be clear.
 */
bool motion_start(
    motion_direction_t direction,
    float amount)
{
    uint32_t duration_ms = 0U;

    if ((!initialized) ||
        (!motion_is_armed()) ||
        motion_is_busy())
    {
        return false;
    }

    if (!calculate_duration(
            direction,
            amount,
            &duration_ms))
    {
        return false;
    }

    /*
     * Normal braking before a new motion.
     *
     * Do not activate or clear the
     * emergency-stop latch here.
     */
    motor_brake_all();

    current_direction = direction;
    requested_duration_ms = duration_ms;

    wait_started_us = time_us_64();
    movement_started_us = 0ULL;

    current_status = MOTION_WAITING;

    return true;
}

/*
 * Periodic, non-blocking update.
 *
 * Call from one designated control task.
 *
 * An emergency stop is NEVER cleared here.
 */
void motion_update(void)
{
    uint64_t now_us;

    if (!initialized)
    {
        return;
    }

    /*
     * Detect an emergency stop activated
     * externally by the safety system.
     */
    if (armed && motor_is_emergency_stopped())
    {
        armed = false;
        requested_duration_ms = 0U;

        current_status = MOTION_ERROR;
        return;
    }

    if (!armed)
    {
        return;
    }

    now_us = time_us_64();

    if (current_status == MOTION_WAITING)
    {
        if ((now_us - wait_started_us) <
            MOTION_BRAKE_WAIT_US)
        {
            return;
        }

        /*
         * Check the emergency-stop latch
         * immediately before motor commands.
         */
        if (motor_is_emergency_stopped())
        {
            armed = false;
            current_status = MOTION_ERROR;
            return;
        }

        apply_motor_direction();

        if (current_status == MOTION_ERROR)
        {
            return;
        }

        movement_started_us = time_us_64();
        current_status = MOTION_RUNNING;
    }

    if (current_status == MOTION_RUNNING)
    {
        uint64_t elapsed_us =
            time_us_64() - movement_started_us;

        if (elapsed_us >=
            ((uint64_t)requested_duration_ms *
             1000ULL))
        {
            /*
             * Normal completion.
             * Brake without latching
             * the emergency stop.
             */
            motor_brake_all();

            current_status = MOTION_COMPLETE;
            requested_duration_ms = 0U;
        }
    }
}

/*
 * Normal movement stop.
 *
 * Leave the emergency-stop latch unchanged.
 */
void motion_stop(void)
{
    if (!initialized)
    {
        return;
    }

    motor_brake_all();

    requested_duration_ms = 0U;
    current_status = MOTION_STOPPED;

    if (motor_is_emergency_stopped())
    {
        armed = false;
    }
}

/*
 * Emergency stop.
 *
 * Further movement is forbidden until
 * an authorised caller invokes motion_arm().
 */
void motion_emergency_stop(void)
{
    motor_emergency_stop();

    armed = false;
    requested_duration_ms = 0U;

    current_status = MOTION_STOPPED;
}

/* Read motion state. */
motion_status_t motion_get_status(void)
{
    return current_status;
}

/* Read whether movement is in progress. */
bool motion_is_busy(void)
{
    return (current_status == MOTION_WAITING) ||
           (current_status == MOTION_RUNNING);
}

/*
 * Read whether the controller is armed.
 *
 * Also check the underlying motor
 * emergency-stop state.
 */
bool motion_is_armed(void)
{
    return initialized &&
           armed &&
           (!motor_is_emergency_stopped());
}

/* Convenience APIs. */

bool motion_move_forward(float distance_cm)
{
    return motion_start(
        MOTION_FORWARD,
        distance_cm);
}

bool motion_move_backward(float distance_cm)
{
    return motion_start(
        MOTION_BACKWARD,
        distance_cm);
}

bool motion_turn_left(float angle_degrees)
{
    return motion_start(
        MOTION_LEFT,
        angle_degrees);
}

bool motion_turn_right(float angle_degrees)
{
    return motion_start(
        MOTION_RIGHT,
        angle_degrees);
}
