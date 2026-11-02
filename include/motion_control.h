
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_WAITING,
    MOTION_RUNNING,
    MOTION_COMPLETE,
    MOTION_STOPPED,
    MOTION_ERROR
} motion_status_t;

typedef enum
{
    MOTION_FORWARD = 0,
    MOTION_BACKWARD,
    MOTION_LEFT,
    MOTION_RIGHT
} motion_direction_t;

typedef struct
{
    uint16_t forward_cm_per_second_x100;
    uint16_t backward_cm_per_second_x100;
    uint16_t left_turn_ms_per_90;
    uint16_t right_turn_ms_per_90;
    uint8_t pwm_percent;
} motion_config_t;

/*
 * Called by the motion controller.
 *
 * starting=true:
 *     Prepare feedback for a new movement.
 *
 * starting=false:
 *     Calculate updated motor PWM commands.
 *
 * Return false if feedback is invalid or unsafe.
 * The motion controller will emergency-stop.
 *
 * All motor PWM commands must be in [-100, 100].
 */
typedef bool (*motion_feedback_callback_t)(
    void *context,
    motion_direction_t direction,
    uint64_t current_time_us,
    bool starting,
    int16_t *motor_1_pwm,
    int16_t *motor_2_pwm);

void motion_init(void);

bool motion_arm(void);

bool motion_configure(
    const motion_config_t *config);

/*
 * Configure optional closed-loop feedback.
 *
 * Must be called while disarmed and not moving.
 * Pass NULL for both arguments to disable feedback.
 *
 * This function does not arm the motors.
 */
bool motion_set_feedback(
    motion_feedback_callback_t callback,
    void *context);

bool motion_start(
    motion_direction_t direction,
    float amount);

/*
 * Call periodically from one designated task.
 *
 * Recommended interval: 10 ms.
 * Feedback sampling interval: 100 ms.
 */
void motion_update(void);

void motion_stop(void);

void motion_emergency_stop(void);

motion_status_t motion_get_status(void);

bool motion_is_busy(void);

bool motion_is_armed(void);

bool motion_move_forward(float distance_cm);

bool motion_move_backward(float distance_cm);

bool motion_turn_left(float angle_degrees);

bool motion_turn_right(float angle_degrees);

#endif
