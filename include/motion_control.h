
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Buddy 2 - Motion Control
 *
 * Non-blocking, time-based motion controller.
 *
 * Call motion_update() periodically from one
 * designated motion-control task.
 *
 * Movement distances and angles are open-loop
 * estimates until encoder/IMU feedback is added.
 */

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
 * Initialise motion control and motor hardware.
 *
 * Motors remain emergency-stopped after startup.
 * motion_arm() must be called explicitly before
 * accepting movement requests.
 */
void motion_init(void);

/*
 * Explicitly arm motion control.
 *
 * Only the authorised safety-control task should
 * call this after confirming the robot is safe.
 *
 * Returns false if uninitialised or moving.
 */
bool motion_arm(void);

/*
 * Configure motion calibration parameters.
 *
 * Only permitted when no movement is active.
 */
bool motion_configure(const motion_config_t *config);

/*
 * Request a non-blocking movement.
 *
 * Forward/backward: amount in centimetres.
 * Left/right: amount in degrees.
 *
 * Requires successful motion_arm().
 */
bool motion_start(motion_direction_t direction,
                  float amount);

/*
 * Update motion state and motor commands.
 *
 * Recommended update interval: 10 ms.
 *
 * Must never clear a latched emergency stop.
 */
void motion_update(void);

/*
 * Normal stop.
 *
 * Cancels current movement and brakes both
 * motors without clearing or changing an
 * existing emergency-stop latch.
 */
void motion_stop(void);

/*
 * Emergency stop.
 *
 * Immediately brakes both motors and disables
 * new movement requests until explicitly armed
 * by the authorised safety-control task.
 */
void motion_emergency_stop(void);

/* Read the current motion state. */
motion_status_t motion_get_status(void);

/* True if a movement is waiting or running. */
bool motion_is_busy(void);

/* True only when motion control is armed. */
bool motion_is_armed(void);

/* Convenience movement requests. */
bool motion_move_forward(float distance_cm);
bool motion_move_backward(float distance_cm);
bool motion_turn_left(float angle_degrees);
bool motion_turn_right(float angle_degrees);

#endif
