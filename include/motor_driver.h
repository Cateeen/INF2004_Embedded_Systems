#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_1 = 0,
    MOTOR_2 = 1
} motor_id_t;

/*
 * Signed motor command:
 * -100 to -1 = reverse
 * 0 = brake
 * 1 to 100 = forward
 *
 * Motor directions depend on physical wiring.
 */

/* Initialize motor PWM hardware in a stopped state. */
void motor_driver_init(void);

/*
 * Command motor speed.
 *
 * The caller must regularly issue commands from the
 * motion-control task.
 *
 * A direction reversal first applies the brakes.
 * A reversal command must be repeated after the
 * braking interval before reverse power is applied.
 */
void motor_set_speed(motor_id_t motor,
                     int16_t speed_percent);

/* Brake both motors without latching emergency stop. */
void motor_brake_all(void);

/* Brake both motors and reject further speed commands. */
void motor_emergency_stop(void);

/* Return whether emergency stop is active. */
bool motor_is_emergency_stopped(void);

/*
 * Only call after the Safety Monitor confirms
 * that the emergency-stop condition is cleared.
 *
 * Clearing does not automatically restart motors.
 */
void motor_clear_emergency_stop(void);

#endif