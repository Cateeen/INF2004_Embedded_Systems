
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
 */
void motor_driver_init(void);

void motor_set_speed(motor_id_t motor, int16_t speed_percent);

void motor_brake_all(void);

void motor_emergency_stop(void);

bool motor_is_emergency_stopped(void);

/*
 * Only call after the Safety Monitor confirms
 * that the emergency-stop condition is cleared.
 */
void motor_clear_emergency_stop(void);

#endif
