#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef enum {
	MOTOR_1 = 0,
	MOTOR_2 = 1
} motor_id_t;

/* Call only from the motion task. Initialization leaves both motors braked
 * and emergency-stop latched; all driver calls require the initializing task. */
ER motor_driver_init(void);

/* Signed command from -100 to 100. Opposite direction is delayed by a brake
 * interval; repeat the command after E_BUSY. */
ER motor_set_speed(motor_id_t motor, int16_t speed_percent);

ER motor_brake_all(void);
ER motor_emergency_stop(void);
bool motor_is_emergency_stopped(void);

/* Clearing does not restart the motors; the motion task must issue a new
 * command after the controller authorizes re-arming. */
ER motor_clear_emergency_stop(void);

#endif
