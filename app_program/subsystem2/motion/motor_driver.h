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
 * and emergency-stop latched; all driver calls require the initializing task
 * (the owner), or the borrower while the motors are lent (below). */
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

/*
 * Lending: exactly one task writes the motors at any moment.
 * motor_driver_lend (owner only, motors not lent, emergency stop cleared)
 * brakes and hands every driver call to borrower; the owner's calls then get
 * E_CTX (set_speed) or E_OBJ (clear), and the borrower drives exactly as the
 * owner did. motor_driver_return (borrower only) brakes and gives them back.
 * Owner's motor_brake_all / motor_emergency_stop while lent REVOKE instead:
 * they return E_OK at once and the borrower brakes and gives the motors back
 * at its next driver call (which then fails with E_CTX); emergency also stays
 * latched. Lend through motion_task_borrow_motors(), never directly.
 */
ER motor_driver_lend(ID borrower);
ER motor_driver_return(void);
/* The calling task is the one allowed to drive right now. */
bool motor_driver_is_writer(void);
bool motor_driver_lent(void);
/* Signed output last applied to each motor (0 = braked), whoever drove it.
 * Any task; for status reporting only. */
void motor_driver_get_output(int16_t *motor_1, int16_t *motor_2);

#endif
