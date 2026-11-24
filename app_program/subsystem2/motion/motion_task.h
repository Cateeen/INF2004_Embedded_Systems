#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <tk/tkernel.h>
#include "motion_control.h"

typedef struct {
	motion_control_config_t control;
	RELTIM period_ms;
	uint32_t command_max_age_ms;
	uint16_t max_duration_ms;
	uint16_t max_linear_speed_cm_s;
} motion_task_config_t;

/* Simulation entry point: never accesses motors or publishes motion status. */
ER motion_task_start(const motion_task_config_t *config);

/* Hardware entry point: explicitly initializes the motor driver in its
 * braked/emergency-latched state. The controller must send a fresh ARM command
 * after its safety checks, followed by a fresh VELOCITY command. This is not
 * called by application startup. */
ER motion_task_start_hardware(const motion_task_config_t *config);

/* Both modes consume the dedicated motion and odometry buffers and neither
 * publishes motion status. The hardware mode uses the original motor polarity
 * mapping and requires independent freshness supervision; no physical stop is
 * guaranteed if this task stalls.
 *
 * VELOCITY carries signed linear_velocity v (cm/s, + forward) and a steering
 * term angular_velocity w (cm/s at each wheel, + turns left/counter-clockwise):
 * the wheel speed targets are left = v - w and right = v + w. |w| must be less
 * than |v| (both wheels keep the same direction; no spins) and |v| + |w| must
 * not exceed max_linear_speed_cm_s. Its nonzero duration is measured from the
 * message timestamp. Hardware mode requires ARM while stopped before
 * VELOCITY; STOP does not re-arm. STOP takes precedence over movement in the
 * same queue batch; EMERGENCY_STOP is latched until restart. While moving, a
 * new VELOCITY in the same direction updates speed and steering without
 * resetting the wheel PIDs; reversing direction requires STOP first.
 * TURN spins on the spot: linear_velocity 0 and angular_velocity w = each
 * wheel's speed in cm/s (+ = left: left wheel backward, right forward), |w|
 * up to max_linear_speed_cm_s, with VELOCITY's duration and arming rules.
 * A new TURN in the same direction refreshes it; switching between VELOCITY
 * and TURN, or the spin direction, requires STOP first (and the wheels to
 * have stopped, or the feedback direction check faults).
 * The controller must issue consecutive sequence numbers, supervise stale
 * data independently, and handle faults. */

/*
 * Motor lending, for a task that must drive the motors directly (the
 * obstacle bypass). Called BY THE BORROWER: asks this task, which owns the
 * motor driver, to lend it; this task lends only while it is stopped (no
 * live VELOCITY/TURN) and armed, and does not touch the motors meanwhile.
 * Waits up to timeout_ms (or until abort() returns true; abort may be NULL)
 * and returns true once the caller may call motor_set_speed etc. The caller
 * must then call motion_task_return_motors() when done, before anything
 * sends this task ARM/VELOCITY/TURN again (ARM while lent is a motion fault).
 * A STOP or EMERGENCY_STOP received while lent revokes the motors (see
 * motor_driver_lend). Hardware mode only; one borrower at a time.
 */
bool motion_task_borrow_motors(bool (*abort)(void *context), void *context,
			       uint32_t timeout_ms);
void motion_task_return_motors(void);

#if APP_TEST
BOOL motion_task_test(void);
#endif

#endif
