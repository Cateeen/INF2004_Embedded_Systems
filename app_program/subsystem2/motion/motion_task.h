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
 * VELOCITY accepts straight motion in signed cm/s; angular velocity must be
 * zero. Its nonzero duration is measured from the message timestamp. Hardware
 * mode requires ARM while stopped before VELOCITY; STOP does not re-arm.
 * STOP takes precedence over movement in the same queue batch; EMERGENCY_STOP
 * is latched until restart. A changed speed or direction requires STOP first.
 * The controller must issue consecutive sequence numbers, supervise stale
 * data independently, and handle faults. */

#if APP_TEST
BOOL motion_task_test(void);
#endif

#endif
