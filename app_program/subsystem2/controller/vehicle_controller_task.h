#ifndef VEHICLE_CONTROLLER_TASK_H
#define VEHICLE_CONTROLLER_TASK_H

#include <tk/tkernel.h>
#include "controller_state.h"

typedef struct {
	bool go;
	bool emergency_stopped;
} vehicle_controller_inputs_t;

typedef bool (*vehicle_controller_read_inputs_t)(
	void *context, vehicle_controller_inputs_t *inputs);

typedef struct {
	car_safety_config_t safety;
	RELTIM period_ms;
	uint32_t motion_status_max_age_ms;
	vehicle_controller_read_inputs_t read_inputs;
	void *input_context;
} vehicle_controller_task_config_t;

/* Opt-in safety/arming task. The callback supplies trusted local GO and
 * emergency-stop inputs and must return promptly without network access.
 * The callback context must remain valid while the task runs. The task
 * exclusively consumes line, IMU, obstacle, fault and motion-status buffers.
 * It sends ARM only after the safety gate passes and a fresh zero-output
 * motion status is received; it does not yet issue motion commands. Motion
 * status is commanded state, not evidence that the vehicle physically stopped.
 * The task period must be no more than half the motion-status publish interval.
 * Do not start it alongside controller_monitor_start(). */
ER vehicle_controller_task_start(const vehicle_controller_task_config_t *config);

#if APP_TEST
BOOL vehicle_controller_task_test(void);
#endif

#endif
