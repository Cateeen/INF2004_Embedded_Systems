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

/* Junction turns. Values equal dashboard_turn_t, so the dashboard's
 * dashboard_next_junction() result can be passed straight through. */
#define VEHICLE_JUNCTION_STRAIGHT 0U
#define VEHICLE_JUNCTION_LEFT     2U
#define VEHICLE_JUNCTION_RIGHT    3U
#define VEHICLE_JUNCTION_UTURN    4U

/* Called when a junction is detected: the turn to take (a value above). */
typedef uint8_t (*vehicle_controller_junction_turn_t)(void *context);
/* Called when that turn has been completed (also for STRAIGHT). */
typedef void (*vehicle_controller_junction_done_t)(void *context, uint8_t turn);
/* Called for each valid decoded barcode (payload is not NUL-terminated). */
typedef void (*vehicle_controller_barcode_t)(void *context, const char *payload,
					     uint8_t length);
/* Called (with input_context) when the robot has stopped by itself while GO
 * is still on -- lost line, pick-up, tilt, an obstacle it will not bypass,
 * a bypass that could not resume. A toggle GO input should turn itself off
 * here, so the next single press is the fresh GO edge that re-arms. */
typedef void (*vehicle_controller_go_reset_t)(void *context);

/*
 * Junction manoeuvre, measured with SENSOR_TEST=junction: after the
 * junction is detected (line sensors just over the crossing) drive straight
 * FORWARD_MM (right turns FORWARD_RIGHT_MM: the tracked tape edge is off the
 * robot's centreline) so the wheels are over the crossing, stop, spin until
 * the line sensors find the new line, stop, and resume line following.
 * "Found" is: leave the line, then see the next one -- a left/right turn
 * stops at the first line, a U-turn passes the side arm and stops at the
 * second (the line it came on). Times are from the speed, since the
 * controller has no distance feedback.
 */
#define VEHICLE_JUNCTION_FORWARD_MM       100U
#define VEHICLE_JUNCTION_FORWARD_RIGHT_MM 120U
#define VEHICLE_JUNCTION_SPIN_CM_S        8      /* each wheel, opposite ways */
#define VEHICLE_JUNCTION_SETTLE_MS        400U   /* wheels stopped before/after the spin */
#define VEHICLE_JUNCTION_SPIN_MAX_MS      4000U  /* no line by then: stop */

typedef struct {
	car_safety_config_t safety;
	RELTIM period_ms;
	uint32_t motion_status_max_age_ms;
	vehicle_controller_read_inputs_t read_inputs;
	void *input_context;
	/* NULL: junctions are driven straight over. */
	vehicle_controller_junction_turn_t junction_turn;
	vehicle_controller_junction_done_t junction_done;  /* may be NULL */
	void *junction_context;
	/* Decoded barcodes are always drained (the line task faults when its
	 * 8-deep queue is full); NULL just discards them. */
	vehicle_controller_barcode_t barcode;
	void *barcode_context;
	/* true: when an obstacle stops the robot while following (ahead reading
	 * <= safety.obstacle_stop_mm), stop, then let the vehicle_bypass task
	 * scan it, drive round it and rejoin the line, and resume following
	 * without a GO press if every arming check passes. Needs
	 * vehicle_bypass_start(), and the obstacle task's emergency distance
	 * well below obstacle_stop_mm (the bypass works ~20-30 cm from it).
	 * false: stop and wait for GO, as before. */
	bool obstacle_bypass;
	/* May be NULL (then two presses of a toggle GO restart the robot). */
	vehicle_controller_go_reset_t go_reset;
} vehicle_controller_task_config_t;

/* Opt-in safety/arming task. The callback supplies trusted local GO and
 * emergency-stop inputs and must return promptly without network access.
 * The callback context must remain valid while the task runs. The task
 * exclusively consumes line, barcode, IMU, obstacle, fault and motion-status
 * buffers.
 * It sends ARM only after the safety gate passes and a fresh zero-output
 * motion status is received. While FOLLOWING it then sends one line-steering
 * VELOCITY command per cycle (speed and turn from line_steering, 200 ms
 * lifetime) and STOP when it leaves FOLLOWING. A junction observation runs
 * the junction manoeuvre above (VELOCITY, STOP, TURN, STOP) instead of the
 * steering until it completes. A motion status that reports
 * movement is accepted only while a VELOCITY or TURN command is live, and for one
 * status period after STOP; otherwise it is a fault. Motion
 * status is commanded state, not evidence that the vehicle physically stopped.
 * The task period must be no more than half the motion-status publish interval.
 * Do not start it alongside controller_monitor_start(). */
ER vehicle_controller_task_start(const vehicle_controller_task_config_t *config);

#if APP_TEST
BOOL vehicle_controller_task_test(void);
#endif

#endif
