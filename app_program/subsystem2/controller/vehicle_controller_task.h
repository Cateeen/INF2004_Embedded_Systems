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
/* Called (with input_context) on a GO press while FAULTED. Faults latch and
 * the failed subsystem task has stopped, so recovery is a full restart:
 * this should reboot the system (it need not return). */
typedef void (*vehicle_controller_restart_t)(void *context);

/*
 * Straddle line following (straddle_steering.h): the two line sensors sit
 * further apart than the tape, which normally runs in the gap between them.
 * Tuned with SENSOR_TEST=line_follow LF_STRADDLE=1 at 10 cm/s (LF_SL_TURN=2,
 * LF_SL_OVERSHOOT_MS=150, LF_SL_BLIND_MS=1000, LF_SL_PIVOT_MAX_MS=4000,
 * LF_SL_COUNTER_PCT=50 / 300 ms). The pivot is 5/4 cm/s (wheels 1 and 9):
 * the motion task refuses |turn| >= speed, so the test's 4/4 (inner wheel
 * stopped) is not possible; the same 8 cm/s wheel difference turns at the
 * same rate.
 */
#define VEHICLE_LINE_SPEED_CM_S       10
#define VEHICLE_LINE_TURN_CM_S        2
/* One sensor on the tape this long = past a bend: pivot. 250 ms recovered
 * too late on the curves (bench line_follow, LF_SL_OVERSHOOT_MS=150). */
#define VEHICLE_LINE_OVERSHOOT_MS     150U
#define VEHICLE_LINE_PIVOT_SPEED_CM_S 5
#define VEHICLE_LINE_PIVOT_TURN_CM_S  4
#define VEHICLE_LINE_BLIND_MS         1000U   /* 0 = no blind search */
#define VEHICLE_LINE_PIVOT_MAX_MS     4000U
#define VEHICLE_LINE_COUNTER_PCT      50U
#define VEHICLE_LINE_COUNTER_MAX_MS   300U
/* 1: no blind search while the IMU reports a hump (over a crest the tape is
 * out of sight). 0 (as the line_follow bench test, which has no IMU): search
 * as usual -- the IMU also reported humps on flat ground, which stopped the
 * robot from finding the tape on curves. */
#define VEHICLE_LINE_HUMP_NO_SEARCH   0

/*
 * Junction manoeuvre, measured with SENSOR_TEST=junction LF_STRADDLE=1:
 * after the junction is detected (both line sensors just over the crossing)
 * drive straight FORWARD_MM (right turns FORWARD_RIGHT_MM) so the wheels are
 * over the crossing, stop, spin until the new line is reached, stop, and
 * resume line following. A line is reached when it arrives at the LEADING
 * line sensor of the spin (the branch straight ahead starts in the gap and
 * leaves through the other one); with SEEK_GAP the spin goes on until the
 * line has passed that sensor into the gap (left spins stopped a few
 * degrees short otherwise). A left/right turn stops at the first line, a
 * U-turn (spun left) passes the side arm and stops at the second (the line
 * it came on). Times are from the speed, since the controller has no
 * distance feedback.
 */
#define VEHICLE_JUNCTION_FORWARD_MM       100U
#define VEHICLE_JUNCTION_FORWARD_RIGHT_MM 120U
#define VEHICLE_JUNCTION_SPIN_CM_S        8      /* each wheel, opposite ways */
#define VEHICLE_JUNCTION_SETTLE_MS        400U   /* wheels stopped before/after the spin */
#define VEHICLE_JUNCTION_SPIN_MAX_MS      4000U  /* no line by then: stop */
/* Left turns and U-turns: true spun on until the new line had passed the
 * leading sensor into the gap, which turned too far on the robot; false
 * stops as soon as the leading sensor sees the line (as right turns), and
 * line following takes out any small shortfall. */
#define VEHICLE_JUNCTION_SEEK_GAP_LEFT    false
#define VEHICLE_JUNCTION_SEEK_GAP_RIGHT   false  /* right turns were fine without */

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
	 * well below obstacle_stop_mm (the bypass starts ~20 cm from it).
	 * false: stop and wait for GO, as before. */
	bool obstacle_bypass;
	/* May be NULL (then two presses of a toggle GO restart the robot). */
	vehicle_controller_go_reset_t go_reset;
	/* May be NULL (then a fault needs a power cycle). */
	vehicle_controller_restart_t restart;
} vehicle_controller_task_config_t;

/* Opt-in safety/arming task. The callback supplies trusted local GO and
 * emergency-stop inputs and must return promptly without network access.
 * The callback context must remain valid while the task runs. The task
 * exclusively consumes line, barcode, IMU, obstacle, fault and motion-status
 * buffers.
 * It sends ARM only after the safety gate passes and a fresh zero-output
 * motion status is received. While FOLLOWING it then sends one line-steering
 * VELOCITY command per cycle (speed and turn from straddle_steering, 200 ms
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
