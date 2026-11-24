/*
 * obstacle_bypass.h - scan an obstacle by turning in place, then drive round
 * it and rejoin the line. DRIVES THE MOTORS DIRECTLY.
 *
 * Shared by SENSOR_TEST=spin_width (bench test, OBS-H01/H03) and the vehicle
 * (VEHICLE=1), so the vehicle runs exactly the tested manoeuvre:
 *   1. FIND   step right until the obstacle is lost (not measured).
 *   2. SWEEP  step left across it, storing heading + distance, until it is
 *             lost on the left and the room beside it has been seen.
 *   3. Turn back to the start heading; edges, width and free gaps from
 *             subsystem5/obstacle/spin_scan.c.
 *   4. BYPASS choose the side (RIGHT unless the obstacle sticks out further
 *             right than left; a side with an unseen edge or too little room
 *             is skipped), turn 90 deg, drive until the car centre is the edge
 *             + margin + clearance + half the car out, turn back.
 *   5. PASS   drive past; from min_depth_mm past the face, turn to face the
 *             line every pass_step_mm and read across (the sensor faces the
 *             obstacle's side square on). Anything in the strip the car will
 *             cross = still beside it.
 *   6. REJOIN when clear: half the car + clearance further, turn to the line,
 *             drive onto it, turn back along it.
 *
 * The calling task must be the motor driver's writer for the whole run (the
 * driver refuses any other task, and the calls here ignore errors): the
 * bench test initialises the driver itself; the vehicle borrows it from the
 * motion task (motion_task_borrow_motors). Call only from a task pinned to core 0
 * (TP_PRC1): it reads the encoders directly. The servo must point ahead.
 * Poses are from the encoders: x mm right of the line, y mm along it, from
 * the point where the scan started (the robot's axle).
 */
#ifndef OBSTACLE_BYPASS_H
#define OBSTACLE_BYPASS_H

#include <tk/tkernel.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	/* Spin calibration (SENSOR_TEST=spin_heading). */
	float track_mm;              /* effective wheel track: 116 */
	uint8_t spin_pwm;            /* 25 % */
	uint8_t spin_pwm_slow;       /* 20 %, last 15 deg of a turn */
	/* Scan. */
	float half_deg;              /* never turn further than this either way: 60 */
	float step_deg;              /* measuring step: 3 */
	float find_step_deg;         /* edge-finding step: 6 */
	float sensor_offset_mm;      /* axle to sensor face: 70 */
	float cluster_mm;            /* obstacle = within this of the ahead echo: 60 */
	float lane_mm;               /* free gap needed beside it (raised to car +
				      * margin + 2 x clearance when bypassing): 200 */
	float beyond_mm;             /* side objects farther beyond it are ignored: 400 */
	float max_obstacle_mm;       /* farther echoes are background: 450 */
	float trim_deg;              /* beam correction: 11.6 (bench, 30 cm) */
	float true_width_mm;         /* known width: print the matching trim (0 = off) */
	bool full_sweep;             /* always sweep R..L half_deg */
	/* Bypass. */
	bool bypass;                 /* pull out after the scan */
	bool rejoin;                 /* then drive past and rejoin the line */
	float car_width_mm;          /* 140 */
	float clear_mm;              /* car side to obstacle edge: 50 */
	float margin_mm;             /* extra for the scan's width error: 80 */
	float speed_cm_s;            /* straight legs, wheel-speed PID: 15 */
	float min_depth_mm;          /* first look this far past the face: 50 */
	float pass_step_mm;          /* then every: 100 */
	float max_depth_mm;          /* give up still beside it this far past: 600 */
} obstacle_bypass_config_t;

/* Progress reports (obstacle_bypass_io_t.event), e.g. for the dashboard. */
typedef enum {
	OBSTACLE_BYPASS_EVENT_PROFILE = 0,  /* scanned, side chosen: pulling out next */
	OBSTACLE_BYPASS_EVENT_PASSING,      /* pulled out: driving past, looking for its back */
	OBSTACLE_BYPASS_EVENT_REJOINED      /* back on the line, facing along it */
} obstacle_bypass_event_kind_t;

typedef struct {
	obstacle_bypass_event_kind_t kind;
	/* PROFILE only: measured width (a lower bound unless both edges were
	 * seen), free room beside it on each side (to the next object, or as far
	 * as the scan looked), and the side it will pass on. */
	uint16_t width_mm;
	uint16_t clear_left_mm;
	uint16_t clear_right_mm;
	bool pass_right;
} obstacle_bypass_event_t;

typedef struct {
	/* Newest ahead reading (OBS_DIST_NONE = no echo) and when it was taken;
	 * false while there is none yet. Must not block. */
	bool (*latest_ahead)(void *context, uint16_t *mm, uint32_t *timestamp_ms);
	/* True = stop now (motors braked, run returns ABORTED). Must not block. */
	bool (*abort)(void *context);
	/* Called every loop (~10 ms), e.g. to drain a queue; may be NULL. */
	void (*poll)(void *context);
	void *context;
	const char *tag;             /* log prefix, e.g. "[BYPASS] " */
	/* Progress report; may be NULL. Must not block. */
	void (*event)(void *context, const obstacle_bypass_event_t *event);
} obstacle_bypass_io_t;

typedef enum {
	OBSTACLE_BYPASS_OK = 0,          /* done (as far as the config asked) */
	/* Outcomes: the robot is stopped and safe, the run just could not go on. */
	OBSTACLE_BYPASS_NOT_FOUND,       /* no obstacle echo near ahead */
	OBSTACLE_BYPASS_NO_SIDE,         /* neither side usable: did not move */
	OBSTACLE_BYPASS_TOO_FAR,         /* pull-out beyond the limit: did not move */
	OBSTACLE_BYPASS_TOO_LONG,        /* still beside it at max_depth_mm: stopped there */
	/* Faults: stopped part way. */
	OBSTACLE_BYPASS_ABORTED,
	OBSTACLE_BYPASS_TIMEOUT,
	OBSTACLE_BYPASS_STALL,
	OBSTACLE_BYPASS_WRONG_WAY,
	OBSTACLE_BYPASS_ENCODER,
	OBSTACLE_BYPASS_SENSOR,
	OBSTACLE_BYPASS_FULL,
	OBSTACLE_BYPASS_PID,
	OBSTACLE_BYPASS_CONFIG
} obstacle_bypass_status_t;

obstacle_bypass_config_t obstacle_bypass_default_config(void);

/* One full run (blocking, ~30-60 s). Motors are braked on return. */
obstacle_bypass_status_t obstacle_bypass_run(const obstacle_bypass_config_t *config,
					     const obstacle_bypass_io_t *io);

const char *obstacle_bypass_status_text(obstacle_bypass_status_t status);

/* True for the fault statuses (ABORTED and after). */
bool obstacle_bypass_is_fault(obstacle_bypass_status_t status);

#endif
