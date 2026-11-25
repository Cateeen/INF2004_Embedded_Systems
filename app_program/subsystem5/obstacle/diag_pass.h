/*
 * diag_pass.h - obstacle width from a diagonal pass (pure logic, no hardware).
 *
 * A flat face echoes only while the sensor is in front of it, whichever way
 * the sensor points (bench: a 9 cm box was heard over exactly 9 cm of sideways
 * travel). So the robot drives diagonally past the face at a fixed heading,
 * with the servo turned to keep the sensor facing the obstacle, and the width
 * is the sideways distance between the points where the echo starts and stops.
 * The robot only translates, so the sensor offset and beam width cancel out.
 *
 * Frame: x = mm to the RIGHT of the line (the robot's axle at the start),
 * y = mm forward along the line. The caller integrates the encoder pose and
 * feeds each new reading with the sensor's position.
 */
#ifndef DIAG_PASS_H
#define DIAG_PASS_H

#include <stdbool.h>
#include <stdint.h>

#define DIAG_PASS_CONFIRM 2U       /* readings in a row to confirm entering the face */
#define DIAG_PASS_LOST    3U       /* readings in a row to confirm leaving it (dropouts) */

typedef enum {
	DIAG_EVENT_NONE = 0,
	DIAG_EVENT_ENTERED,        /* the face is heard (confirmed) */
	DIAG_EVENT_EXITED          /* the face is lost (confirmed) */
} diag_event_t;

typedef struct {
	float face_max_mm;         /* first face echo must be within this */
	float cluster_mm;          /* later echoes within this of the expected face distance */
	/* State. */
	bool have_face;            /* face position known (y of the face plane) */
	float face_y;
	bool on_face;
	unsigned hit_run, miss_run;
	bool have_miss;            /* a miss was seen before the current hit run */
	float last_miss_x, last_hit_x;
	float pending_entry_x, pending_exit_x;
	bool pending_entry_known;
	float pending_face_y;
	/* Results of the last events. */
	bool entry_known;          /* false = it started on the face: no entry edge */
	float entry_x;             /* edge where the echo started */
	float exit_x;              /* edge where the echo stopped */
} diag_pass_t;

typedef enum { DIAG_SIDE_RIGHT = 0, DIAG_SIDE_LEFT } diag_side_t;

typedef struct {
	float left_x, right_x;     /* edges, x right of the line */
	float width_mm;
	float left_extent_mm;      /* how far the obstacle reaches left of the line (may be < 0) */
	float right_extent_mm;     /* ... right of the line */
	diag_side_t side;          /* bypass side: the one it sticks out less on, RIGHT on a tie */
	float target_x;            /* car centre x that clears it on that side */
} diag_plan_t;

/* Start tracking. keep_face keeps a known face position (second leg). */
void diag_pass_start(diag_pass_t *p, float face_max_mm, float cluster_mm, bool keep_face);

/* One reading (OBS_DIST_NONE = no echo) with the sensor at (x, y). */
diag_event_t diag_pass_feed(diag_pass_t *p, float sensor_x, float sensor_y, uint16_t mm);

/* Expected sensor distance to the face from y; false when not known yet. */
bool diag_pass_expected_mm(const diag_pass_t *p, float sensor_y, float *mm);

/* Side and clearing target from both edges. tie_mm: right wins within this. */
bool diag_pass_plan(float left_x, float right_x, float car_width_mm, float clearance_mm,
		    float tie_mm, diag_plan_t *plan);

#endif
