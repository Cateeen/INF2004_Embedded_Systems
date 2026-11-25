/*
 * spin_scan.h - obstacle width from a spin-in-place ultrasonic scan.
 *
 * The servo holds the HC-SR04 pointing ahead and the whole robot turns in
 * place; each reading is stored with the encoder heading it was taken at.
 * This module is pure maths (no kernel, no hardware), shared by the
 * spin_width hardware test and the vehicle.
 *
 * Headings are degrees relative to the line direction (the heading the robot
 * stopped at), + = LEFT, - = RIGHT. Distances are sensor distances in mm.
 *
 * Scan sequence (driven by the caller):
 *   1. FIND   from 0 deg, step right (coarse) feeding spin_scan_edge_update()
 *             until it reports the obstacle lost; readings are not kept.
 *   2. Turn to spin_scan_sweep_start_deg() (beyond the right edge).
 *   3. SWEEP  step left (fine), store every reading, feed the left edge
 *             tracker with readings at >= 0 deg; stop when
 *             spin_scan_sweep_done() says so.
 *   4. Turn back to 0 deg, spin_scan_sort(), spin_scan_analyse().
 */
#ifndef SPIN_SCAN_H
#define SPIN_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#define SPIN_SCAN_MAX_SAMPLES 160U
#define SPIN_SCAN_LOST_MISSES 2U   /* consecutive non-obstacle readings = edge passed */
#define SPIN_SCAN_ROOM_EXTRA_DEG 3.0f /* look this much past the computed room angle */

typedef struct {
	float deg;               /* heading, + = left */
	uint16_t mm;             /* sensor distance, OBS_DIST_NONE = no echo */
} spin_scan_sample_t;

typedef struct {
	float sensor_offset_mm;  /* wheel axle (turning point) to the sensor face */
	float cluster_mm;        /* a reading is the obstacle within this of the ahead echo */
	float trim_deg;          /* beam hears past an edge: pull each edge in by this */
	float centre_zone_deg;   /* obstacle's ahead echo must be within this of 0 */
	float half_deg;          /* never turn further than this either way */
	float lane_mm;           /* free gap needed beside the obstacle to pass it: robot width + clearance */
	float beyond_mm;         /* side objects more than this far beyond the obstacle are
				  * not in the bypass path (assumed depth + robot length + margin) */
	float max_obstacle_mm;   /* the obstacle is within this (the car stops at 300 mm);
				  * anything farther is background */
} spin_scan_config_t;

/* Edge tracker for the live scan. */
typedef struct {
	float ref_mm;            /* obstacle distance (the ahead echo) */
	float last_hit_deg;      /* last reading that heard the obstacle */
	float last_hit_mm;
	float first_miss_deg;    /* first of the consecutive misses */
	unsigned misses;
	bool lost;               /* SPIN_SCAN_LOST_MISSES misses in a row */
	bool hidden;             /* the first miss was a NEARER object */
} spin_scan_edge_t;

/* Free space beside the obstacle on one side. */
typedef struct {
	bool object_seen;        /* another echo beside the obstacle */
	float gap_mm;            /* object's lateral position - obstacle extent */
	float object_mm;         /* object's lateral distance from the line */
	float object_fwd_mm;     /* object's distance ahead of the turning point */
	float reach_mm;          /* how far sideways from the line the scan looked */
	bool passable;           /* a lane_mm gap beside the obstacle was seen to be free */
	unsigned ignored;        /* echoes on this side too far ahead to matter */
	float ahead_limit_mm;    /* ...beyond this distance ahead of the turning point */
} spin_scan_side_t;

typedef struct {
	bool found;              /* an obstacle echo within centre_zone_deg */
	unsigned closest_index;
	unsigned left_index;     /* last sample of the obstacle on each side */
	unsigned right_index;
	float closest_mm;        /* sensor distance of the obstacle's closest point */
	float closest_deg;
	float left_deg;          /* edge headings (left > 0, right < 0) */
	float right_deg;
	float left_edge_mm;      /* sensor distance at each edge reading */
	float right_edge_mm;
	bool left_seen, right_seen;
	bool left_hidden, right_hidden;
	float raw_left_mm, raw_right_mm;   /* extents with no trim */
	float left_mm, right_mm;           /* extents with trim_deg, + = towards that side */
	float raw_width_mm;
	float width_mm;          /* left_mm + right_mm; a lower bound unless both edges seen */
	float mean_step_deg;
	spin_scan_side_t left_free;
	spin_scan_side_t right_free;
} spin_scan_result_t;

typedef enum { SPIN_SCAN_BYPASS_RIGHT = 0, SPIN_SCAN_BYPASS_LEFT } spin_scan_bypass_side_t;

/* Bypass plan from a scan. Lateral values are mm RIGHT of the line (the
 * robot's axle at the scan), so a left target is negative. */
typedef struct {
	bool ok;                 /* a usable side was found */
	bool right_usable;       /* edge seen and the room beside it free (side.passable) */
	bool left_usable;
	bool preferred;          /* the chosen side is the one it sticks out less on */
	spin_scan_bypass_side_t side;
	float edge_x;            /* the passed edge */
	float target_x;          /* car centre: edge + margin + clearance + half the car,
				  * never back towards the obstacle (0 = already clear) */
} spin_scan_bypass_t;

bool spin_scan_valid_mm(uint16_t mm);

/* Lateral extent of an edge heard at heading a_abs (deg, measured towards
 * that side) at sensor distance d_mm, with the beam pulled in by trim_deg. */
float spin_scan_extent(const spin_scan_config_t *config, float d_mm, float a_abs, float trim_deg);

/* Live edge tracking. update() returns true once the edge has been passed. */
void spin_scan_edge_start(spin_scan_edge_t *edge, uint16_t ref_mm, float deg);
bool spin_scan_edge_update(spin_scan_edge_t *edge, const spin_scan_config_t *config,
			   float deg, uint16_t mm);
/* Edge heading (+/-, as stored), half way between the last hit and the first miss. */
float spin_scan_edge_deg(const spin_scan_edge_t *edge);

/* |heading| at which the scan has looked lane_mm beyond an edge at edge_deg,
 * clamped to half_deg. */
float spin_scan_room_deg(const spin_scan_config_t *config, float edge_deg, float edge_mm);

/* Where the sweep starts (negative = right) after the find pass. right_edge
 * NULL or not lost = the edge lies beyond the find limit: start at -half_deg. */
float spin_scan_sweep_start_deg(const spin_scan_config_t *config, const spin_scan_edge_t *right_edge,
				float fine_step_deg);

/* Sweep finished: left edge lost and the room beyond it seen, or half_deg reached. */
bool spin_scan_sweep_done(const spin_scan_config_t *config, const spin_scan_edge_t *left_edge,
			  float deg);

/* Sort samples by heading (right to left). */
void spin_scan_sort(spin_scan_sample_t *samples, unsigned n);

/* Width, edges and free gaps from samples sorted by heading. Returns
 * result->found. */
bool spin_scan_analyse(const spin_scan_sample_t *samples, unsigned n,
		       const spin_scan_config_t *config, spin_scan_result_t *result);

/* Trim that would make this scan measure true_width_mm. False when an edge
 * was not seen. */
bool spin_scan_calibrate_trim(const spin_scan_result_t *result, const spin_scan_config_t *config,
			      float true_width_mm, float *trim_deg);

/* Bypass side: RIGHT unless the obstacle sticks out more than tie_mm further
 * right than left; a side is usable only when its edge was seen and its free
 * gap is passable (set the config's lane_mm to car width + margin + 2 x
 * clearance). If the preferred side is not usable the other one is taken.
 * margin_mm covers the scan's width error. Returns plan->ok. */
bool spin_scan_plan_bypass(const spin_scan_result_t *result, float car_width_mm,
			   float clearance_mm, float margin_mm, float tie_mm,
			   spin_scan_bypass_t *plan);

#endif
