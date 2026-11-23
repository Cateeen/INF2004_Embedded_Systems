/* obs_profile.h - pure geometry: width, closest point, side clearance, bypass plan. */
#ifndef OBS_PROFILE_H
#define OBS_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include "obs_profile_config.h"

typedef enum {
    OBS_ACT_STOP = 0, OBS_ACT_CONTINUE, OBS_ACT_LEFT, OBS_ACT_RIGHT, OBS_ACT_REVERSE
} obs_action_t;

typedef enum {
    OBS_PROFILE_INVALID = 0,
    OBS_PROFILE_CLEAR,
    OBS_PROFILE_FOUND
} obs_profile_status_t;

/* Which side of the robot centreline the obstacle mainly lies on. */
typedef enum {
    OBS_SIDE_UNKNOWN = 0, OBS_SIDE_CENTRE, OBS_SIDE_LEFT, OBS_SIDE_RIGHT
} obs_side_t;

typedef struct {
    uint16_t closest_mm;
    uint16_t width_mm;
    uint16_t left_mm;       /* free lateral space left of the obstacle  */
    uint16_t right_mm;      /* free lateral space right of the obstacle */
    uint16_t lateral_mm;    /* sideways distance for the chosen bypass  */
    uint16_t advance_mm;    /* forward distance to clear the obstacle   */
    /* Obstacle edges relative to the robot centreline: left_edge_mm is how far
     * the obstacle extends to the LEFT of centre, right_edge_mm how far to the
     * RIGHT. Negative when the whole obstacle lies on the other side, so
     * width_mm == left_edge_mm + right_edge_mm (within rounding). */
    int16_t  left_edge_mm;
    int16_t  right_edge_mm;
    /* false when the obstacle filled that end of the scan: the edge lies
     * beyond the sweep and its position is only an extrapolated minimum. */
    bool     left_edge_seen;
    bool     right_edge_seen;
    /* Heard at BOTH ends of the sweep. The HC-SR04 beam (measured >= +/-28 deg
     * at 30 cm) is wider than the +/-20 deg sweep, so a narrow object close up
     * and a wide surface both look like this: width and edges are then not
     * measurements, only side/closest/range_trend are meaningful. */
    bool     smeared;
    /* obs_side_t. With BOTH edges seen: from the obstacle's centre position.
     * Otherwise: from range_trend_mm, because off-axis echoes are weaker and
     * read 10-20 mm longer, so the range is shortest pointing at the object.
     * A weak cue (bench: 5 of 6 off-centre scans right, none wrong-way). */
    uint8_t  object_side;
    /* Least-squares range change across the obstacle's samples, leftmost minus
     * rightmost. > 0: farther on the left, so the object leans RIGHT. */
    int16_t  range_trend_mm;
    uint8_t  action;        /* obs_action_t */
    bool     valid;
} obs_profile_t;

/* Ordered scan distances are in millimetres or OBS_DIST_NONE for a confirmed
 * no-object measurement. Sensor failures must not be represented as NONE.
 * CLEAR means no detected obstacle in the scan, not proof that the path is safe.
 * The computed bypass is advisory and has not been validated on hardware. */
obs_profile_status_t obs_profile_compute(const uint16_t *distances, uint8_t n,
                                         uint8_t angle0_deg, uint8_t step_deg,
                                         obs_profile_t *out);

#endif