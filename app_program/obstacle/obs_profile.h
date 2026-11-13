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

typedef struct {
    uint16_t closest_mm;
    uint16_t width_mm;
    uint16_t left_mm;       /* free lateral space left of the obstacle  */
    uint16_t right_mm;      /* free lateral space right of the obstacle */
    uint16_t lateral_mm;    /* sideways distance for the chosen bypass  */
    uint16_t advance_mm;    /* forward distance to clear the obstacle   */
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