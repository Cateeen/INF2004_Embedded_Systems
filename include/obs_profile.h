/* obs_profile.h - pure geometry: width, closest point, side clearance, bypass plan. */
#ifndef OBS_PROFILE_H
#define OBS_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

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

/* Convert fine-scan distances into width, closest point, side clearances, chosen side and bypass distances. Returns false if nothing is blocked.*/
bool obs_profile_compute(const uint8_t *packed, uint8_t n, uint8_t angle0_deg, uint8_t step_deg, obs_profile_t *out);

#endif