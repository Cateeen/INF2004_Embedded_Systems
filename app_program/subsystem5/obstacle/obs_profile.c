#include "obs_profile.h"
#include <stddef.h>
#include <math.h>

#define PI_F 3.14159265f

/* sat16 - round a float and clamp it to 0..max. */
static uint16_t sat16(float v, uint16_t max)
{
    if (v <= 0.0f) {
        return 0U;
    }
    if (v >= (float)max) {
        return max;
    }
    return (uint16_t)(v + 0.5f);
}

/**
 * obs_profile_compute - turn a fine-scan into an obstacle description.
 * Params : distances = n millimetre readings (OBS_DIST_NONE = nothing seen),
 *          n = sample count (max 25), angle0_deg = angle of sample 0,
 *          step_deg = angle step, out = result.
 * Returns: INVALID for invalid inputs, CLEAR when nothing is detected within
 *          OBS_BLOCK_MM, or FOUND when out holds a profile.
 */
/* x: lateral position, positive to the robot's RIGHT (x = d*cos(angle)).
 * y: forward distance (y = d*sin(angle))
 */
obs_profile_status_t obs_profile_compute(const uint16_t *dist, uint8_t n, uint8_t angle0_deg, uint8_t step_deg, obs_profile_t *out){
    float    px[OBS_FINE_MAX_SAMPLES], py[OBS_FINE_MAX_SAMPLES];
    uint8_t  cid[OBS_FINE_MAX_SAMPLES];
    float xr = -1.0e9f, xl = 1.0e9f, d_xr = 0.0f, d_xl = 0.0f, far_y = 0.0f;
    uint16_t closest = 0xFFFFU;
    uint8_t ncl = 0U, best = 0U, i, c;
    uint32_t best_dev = 0xFFFFU;
    bool blocked_first, blocked_last;

    if (out == NULL) {
        return OBS_PROFILE_INVALID;
    }
    *out = (obs_profile_t){0};
    if (dist == NULL || n == 0U || n > OBS_FINE_MAX_SAMPLES ||
        step_deg == 0U ||
        (uint32_t)angle0_deg + (uint32_t)(n - 1U) * step_deg > 180U) {
        return OBS_PROFILE_INVALID;
    }
    for (i = 0U; i < n; i++) {
        if (dist[i] != OBS_DIST_NONE &&
            (dist[i] < OBS_MIN_RANGE_MM || dist[i] > OBS_MAX_RANGE_MM)) {
            return OBS_PROFILE_INVALID;
        }
    }

    for (i = 0U; i < n; i++) {
        float rad = ((float)(angle0_deg + (uint32_t)i * step_deg) - (float)(OBS_ANGLE_AHEAD_DEG - 90U))* (PI_F / 180.0f);
        px[i] = (float)dist[i] * cosf(rad);
        py[i] = (float)dist[i] * sinf(rad);
        if ((dist[i] != OBS_DIST_NONE) && (dist[i] < OBS_BLOCK_MM)) {
            bool join = false;
            if ((i > 0U) && (cid[i - 1U] != 0U)) {
                float dx = px[i] - px[i - 1U];
                float dy = py[i] - py[i - 1U];
                join = (sqrtf((dx * dx) + (dy * dy)) <= (float)OBS_CLUSTER_GAP_MM);
            }
            if (join) {
                cid[i] = cid[i - 1U];
            } else {
                ncl++;
                cid[i] = ncl;
            }
        } else {
            cid[i] = 0U;
        }
    }
    if (ncl == 0U) {
        out->action = OBS_ACT_CONTINUE;
        return OBS_PROFILE_CLEAR;
    }

    for (c = 1U; c <= ncl; c++) {                      /* pick the cluster nearest straight ahead */
        uint32_t dev = 0xFFFFU;
        for (i = 0U; i < n; i++) {
            if (cid[i] == c) {
                uint32_t a = angle0_deg + (uint32_t)i * step_deg;
                uint32_t dd = (a > OBS_ANGLE_AHEAD_DEG) ? (a - OBS_ANGLE_AHEAD_DEG) : (OBS_ANGLE_AHEAD_DEG - a);
                if (dd < dev) { dev = dd; }
            }
        }
        if (dev < best_dev) { best_dev = dev; best = c; }
    }

    for (i = 0U; i < n; i++) {
        if (cid[i] == best) {
            float x = px[i];
            float y = py[i];
            if (dist[i] < closest) { closest = (uint16_t)dist[i]; }
            if (x > xr) { xr = x; d_xr = (float)dist[i]; }
            if (x < xl) { xl = x; d_xl = (float)dist[i]; }
            if (y > far_y) { far_y = y; }
        }
    }
    blocked_first = (cid[0] == best);
    blocked_last = (cid[n - 1U] == best);

    /* Window filled: that edge is outside the scan, so push it out by the beam half-angle. */
    {
        float beam = tanf(15.0f * (PI_F / 180.0f));
        if (blocked_first) { xr += d_xr * beam; }
        if (blocked_last)  { xl -= d_xl * beam; }
    }

    /* Pull each edge in by the beam half-angle error (calibrate). */
    {
        float trim = tanf(OBS_BEAM_TRIM_DEG * (PI_F / 180.0f));
        xr -= d_xr * trim;
        xl += d_xl * trim;
        if (xr < xl) {
            float mid = 0.5f * (xr + xl);
            xr = mid;
            xl = mid;
        }
    }

    /* Nearest non-obstacle object on each side, for clearance.
     * Side is decided by sample index, not by x: a lower angle is the robot's right, a higher
     * angle is its left. An object that lies inside the obstacle's x-span but sits on one side
     * of it in the scan therefore gives 0 clearance instead of being ignored. */
    {
        uint8_t i_first = 0xFFU, i_last = 0U;
        float right_c = 1.0e9f, left_c = 1.0e9f;

        for (i = 0U; i < n; i++) {
            if (cid[i] == best) {
                if (i_first == 0xFFU) { i_first = i; }
                i_last = i;
            }
        }
        for (i = 0U; i < n; i++) {
            if ((cid[i] == best) || (dist[i] == OBS_DIST_NONE) || (dist[i] >= OBS_SIDE_RANGE_MM)) {
                continue;
            }
            if (i > i_last) {                         /* left of the obstacle */
                float c = xl - px[i];
                if (c < 0.0f) { c = 0.0f; }
                if (c < left_c) { left_c = c; }
            }
            if (i < i_first) {                        /* right of the obstacle */
                float c = px[i] - xr;
                if (c < 0.0f) { c = 0.0f; }
                if (c < right_c) { right_c = c; }
            }
        }
        if (blocked_first) { right_c = 0.0f; }
        if (blocked_last)  { left_c = 0.0f; }
        out->right_mm = ((right_c < 1.0e8f) ? sat16(right_c, 2047U) : (uint16_t)OBS_MAX_RANGE_MM);
        out->left_mm  = ((left_c < 1.0e8f) ? sat16(left_c, 2047U) : (uint16_t)OBS_MAX_RANGE_MM);
    }

    out->closest_mm = closest;
    out->width_mm = sat16(xr - xl, OBS_OFFSET_MAX_MM);

    /* Choose a side: it must fit the robot plus margin on both sides. */
    {
        uint16_t need = (uint16_t)(OBS_ROBOT_WIDTH_MM + 2U * OBS_SAFETY_MARGIN_MM);
        bool ok_r = (out->right_mm >= need);
        bool ok_l = (out->left_mm >= need);
        float half = 0.5f * (float)OBS_ROBOT_WIDTH_MM + (float)OBS_SAFETY_MARGIN_MM;
        float depth = (float)out->width_mm;
        float adv;

        if (depth < (float)OBS_DEPTH_MIN_MM) {
            depth = (float)OBS_DEPTH_MIN_MM;            /* assumption: depth ~ width */
        }
        adv = far_y + depth + (float)OBS_ROBOT_LENGTH_MM + (float)OBS_SAFETY_MARGIN_MM;
        out->advance_mm = sat16(adv, OBS_OFFSET_MAX_MM);

        if (ok_r && (!ok_l || (out->right_mm >= out->left_mm))) {
            out->action = OBS_ACT_RIGHT;
            out->lateral_mm = sat16(((xr > 0.0f) ? xr : 0.0f) + half, OBS_OFFSET_MAX_MM);
        } else if (ok_l) {
            out->action = OBS_ACT_LEFT;
            out->lateral_mm = sat16(((xl < 0.0f) ? -xl : 0.0f) + half, OBS_OFFSET_MAX_MM);
        } else {
            out->action = OBS_ACT_REVERSE;              /* no side fits: back off and rescan */
            out->lateral_mm = 0U;
        }
    }
    out->valid = true;
    return OBS_PROFILE_FOUND;
} 