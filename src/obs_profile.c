#include "obs_profile.h"
#include "obs_cfg.h"
#include "obs_pack.h"
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
 * Params : packed = n distances (12 bits each, OBS_DIST_NONE = nothing seen),
 *          n = sample count (max 25), angle0_deg = angle of sample 0,
 *          step_deg = angle step, out = result.
 * Returns: false when no sample is closer than OBS_BLOCK_MM (nothing to avoid),
 *          true when out holds a valid profile.
 */
/* x: lateral position, positive to the robot's RIGHT (x = d*cos(angle)).
 * y: forward distance (y = d*sin(angle))
 */
bool obs_profile_compute(const uint8_t *packed, uint8_t n, uint8_t angle0_deg, uint8_t step_deg, obs_profile_t *out){
    uint32_t dist[OBS_FINE_MAX_SAMPLES];
    float    px[OBS_FINE_MAX_SAMPLES], py[OBS_FINE_MAX_SAMPLES];
    uint8_t  cid[OBS_FINE_MAX_SAMPLES];
    float xr = -1.0e9f, xl = 1.0e9f, d_xr = 0.0f, d_xl = 0.0f, far_y = 0.0f;
    uint16_t closest = 0xFFFFU;
    uint8_t ncl = 0U, best = 0U, i, c;
    uint32_t best_dev = 0xFFFFU;
    bool blocked_first, blocked_last;

    out->valid = false;
    out->action = OBS_ACT_CONTINUE;
    if (n > OBS_FINE_MAX_SAMPLES) {
        n = (uint8_t)OBS_FINE_MAX_SAMPLES;
    }

    for (i = 0U; i < n; i++) {
        float rad = (float)(angle0_deg + (uint32_t)i * step_deg) * (PI_F / 180.0f);
        dist[i] = obs_bits_get(packed, (uint32_t)i * OBS_DIST_BITS, OBS_DIST_BITS);
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
        return false;
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

    /* Nearest non-obstacle object on each side, for clearance. */
    {
        float right_obj = 1.0e9f, left_obj = -1.0e9f;
        for (i = 0U; i < n; i++) {
            uint32_t d = dist[i];
            if ((cid[i] != best) && (d != OBS_DIST_NONE) && (d < OBS_SIDE_RANGE_MM)) {
                float x = px[i];
                if ((x > xr) && (x < right_obj)) { right_obj = x; }
                if ((x < xl) && (x > left_obj)) { left_obj = x; }
            }
        }
        out->right_mm = blocked_first ? 0U
                      : ((right_obj < 1.0e8f) ? sat16(right_obj - xr, 2047U) : (uint16_t)OBS_MAX_RANGE_MM);
        out->left_mm  = blocked_last ? 0U
                      : ((left_obj > -1.0e8f) ? sat16(xl - left_obj, 2047U) : (uint16_t)OBS_MAX_RANGE_MM);
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
    return true;
} 