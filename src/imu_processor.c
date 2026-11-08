#include "imu_processor.h"
#include "imu_config.h"
#include <math.h>
#include "pico/stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static uint32_t hump_start_ms = 0;

static float get_pitch_deg(const accel_data_t *a) {
    return atan2f(-a->ax, sqrtf(a->ay * a->ay + a->az * a->az)) * (180.0f / (float)M_PI);
}

static float get_roll_deg(const accel_data_t *a) {
    return atan2f(a->ay, a->az) * (180.0f / (float)M_PI);
}

static bool check_tilt(float pitch_deg, float roll_deg) {
    return (fabsf(roll_deg) > TILT_THRESHOLD_DEG || fabsf(pitch_deg) > TILT_THRESHOLD_DEG);
}

static bool check_shock(const accel_data_t *a) {
    float magnitude = sqrtf(a->ax * a->ax + a->ay * a->ay + a->az * a->az);
    return (fabsf(magnitude - GRAVITY_MS2) > SHOCK_THRESHOLD * GRAVITY_MS2);
}

static uint8_t quantize_height_qcm(float height_m) {
    float height_cm = height_m * 100.0f;
    if (height_cm < 0.0f) height_cm = 0.0f;
    uint32_t qcm = (uint32_t)(height_cm * 4.0f + 0.5f); /* 0.25 cm steps */
    if (qcm > 62U) qcm = 62U;
    return (uint8_t)qcm;
}

static uint8_t quantize_pitch_bucket(float pitch_deg) {
    int32_t idx = (int32_t)((pitch_deg + 35.0f) / 5.0f);
    if (idx < 0)  idx = 0;
    if (idx > 15) idx = 15;
    return (uint8_t)idx;
}

void imu_processor_init(imu_state_t *state) {
    state->fsm_state             = IMU_STATE_MONITORING;
    state->motion_event          = IMU_EVENT_STATIONARY;
    state->hump_peak_m           = 0.0f;
    state->current_hump_m        = 0.0f;
    state->hump_active           = false;
    state->shock_detected        = false;
    state->tilt_detected         = false;
    state->last_distance_m       = 0.0f;
    state->hump_start_distance_m = 0.0f;
    state->current_pitch_deg     = 0.0f;
    state->current_roll_deg      = 0.0f;
    state->packed_telemetry      = 0U;
}

void imu_processor_update(imu_state_t *state, accel_data_t *accel, float current_distance_m) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    state->current_pitch_deg = get_pitch_deg(accel);
    state->current_roll_deg  = get_roll_deg(accel);

    /* 1. Global Safety Watchdog (Active across all states) */
    if (check_shock(accel)) {
        state->shock_detected = true;
        state->motion_event   = IMU_EVENT_SHOCK;
        state->fsm_state      = IMU_STATE_SHOCK;
        return;
    }
    if (check_tilt(state->current_pitch_deg, state->current_roll_deg)) {
        state->tilt_detected = true;
        state->fsm_state     = IMU_STATE_TILT;
        return;
    }

    /* 2. Terrain FSM */
    switch (state->fsm_state) {
    case IMU_STATE_MONITORING:
        state->motion_event = (fabsf(accel->ax) > 2.0f) ? IMU_EVENT_ACCEL : IMU_EVENT_STATIONARY;

        if (state->current_pitch_deg > PITCH_START_DEG) {
            state->hump_start_distance_m = current_distance_m;
            state->last_distance_m       = current_distance_m;
            state->current_hump_m        = 0.0f;
            state->hump_active           = true;
            hump_start_ms                = now;
            state->motion_event          = IMU_EVENT_CLIMBING;
            state->fsm_state             = IMU_STATE_HUMP_RISING;
        }
        break;

    case IMU_STATE_HUMP_RISING:
        state->motion_event = IMU_EVENT_CLIMBING;

        // Auto-timeout if stationary on ramp for > 5 seconds
        if (now - hump_start_ms > 5000U) {
            state->fsm_state   = IMU_STATE_MONITORING;
            state->hump_active = false;
            break;
        }

        // Integrate height: dh = ds * sin(pitch)
        float delta_dist = current_distance_m - state->last_distance_m;
        if (delta_dist > 0.0f) {
            float pitch_rad = state->current_pitch_deg * ((float)M_PI / 180.0f);
            state->current_hump_m += delta_dist * sinf(pitch_rad);
        }
        state->last_distance_m = current_distance_m;

        // When pitch levels out below crest threshold or tilts downward
        if (state->current_pitch_deg <= PITCH_END_DEG) {
            if (state->current_hump_m > MIN_HUMP_HEIGHT_M) {
                if (state->current_hump_m > state->hump_peak_m) {
                    state->hump_peak_m = state->current_hump_m;
                }
            }
            hump_start_ms    = now;
            state->fsm_state = IMU_STATE_HUMP_FALLING;
        }
        break;

    case IMU_STATE_HUMP_FALLING:
        state->motion_event = IMU_EVENT_DESCENDING;

        // Re-climb check: if pitch tilts nose-up again, transition back to RISING
        if (state->current_pitch_deg > PITCH_START_DEG) {
            state->last_distance_m = current_distance_m;
            hump_start_ms          = now;
            state->fsm_state       = IMU_STATE_HUMP_RISING;
            break;
        }

        // Return to MONITORING once leveled flat OR after 4-second timeout
        if ((fabsf(state->current_pitch_deg) < PITCH_END_DEG) || (now - hump_start_ms > 4000U)) {
            state->fsm_state   = IMU_STATE_MONITORING;
            state->hump_active = false;
        }
        break;

    case IMU_STATE_SHOCK:
        if (!check_shock(accel)) state->fsm_state = IMU_STATE_MONITORING;
        break;

    case IMU_STATE_TILT:
        if (!check_tilt(state->current_pitch_deg, state->current_roll_deg)) {
            state->fsm_state = IMU_STATE_MONITORING;
        }
        break;

    default:
        state->fsm_state = IMU_STATE_MONITORING;
        break;
    }

    /* 3. Pack 16-Bit Word */
    uint8_t h_qcm = quantize_height_qcm(state->hump_peak_m);
    uint8_t p_idx = quantize_pitch_bucket(state->current_pitch_deg);
    state->packed_telemetry = imu_pack_telemetry((uint8_t)state->fsm_state,
                                                 (uint8_t)state->motion_event,
                                                 h_qcm,
                                                 p_idx);
}