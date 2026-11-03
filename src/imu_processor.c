/*
Implementation of filters, hump detection, tilt/shock logic and FSM
*/

#include "imu_processor.h"
#include "imu_config.h"
#include <math.h>
#include "pico/stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Static variables used to keep track of filter history and state machine timiing
static float hp_z_prev = 0.0f;
static float hp_z_out = 0.0f;
static float lp_out = 0.0f;
static uint32_t hump_start_ms = 0;
static float peak_velocity = 0.0f;

// filters the raw Z-axis acceleration to isolate dynamic movement
// az_raw default value (9.8m/s2) when flat
static float process_accel_z(float az_raw) {
    // high pass filter: removes DC offset (gravity)
    hp_z_out = ALPHA_HP * (hp_z_out + az_raw - hp_z_prev);
    hp_z_prev = az_raw;

    // low pass filter: smooths the resulting signal
    lp_out = lp_out + ALPHA_LP * (hp_z_out - lp_out);
    
    return lp_out;
}

/* Event Detection*/

// check if robot has tilted too far
static bool check_tilt(accel_data_t *a) {
    // atan2f returns radians, multiplying by 180/pi returns degrees
    float roll = atan2f(a->ay, a->az) * (180.0f/M_PI);
    float pitch = atan2f(-a->ax, sqrtf(a->ay * a->ay + a->az * a->az)) * (180.0f/M_PI);

    // return true if either axis exceeds threshold
    return (fabsf(roll) > TILT_THRESHOLD_DEG || fabsf(pitch) > TILT_THRESHOLD_DEG);
}

// check if sudden impact
static bool check_shock(accel_data_t *a) {
    float magnitude = sqrtf(a->ax * a->ax + a->ay * a->ay + a->az * a->az);

    return (fabsf(magnitude - GRAVITY_MS2) > SHOCK_THRESHOLD * GRAVITY_MS2);
}

/* State Machine */

// reset all state variables to safe default
void imu_processor_init(imu_state_t *state) {
    state->fsm_state = IMU_STATE_MONITORING;
    state->hump_peak_m = 0.0f;
    state->current_hump_m = 0.0f;
    state->hump_active = false;
    state->shock_detected = false;
    state->tilt_detected = false;
}

// update every 100Hz
void imu_processor_update(imu_state_t *state, accel_data_t *accel) {
    float dynamic_az = process_accel_z(accel->az);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    const float DT_S = 0.01f; // define delta time (10ms = 0.01s)

    // Run state machine
    switch (state->fsm_state) {
    
    case IMU_STATE_MONITORING:
        if (check_shock(accel)) {
            state->shock_detected = true;
            state->fsm_state = IMU_STATE_SHOCK;
        }
        else if (check_tilt(accel)) {
            state->tilt_detected = true;
            state->fsm_state = IMU_STATE_TILT;
        }
        else if (!check_tilt(accel) && dynamic_az > HUMP_THRESHOLD) {
            hump_start_ms = now;
            peak_velocity = 0.0f;
            state->hump_active = true;
            state->fsm_state = IMU_STATE_HUMP_RISING;
        }
        break;
    
    case IMU_STATE_HUMP_RISING:
        // if in this state for > 1.5 seconds, abort
        // prevent getting stucked
        if (now - hump_start_ms > 1500) {
            state->fsm_state = IMU_STATE_MONITORING;
            state->hump_active = false;
            break;
        }

        peak_velocity += dynamic_az * DT_S;

        if (dynamic_az < HUMP_THRESHOLD / 2.0f) {
            state->current_hump_m = (peak_velocity * peak_velocity) / (2.0f * GRAVITY_MS2);

            // record the highest hump
            if (state->current_hump_m > 0.015f) {
                if (state->current_hump_m > state->hump_peak_m) {
                state->hump_peak_m = state->current_hump_m;
                }
            } else {
                state->current_hump_m = 0.0f; // reject noise
            }
            // descending
            state->fsm_state = IMU_STATE_HUMP_FALLING;
        }
        break;
    
    case IMU_STATE_HUMP_FALLING:
        if (fabsf(dynamic_az) < HUMP_THRESHOLD / 4.0f && (now - hump_start_ms > 200)){
            state->fsm_state = IMU_STATE_MONITORING;
            state->hump_active = false;
        }
        break;
    
    case IMU_STATE_SHOCK:
        if (!check_shock(accel)) state->fsm_state = IMU_STATE_MONITORING;
        break;

    case IMU_STATE_TILT:
        if(!check_tilt(accel)) state->fsm_state = IMU_STATE_MONITORING;
        break;

    default:
        state->fsm_state = IMU_STATE_MONITORING;
        break;
    }
}