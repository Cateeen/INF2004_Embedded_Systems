/*
Implementation of filters, pitch calculation, and the Pitch x Distance state machine
*/

#include "imu_processor.h"
#include "imu_config.h"
#include <math.h>
#include "pico/stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint32_t hump_start_ms = 0;

// Calculate Pitch (up/down angle)
static float get_pitch_deg(accel_data_t *a) {
    return atan2f(-a->ax, sqrtf(a->ay * a->ay + a->az * a->az)) * (180.0f / M_PI);
}

// Calculate Roll (left/right angle)
static float get_roll_deg(accel_data_t *a) {
    return atan2f(a->ay, a->az) * (180.0f / M_PI);
}

// Check if the robot has fallen over or tilted beyond safety limits
static bool check_tilt(accel_data_t *a) {
    float roll = get_roll_deg(a);
    float pitch = get_pitch_deg(a);
    return (fabsf(roll) > TILT_THRESHOLD_DEG || fabsf(pitch) > TILT_THRESHOLD_DEG);
}

// Check for sudden impacts
static bool check_shock(accel_data_t *a) {
    float magnitude = sqrtf(a->ax * a->ax + a->ay * a->ay + a->az * a->az);
    return (fabsf(magnitude - GRAVITY_MS2) > SHOCK_THRESHOLD * GRAVITY_MS2);
}

void imu_processor_init(imu_state_t *state) {
    state->fsm_state = IMU_STATE_MONITORING;
    state->hump_peak_m = 0.0f;
    state->current_hump_m = 0.0f;
    state->hump_active = false;
    state->shock_detected = false;
    state->tilt_detected = false;
    state->last_distance_m = 0.0f;
    state->hump_start_distance_m = 0.0f;
    state->current_pitch_deg = 0.0f;
}

// Notice the new parameter "current_distance_m" passed from Buddy 2's encoder
void imu_processor_update(imu_state_t *state, accel_data_t *accel, float current_distance_m) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    state->current_pitch_deg = get_pitch_deg(accel);

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
        // Start a hump if pitch exceeds 5 degrees (matches your ramp test)
        else if (state->current_pitch_deg > PITCH_START_DEG) {
            state->hump_start_distance_m = current_distance_m;
            state->last_distance_m = current_distance_m;
            state->current_hump_m = 0.0f;
            state->hump_active = true;
            hump_start_ms = now;
            state->fsm_state = IMU_STATE_HUMP_RISING;
        }
        break;
    
    case IMU_STATE_HUMP_RISING:
        // Timeout if on the ramp for > 5 seconds
        if (now - hump_start_ms > 5000) {
            state->fsm_state = IMU_STATE_MONITORING;
            state->hump_active = false;
            break;
        }

        // Calculate distance traveled in this loop
        float delta_dist = current_distance_m - state->last_distance_m;
        
        // Add to height using geometry (Height = Distance * sin(Pitch))
        if (delta_dist > 0) {
            float pitch_rad = state->current_pitch_deg * (M_PI / 180.0f);
            state->current_hump_m += delta_dist * sinf(pitch_rad);
        }
        state->last_distance_m = current_distance_m;

        // When the robot flattens out at the top
        if (state->current_pitch_deg <= PITCH_END_DEG) {
            // Only record if > 1.5cm (noise filter)
            if (state->current_hump_m > MIN_HUMP_HEIGHT_M) {
                if (state->current_hump_m > state->hump_peak_m) {
                    state->hump_peak_m = state->current_hump_m;
                }
            } else {
                state->current_hump_m = 0.0f; 
            }
            state->fsm_state = IMU_STATE_HUMP_FALLING;
        }
        break;
    
    case IMU_STATE_HUMP_FALLING:
        // Wait for the robot to be completely flat again
        if (fabsf(state->current_pitch_deg) < PITCH_END_DEG && (now - hump_start_ms > 200)) {
            state->fsm_state = IMU_STATE_MONITORING;
            state->hump_active = false;
        }
        break;
    
    case IMU_STATE_SHOCK:
        if (!check_shock(accel)) state->fsm_state = IMU_STATE_MONITORING;
        break;

    case IMU_STATE_TILT:
        if (!check_tilt(accel)) state->fsm_state = IMU_STATE_MONITORING;
        break;

    default:
        state->fsm_state = IMU_STATE_MONITORING;
        break;
    }
}