#include "imu_processor.h"
#include "imu_processor_config.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float get_pitch_deg(const accel_data_t *a) {
    return atan2f(-a->ax, sqrtf(a->ay * a->ay + a->az * a->az)) * (180.0f / (float)M_PI);
}

static float get_roll_deg(const accel_data_t *a) {
    return atan2f(a->ay, a->az) * (180.0f / (float)M_PI);
}

/* Detects when the robot has been lifted by hand or overturned */
static bool check_lifted_or_overturned(const accel_data_t *a, float pitch_deg, float roll_deg) {
    if (a->az < OVERTURN_MIN_AZ) {
        return true; /* Overturned on its side or upside-down */
    }
    if (fabsf(roll_deg) > LIFT_ROLL_DEG) {
        return true; /* Lifted by hand or tipped sideways */
    }
    if (fabsf(pitch_deg) > LIFT_PITCH_DEG) {
        return true; /* Lifted steeper than any ramp */
    }
    return false;
}

static uint8_t quantize_height_qcm(float height_m) {
    float height_cm = height_m * 100.0f;
    if (height_cm < 0.0f) height_cm = 0.0f;
    if (height_cm >= 15.5f) return 62U;
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

bool imu_processor_init(imu_state_t *state) {
    if (state == NULL) return false;
    state->fsm_state             = IMU_STATE_MONITORING;
    state->motion_event          = IMU_EVENT_STATIONARY;
    state->hump_peak_m           = 0.0f;
    state->current_hump_m        = 0.0f;
    state->hump_active           = false;
    state->shock_detected        = false; /* Collision detection removed per Prof */
    state->tilt_detected         = false; /* true when lifted or overturned, until level for IMU_TILT_CLEAR_MS */
    state->last_tilt_ms          = 0U;
    state->last_distance_m       = 0.0f;
    state->hump_start_distance_m = 0.0f;
    state->current_pitch_deg     = 0.0f;
    state->current_roll_deg      = 0.0f;
    state->terrain_pitch_deg     = 0.0f;
    state->terrain_roll_deg      = 0.0f;
    state->pitch_offset_deg      = 0.0f;
    state->roll_offset_deg       = 0.0f;
    state->mount_yaw_cos         = 1.0f;
    state->mount_yaw_sin         = 0.0f;
    state->terrain_filter_alpha  = 1.0f;
    state->terrain_valid         = false;
    state->hump_confirm_ms       = 0U;
    state->hump_candidate        = false;
    state->hump_candidate_ms     = 0U;
    state->hump_candidate_distance_m = 0.0f;
    state->tilt_confirm_ms       = 0U;
    state->tilt_candidate        = false;
    state->tilt_candidate_ms     = 0U;
    state->wheel_left_m          = 0.0f;
    state->wheel_right_m         = 0.0f;
    state->ball_m                = 0.0f;
    state->hump_history_m[0]     = 0.0f;
    state->hump_history_m[1]     = 0.0f;
    state->hump_start_ms         = 0U;
    state->hump_below_since_ms   = 0U;
    state->packed_telemetry      = 0U;
    return true;
}

bool imu_processor_init_vehicle(imu_state_t *state) {
    if (!imu_processor_init(state) ||
        !imu_processor_set_terrain_filter(state, IMU_TERRAIN_FILTER_ALPHA) ||
        !imu_processor_set_mount_yaw(state, IMU_MOUNT_YAW_DEG)) return false;
    state->hump_confirm_ms = IMU_HUMP_CONFIRM_MS;
    state->tilt_confirm_ms = IMU_TILT_CONFIRM_MS;
    return true;
}

void imu_level_calibrator_reset(imu_level_calibrator_t *calibrator) {
    unsigned i;

    if (calibrator == NULL) return;
    for (i = 0U; i < 3U; i++) {
        calibrator->sum[i] = 0.0f;
        calibrator->min[i] = 0.0f;
        calibrator->max[i] = 0.0f;
    }
    calibrator->count = 0U;
}

bool imu_level_calibrator_add(imu_level_calibrator_t *calibrator, const accel_data_t *sample) {
    const float axis[3] = {sample ? sample->ax : 0.0f, sample ? sample->ay : 0.0f,
                           sample ? sample->az : 0.0f};
    unsigned i;

    if (calibrator == NULL || sample == NULL ||
        !isfinite(axis[0]) || !isfinite(axis[1]) || !isfinite(axis[2])) return false;
    if (calibrator->count >= IMU_LEVEL_SAMPLES) return true; /* already full */
    for (i = 0U; i < 3U; i++) {
        if (calibrator->count == 0U || axis[i] < calibrator->min[i]) calibrator->min[i] = axis[i];
        if (calibrator->count == 0U || axis[i] > calibrator->max[i]) calibrator->max[i] = axis[i];
        calibrator->sum[i] += axis[i];
    }
    calibrator->count++;
    return true;
}

bool imu_level_calibrator_full(const imu_level_calibrator_t *calibrator) {
    return calibrator != NULL && calibrator->count >= IMU_LEVEL_SAMPLES;
}

imu_level_result_t imu_processor_apply_level(imu_state_t *state,
                                             const imu_level_calibrator_t *calibrator) {
    accel_data_t level = {0};
    unsigned i;

    if (state == NULL || !imu_level_calibrator_full(calibrator)) return IMU_LEVEL_INCOMPLETE;
    for (i = 0U; i < 3U; i++) {
        if (calibrator->max[i] - calibrator->min[i] > IMU_LEVEL_MAX_SPREAD_MS2)
            return IMU_LEVEL_MOVING;
    }
    level.ax = calibrator->sum[0] / (float)calibrator->count;
    level.ay = calibrator->sum[1] / (float)calibrator->count;
    level.az = calibrator->sum[2] / (float)calibrator->count;
    return imu_processor_set_level(state, &level) ? IMU_LEVEL_OK : IMU_LEVEL_NOT_LEVEL;
}

bool imu_processor_set_level(imu_state_t *state, const accel_data_t *level) {
    float pitch_deg, roll_deg;

    if (state == NULL || level == NULL ||
        !isfinite(level->ax) || !isfinite(level->ay) || !isfinite(level->az) ||
        level->az < OVERTURN_MIN_AZ) return false;
    pitch_deg = get_pitch_deg(level);
    roll_deg  = get_roll_deg(level);
    if (fabsf(pitch_deg) > IMU_LEVEL_MAX_DEG || fabsf(roll_deg) > IMU_LEVEL_MAX_DEG)
        return false;
    state->pitch_offset_deg = pitch_deg;
    state->roll_offset_deg  = roll_deg;
    state->terrain_valid    = false;
    return true;
}

bool imu_processor_set_mount_yaw(imu_state_t *state, float yaw_deg) {
    float yaw_rad;

    if (state == NULL || !isfinite(yaw_deg) || fabsf(yaw_deg) > IMU_MOUNT_YAW_MAX_DEG)
        return false;
    yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    state->mount_yaw_cos = cosf(yaw_rad);
    state->mount_yaw_sin = sinf(yaw_rad);
    state->terrain_valid = false;
    return true;
}

bool imu_processor_set_terrain_filter(imu_state_t *state, float alpha) {
    if (state == NULL || !isfinite(alpha) || alpha <= 0.0f || alpha > 1.0f) return false;
    state->terrain_filter_alpha = alpha;
    state->terrain_valid        = false;
    return true;
}

/* Heights of the three ground contacts from the filtered body angles.
 * Relative to the ball, the rear axle centre sits -L*sin(pitch) higher and
 * each rear wheel a further +/-(W/2)*sin(roll); the lowest contact is ground. */
static float update_contact_heights(imu_state_t *state) {
    const float deg_to_rad = (float)M_PI / 180.0f;
    float sin_pitch = sinf(state->terrain_pitch_deg * deg_to_rad);
    float sin_roll  = sinf(state->terrain_roll_deg * deg_to_rad);
    float axle      = -IMU_WHEELBASE_M * sin_pitch;
    float half_side = 0.5f * IMU_TRACK_WIDTH_M * sin_roll;
    float left      = axle - half_side;
    float right     = axle + half_side;
    float ball      = 0.0f;
    float lowest    = fminf(ball, fminf(left, right));

    state->wheel_left_m  = left - lowest;
    state->wheel_right_m = right - lowest;
    state->ball_m        = ball - lowest;
    return fmaxf(state->ball_m, fmaxf(state->wheel_left_m, state->wheel_right_m));
}

/* The peak only takes a height held for three samples in a row (the minimum
 * of the latest three), so a single noisy sample cannot set it. */
static void record_hump_height(imu_state_t *state, float height_m, float held_m) {
    state->current_hump_m = height_m;
    if (held_m > MIN_HUMP_HEIGHT_M && held_m > state->hump_peak_m)
        state->hump_peak_m = held_m;
}

bool imu_processor_update(imu_state_t *state, const accel_data_t *accel,
                          float current_distance_m, uint32_t now_ms) {
    float hump_m, held_m, pitch_deg, roll_deg;

    if (state == NULL || accel == NULL ||
        !isfinite(accel->ax) || !isfinite(accel->ay) ||
        !isfinite(accel->az) || !isfinite(current_distance_m)) return false;
    pitch_deg = get_pitch_deg(accel) - state->pitch_offset_deg;
    roll_deg  = get_roll_deg(accel) - state->roll_offset_deg;
    /* Undo the board's rotation on the chassis so roll is about the
     * robot's front-back axis and pitch about its rear axle. */
    state->current_roll_deg  = state->mount_yaw_cos * roll_deg -
                               state->mount_yaw_sin * pitch_deg;
    state->current_pitch_deg = state->mount_yaw_sin * roll_deg +
                               state->mount_yaw_cos * pitch_deg;

    /* Smooth the terrain angles so pushing or braking is not read as a slope. */
    if (!state->terrain_valid) {
        state->terrain_pitch_deg = state->current_pitch_deg;
        state->terrain_roll_deg  = state->current_roll_deg;
        state->terrain_valid     = true;
    } else {
        state->terrain_pitch_deg += state->terrain_filter_alpha *
                                    (state->current_pitch_deg - state->terrain_pitch_deg);
        state->terrain_roll_deg  += state->terrain_filter_alpha *
                                    (state->current_roll_deg - state->terrain_roll_deg);
    }
    hump_m = update_contact_heights(state);
    held_m = fminf(hump_m, fminf(state->hump_history_m[0], state->hump_history_m[1]));
    state->hump_history_m[1] = state->hump_history_m[0];
    state->hump_history_m[0] = hump_m;

    /* 1. Global Safety Watchdog: Lifted or Overturned (immediately disarms motors) */
    if (check_lifted_or_overturned(accel, state->current_pitch_deg, state->current_roll_deg)) {
        if (!state->tilt_candidate) {
            state->tilt_candidate    = true;
            state->tilt_candidate_ms = now_ms;
        }
        if ((uint32_t)(now_ms - state->tilt_candidate_ms) < state->tilt_confirm_ms)
            goto pack_telemetry;    /* maybe a jolt: wait, change nothing */
        state->tilt_detected = true;
        state->last_tilt_ms  = now_ms;
        state->hump_candidate = false;
        state->hump_active   = false;
        state->fsm_state     = IMU_STATE_TILT;
        goto pack_telemetry;
    }
    state->tilt_candidate = false;
    if (state->tilt_detected &&
        (uint32_t)(now_ms - state->last_tilt_ms) >= IMU_TILT_CLEAR_MS) {
        state->tilt_detected = false;   /* put down and level again */
    }

    /* 2. Terrain FSM */
    switch (state->fsm_state) {
    case IMU_STATE_MONITORING:
        state->motion_event = (fabsf(accel->ax) > 2.0f) ? IMU_EVENT_ACCEL : IMU_EVENT_STATIONARY;

        if (hump_m <= IMU_HUMP_START_M) {
            state->hump_candidate = false;
            break;
        }
        if (!state->hump_candidate) {
            state->hump_candidate            = true;
            state->hump_candidate_ms         = now_ms;
            state->hump_candidate_distance_m = current_distance_m;
        }
        if ((uint32_t)(now_ms - state->hump_candidate_ms) >= state->hump_confirm_ms) {
            /* Held long enough: a hump, from where the climb began. */
            state->hump_candidate        = false;
            state->hump_start_distance_m = state->hump_candidate_distance_m;
            state->last_distance_m       = current_distance_m;
            state->hump_active           = true;
            state->hump_start_ms         = now_ms;
            state->motion_event          = IMU_EVENT_CLIMBING;
            state->fsm_state             = IMU_STATE_HUMP_RISING;
            record_hump_height(state, hump_m, held_m);
        }
        break;

    case IMU_STATE_HUMP_RISING:
        state->motion_event = IMU_EVENT_CLIMBING;

        if ((uint32_t)(now_ms - state->hump_start_ms) > IMU_HUMP_TIMEOUT_MS) {
            state->fsm_state   = IMU_STATE_MONITORING;
            state->hump_active = false;
            break;
        }

        record_hump_height(state, hump_m, held_m);
        state->last_distance_m = current_distance_m;

        if (hump_m <= IMU_HUMP_END_M) {
            state->hump_start_ms       = now_ms; /* settle timeout */
            state->hump_below_since_ms = now_ms; /* end-of-hump hold */
            state->fsm_state           = IMU_STATE_HUMP_FALLING;
        }
        break;

    case IMU_STATE_HUMP_FALLING:
        state->motion_event = IMU_EVENT_DESCENDING;
        state->current_hump_m = hump_m;

        if (hump_m > IMU_HUMP_START_M) {
            state->last_distance_m = current_distance_m;
            state->hump_start_ms   = now_ms;
            state->fsm_state       = IMU_STATE_HUMP_RISING;
            record_hump_height(state, hump_m, held_m);
            break;
        }

        /* Restart the hold whenever a contact is back above the end level. */
        if (hump_m >= IMU_HUMP_END_M) state->hump_below_since_ms = now_ms;

        if ((uint32_t)(now_ms - state->hump_below_since_ms) >= IMU_HUMP_END_HOLD_MS ||
            (uint32_t)(now_ms - state->hump_start_ms) > IMU_HUMP_SETTLE_MS) {
            state->fsm_state   = IMU_STATE_MONITORING;
            state->hump_active = false;
        }
        break;

    case IMU_STATE_TILT:
        if (!check_lifted_or_overturned(accel, state->current_pitch_deg, state->current_roll_deg)) {
            state->fsm_state = IMU_STATE_MONITORING;
        }
        break;

    default:
        state->fsm_state = IMU_STATE_MONITORING;
        break;
    }

    /* 3. Pack 16-Bit Word */
pack_telemetry:
    {
        uint8_t h_qcm = quantize_height_qcm(state->hump_peak_m);
        uint8_t p_idx = quantize_pitch_bucket(state->current_pitch_deg);
        state->packed_telemetry = imu_pack_telemetry((uint8_t)state->fsm_state,
                                                     (uint8_t)state->motion_event,
                                                     h_qcm,
                                                     p_idx);
    }
    return true;
}