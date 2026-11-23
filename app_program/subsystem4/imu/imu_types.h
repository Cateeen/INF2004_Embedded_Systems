#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    float   ax;
    float   ay;
    float   az;
    bool    i2c_ok;
} accel_data_t;

/* 3-bit IMU FSM States (0..5 fit in 3 bits) */
typedef enum {
    IMU_STATE_MONITORING   = 0,
    IMU_STATE_HUMP_RISING  = 1,
    IMU_STATE_HUMP_FALLING = 2,
    IMU_STATE_SHOCK        = 3,
    IMU_STATE_TILT         = 4,
    IMU_STATE_FAULT        = 5
} imu_fsm_state_t;

/* 3-bit Motion Events (0..4 fit in 3 bits) */
typedef enum {
    IMU_EVENT_STATIONARY = 0,
    IMU_EVENT_ACCEL      = 1,
    IMU_EVENT_CLIMBING   = 2,
    IMU_EVENT_DESCENDING = 3,
    IMU_EVENT_SHOCK      = 4
} imu_event_type_t;

typedef struct {
    float           hump_peak_m;
    float           current_hump_m;
    bool            hump_active;
    bool            shock_detected; /* collision detection is disabled */
    bool            tilt_detected;  /* latched until imu_processor_init() */
    imu_fsm_state_t fsm_state;
    imu_event_type_t motion_event;

    float           last_distance_m;
    float           hump_start_distance_m;
    float           current_pitch_deg; /* level-corrected, unfiltered */
    float           current_roll_deg;  /* level-corrected, unfiltered */
    float           terrain_pitch_deg; /* filtered pitch used for hump detection */
    float           terrain_roll_deg;  /* filtered roll used for hump detection */
    float           pitch_offset_deg;  /* resting pitch removed by set_level */
    float           roll_offset_deg;   /* resting roll removed by set_level */
    float           mount_yaw_cos;     /* board rotation about vertical, */
    float           mount_yaw_sin;     /* undone after the level offsets */
    float           terrain_filter_alpha; /* 1 = unfiltered */
    bool            terrain_valid;
    /* Height of each ground contact above the lowest one, from the filtered
     * pitch/roll and the robot geometry (rear wheels + front ball). */
    float           wheel_left_m;
    float           wheel_right_m;
    float           ball_m;
    float           hump_history_m[2]; /* previous two hump heights, newest first */
    uint32_t        hump_start_ms;
    uint32_t        hump_below_since_ms; /* falling: when heights last went below end */
    uint16_t        packed_telemetry; /* 16-bit Minimum-Bit packed word */
} imu_state_t;

/* Minimum-Bit Data Representation (16 bits / 2 bytes total):
 * Bits [2:0]   = fsm_state       (3 bits: 0..5)
 * Bits [5:3]   = motion_event    (3 bits: 0..4)
 * Bits [11:6]  = hump_height_qcm (6 bits: 0..62 -> 0.00..15.50 cm in 0.25 cm steps)
 * Bits [15:12] = pitch_bucket    (4 bits: 0..15 -> -35..+40 deg in 5 deg steps)
 */
static inline uint16_t imu_pack_telemetry(uint8_t state,
                                          uint8_t event,
                                          uint8_t height_qcm,
                                          uint8_t pitch_idx) {
    return (uint16_t)(((uint16_t)(state      & 0x07U))        |
                      ((uint16_t)(event      & 0x07U) << 3U)  |
                      ((uint16_t)(height_qcm & 0x3FU) << 6U)  |
                      ((uint16_t)(pitch_idx  & 0x0FU) << 12U));
}

#endif // IMU_TYPES_H