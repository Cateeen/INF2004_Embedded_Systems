// Contains struct and enum

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
 } accel_data_t;

typedef enum {
    IMU_STATE_MONITORING,
    IMU_STATE_HUMP_RISING,
    IMU_STATE_HUMP_FALLING,
    IMU_STATE_SHOCK,
    IMU_STATE_TILT,
    IMU_STATE_FAULT
} imu_fsm_state_t;

typedef struct {
    float hump_peak_m;
    float current_hump_m;
    bool hump_active;
    bool shock_detected;
    bool tilt_detected;
    imu_fsm_state_t fsm_state;
} imu_state_t;

typedef struct {
    uint8_t event_type;
    float value;
} imu_event_msg_t;

#endif // IMU_TYPES_H