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

// --- UPDATED FOR MQTT INTEGRATION ---

// 1. Define clear event types so Buddy 1 knows what to publish
typedef enum {
    IMU_EVENT_HUMP = 0,
    IMU_EVENT_SHOCK = 1,
    IMU_EVENT_TILT = 2,
    IMU_EVENT_NONE = 3
} imu_event_type_t;

// 2. Your internal RTOS queue message (used between IMU task and Vehicle Controller)
typedef struct {
    imu_event_type_t event_type;  // Changed from uint8_t to enum for clarity
    float value;                  // Hump height in meters (0.0 for shock/tilt)
} imu_event_msg_t;

// Optional application snapshot; Buddy 1's current MQTT wire format is the
// packed telemetry_data_t binary packet, not JSON.
typedef struct {
    uint32_t timestamp_ms;
    float hump_peak_cm;
    bool is_tilted;
    bool is_shock;
    imu_fsm_state_t current_state;
} imu_telemetry_t;

#endif // IMU_TYPES_H