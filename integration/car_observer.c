#include "car_observer.h"
#include "car_link.h"
#include "line_status.h"
#include "obs_port.h"
#include "obs_pack.h"
#include "car_measurements.h"
#include "pico/stdlib.h"
#include <stdint.h>

#ifndef PICOCAR_ENABLE_IMU
#define PICOCAR_ENABLE_IMU 0
#endif
#ifndef PICOCAR_ENABLE_MQTT
#define PICOCAR_ENABLE_MQTT 0
#endif

#if PICOCAR_ENABLE_IMU
#include "imu_driver.h"
#include "imu_processor.h"
static imu_state_t s_imu;
static uint32_t s_last_imu_ms;
#endif
#if PICOCAR_ENABLE_MQTT
#include "buddy1.h"
static buddy1_t s_buddy1;
static uint32_t s_last_status_ms;
#endif

void car_observer_init(void)
{
#if PICOCAR_ENABLE_IMU
    imu_driver_init();  /* Driver does not verify I2C ACK; validate on hardware. */
    imu_processor_init(&s_imu);
    s_last_imu_ms = 0U;
#endif
#if PICOCAR_ENABLE_MQTT
    buddy1_init(&s_buddy1, obs_port_ms());
    s_last_status_ms = 0U;
#endif
}

void car_observer_poll(void)
{
#if PICOCAR_ENABLE_IMU || PICOCAR_ENABLE_MQTT
    uint32_t now_ms = obs_port_ms();
#endif
#if PICOCAR_ENABLE_IMU
    if ((uint32_t)(now_ms - s_last_imu_ms) >= 50U) {
        accel_data_t a = {0};
        s_last_imu_ms = now_ms;
        imu_driver_read_accel(&a);
        float meters = 0.0f;
        /* Hump height requires calibrated encoder data. Until available,
           IMU processing can observe shock/tilt only; hump not validated. */
        (void)car_measurements_read_distance(&meters);
        imu_processor_update(&s_imu, &a, meters);
        /* Do not latch motor faults automatically from unvalidated IMU data. */
    }
#endif
#if PICOCAR_ENABLE_MQTT
    if ((uint32_t)(now_ms - s_last_status_ms) >= 200U) {
        telemetry_data_t data = {0};
        const uint32_t fault_bits = car_fault_bits_get();
        data.message_type = MSG_TELEMETRY;
        data.vehicle_state = fault_bits ? VEHICLE_FAULT :
            (vc_port_run_enabled() ? VEHICLE_RUNNING : VEHICLE_PAUSED);
        data.line_state = line_status_is_present(now_ms) ? LINE_FOUND : LINE_UNKNOWN;
        data.obstacle_state = car_obstacle_present() ? OBSTACLE_DETECTED : OBSTACLE_CLEAR;
        data.terrain_state = TERRAIN_UNKNOWN;
#if PICOCAR_ENABLE_IMU
        data.terrain_state = s_imu.hump_active ? TERRAIN_HUMP : TERRAIN_UNKNOWN;
#endif
        data.battery_pct = 0U; /* No battery gauge; 0 does not mean empty. */
        if ((fault_bits & OBS_FAULT_NO_ECHO) != 0U)
            data.fault_flags |= FAULT_NO_ECHO;
        buddy1_set_telemetry(&s_buddy1, &data);
        s_last_status_ms = now_ms;
    }
    buddy1_poll(&s_buddy1, now_ms);
    remote_command_t command;
    while (buddy1_take_command(&s_buddy1, &command)) {
        if (command.type == CMD_START) {
            vc_port_set_run(true);
        } else if (command.type == CMD_STOP) {
            vc_port_set_run(false);
        } else if (command.type == CMD_JUNCTION && command.argument <= 4U) {
            vc_port_motion_send(command.argument);
        }
    }
#endif
}
