#include <assert.h>
#include <stdio.h>
#include "imu_processor.h"
#include "imu_config.h"

static uint32_t clock_ms;
uint32_t test_imu_clock_ms(void) { return clock_ms; }

int main(void)
{
    imu_state_t imu = {0};
    accel_data_t flat = {.ax=0.0f,.ay=0.0f,.az=GRAVITY_MS2};
    accel_data_t ramp = {.ax=-1.4f,.ay=0.0f,.az=9.70f};
    accel_data_t tilted = {.ax=0.0f,.ay=5.0f,.az=8.4f};
    imu_processor_init(&imu);
    clock_ms = 10;
    imu_processor_update(&imu, &flat, 0.0f);
    assert(imu.fsm_state == IMU_STATE_MONITORING);
    clock_ms = 20;
    imu_processor_update(&imu, &ramp, 0.0f);
    assert(imu.fsm_state == IMU_STATE_HUMP_RISING);
    clock_ms = 50;
    imu_processor_update(&imu, &ramp, 0.30f);
    clock_ms = 80;
    imu_processor_update(&imu, &flat, 0.30f);
    assert(imu.hump_peak_m > 0.015f);
    imu_processor_init(&imu);
    clock_ms = 100;
    imu_processor_update(&imu, &tilted, 0.30f);
    assert(imu.tilt_detected);
    puts("PASS: simulated IMU flat/ramp/tilt state changes");
    return 0;
}
