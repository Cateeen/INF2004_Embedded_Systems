#include <stdio.h>
#include "imu_config.h"
#include "imu_driver.h"
#include "imu_processor.h"
#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();

    /* Wait up to 5 seconds for USB Serial Monitor to connect */
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }

    printf("\n================================================================================\n");
    printf("=== Buddy 4: IMU LSM303DLHC Hardware & Minimum-Bit Test (GP2=SDA, GP3=SCL) ===\n");
    printf("================================================================================\n");
    stdio_flush();

    imu_driver_init();
    imu_state_t state;
    imu_processor_init(&state);

    accel_data_t accel = {0};
    float simulated_distance_m = 0.0f;
    uint32_t last_print_ms = 0U;

    printf("Time(ms) | I2C |  AX   |  AY   |  AZ   | Pitch  | Roll   | Hump(cm) | Packed | State\n");
    printf("------------------------------------------------------------------------------------\n");
    stdio_flush();

    while (true) {
        imu_driver_read_accel(&accel);

        /* Simulate 0.20 m/s forward wheel roll (0.002 m per 10 ms) while climbing a hump
         * so you can test hump height calculation by tilting the sensor on your desk! */
        if (state.fsm_state == IMU_STATE_HUMP_RISING) {
            simulated_distance_m += 0.002f;
        }

        if (accel.i2c_ok) {
            imu_processor_update(&state, &accel, simulated_distance_m);
        } else {
            state.fsm_state = IMU_STATE_FAULT;
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print_ms >= 200U) {
            last_print_ms = now;

            if (!accel.i2c_ok) {
                printf("%8lu | ERR | Check GP2 (SDA) & GP3 (SCL) wiring / 3.3V power!\n",
                       (unsigned long)now);
                stdio_flush();
                sleep_ms(10);
                continue;
            }

            const char *state_str = "UNKNOWN";
            switch (state.fsm_state) {
                case IMU_STATE_MONITORING:   state_str = "MONITORING";   break;
                case IMU_STATE_HUMP_RISING:  state_str = "HUMP_RISING";  break;
                case IMU_STATE_HUMP_FALLING: state_str = "HUMP_FALLING"; break;
                case IMU_STATE_SHOCK:        state_str = "SHOCK!";       break;
                case IMU_STATE_TILT:         state_str = "TILT!";        break;
                case IMU_STATE_FAULT:        state_str = "I2C_FAULT";    break;
            }

            printf("%8lu | OK  | %5.2f | %5.2f | %5.2f | %5.1f° | %5.1f° |  %5.2f   | 0x%04X | %s\n",
                   (unsigned long)now,
                   accel.ax, accel.ay, accel.az,
                   state.current_pitch_deg,
                   state.current_roll_deg,
                   state.hump_peak_m * 100.0f,
                   (unsigned int)state.packed_telemetry,
                   state_str);

            if (state.shock_detected) {
                printf("  >>> [ALERT] SUDDEN IMPACT DETECTED (>3g)! <<<\n");
                state.shock_detected = false;
            }
            if (state.tilt_detected) {
                printf("  >>> [ALERT] UNSAFE TILT DETECTED (>15 deg)! <<<\n");
                state.tilt_detected = false;
            }
            stdio_flush();
        }

        sleep_ms(10); /* 100 Hz (10 ms) sampling loop */
    }
}