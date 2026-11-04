#include <stdio.h>

#include "imu_config.h"
#include "imu_driver.h"
#include "imu_processor.h"
#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();

    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }

    printf("\n=== Buddy 4: IMU-only test ===\n");
    stdio_flush();

    imu_driver_init();
    imu_state_t state;
    imu_processor_init(&state);

    accel_data_t accel;
    uint32_t last_print_ms = 0U;

    printf("IMU initialized. Sampling at 100 Hz.\n");
    printf("Time(ms) | X (m/s2) | Y (m/s2) | Z (m/s2) | State\n");
    printf("------------------------------------------------------------\n");
    stdio_flush();

    while (true) {
        imu_driver_read_accel(&accel);
        const float test_distance_m = 0.0f;
        imu_processor_update(&state, &accel, test_distance_m);

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print_ms >= 200U) {
            last_print_ms = now;
            printf("%8lu | %8.2f | %8.2f | %8.2f | ",
                   (unsigned long)now, accel.ax, accel.ay, accel.az);

            switch (state.fsm_state) {
                case IMU_STATE_MONITORING:   printf("MONITORING\n"); break;
                case IMU_STATE_HUMP_RISING:  printf("HUMP RISING\n"); break;
                case IMU_STATE_HUMP_FALLING: printf("HUMP FALLING\n"); break;
                case IMU_STATE_SHOCK:        printf("SHOCK!\n"); break;
                case IMU_STATE_TILT:         printf("TILT!\n"); break;
                default:                     printf("UNKNOWN\n"); break;
            }

            if (state.hump_active && state.current_hump_m > 0.015f) {
                printf("  --> HUMP DETECTED! Peak Height: %.2f cm\n",
                       state.current_hump_m * 100.0f);
                state.current_hump_m = 0.0f;
            }
            if (state.shock_detected) {
                printf("  --> SUDDEN IMPACT DETECTED!\n");
                state.shock_detected = false;
            }
            if (state.tilt_detected) {
                printf("  --> TILT DETECTED!\n");
                state.tilt_detected = false;
            }
            stdio_flush();
        }

        sleep_ms(10);
    }
}
