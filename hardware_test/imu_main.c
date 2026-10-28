#include <stdio.h>
#include "pico/stdlib.h"
#include "imu_driver.h"
#include "imu_processor.h"
#include "imu_config.h"

int main() {
    // 1. Initialize USB Serial for printf
    stdio_init_all();

    // Wait for the USB serial to connect (up to 5 seconds)
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }
    printf("\n=== Buddy 4: IMU LSM303DLHC Test ===\n");

    // 2. Initialize the IMU Driver (I2C, Registers)
    imu_driver_init();
    printf("IMU Initialized. Starting 100Hz loop...\n");

    // 3. Initialize the Processor State
    imu_state_t state;
    imu_processor_init(&state);
    
    accel_data_t accel;
    uint32_t last_print_ms = 0;

    printf("Time(ms) | X (m/s2) | Y (m/s2) | Z (m/s2) | State\n");
    printf("------------------------------------------------------------\n");

    while (1) {
        // 1. Read raw sensor data
        imu_driver_read_accel(&accel);

        // 2. Pass data through the filter and FSM
        imu_processor_update(&state, &accel);

        // 3. Print data every 200ms so it doesn't flood the terminal
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print_ms >= 200) {
            last_print_ms = now;

            // Print X, Y, Z
            printf("%8lu | %8.2f | %8.2f | %8.2f | ", 
                   now, accel.ax, accel.ay, accel.az);

            // Print current FSM state
            switch (state.fsm_state) {
                case IMU_STATE_MONITORING:   printf("MONITORING\n"); break;
                case IMU_STATE_HUMP_RISING:  printf("HUMP RISING\n"); break;
                case IMU_STATE_HUMP_FALLING: printf("HUMP FALLING\n"); break;
                case IMU_STATE_SHOCK:        printf("SHOCK!\n"); break;
                case IMU_STATE_TILT:         printf("TILT!\n"); break;
                default:                     printf("UNKNOWN\n"); break;
            }

            // Print events if they just triggered
            if (state.hump_active && state.current_hump_m > 0.0f) {
                printf("  --> HUMP DETECTED! Peak Height: %.2f cm\n", state.current_hump_m * 100.0f);
                state.current_hump_m = 0.0f; // reset for next hump
            }
            if (state.shock_detected) {
                printf("  --> SUDDEN IMPACT DETECTED!\n");
                state.shock_detected = false; // reset flag
            }
            if (state.tilt_detected) {
                printf("  --> TILT DETECTED!\n");
                state.tilt_detected = false; // reset flag
            }
        }

        // 4. Sleep to maintain 100Hz (10ms)
        sleep_ms(10);
    }
    
    return 0;
}