#include <stdio.h>
#include "pico/stdlib.h"
#include "buddy1.h"
#include "imu_driver.h"
#include "imu_processor.h"
#include "imu_config.h"
#include "telemetry_protocol.h"

static bool is_hump_state(imu_fsm_state_t state)
{
    return (state == IMU_STATE_HUMP_RISING) ||
           (state == IMU_STATE_HUMP_FALLING);
}

static bool is_imu_alert_state(imu_fsm_state_t state)
{
    return (state == IMU_STATE_SHOCK) ||
           (state == IMU_STATE_TILT) ||
           (state == IMU_STATE_FAULT);
}

static const char *network_state_name(network_state_t state)
{
    switch (state) {
        case NET_DISCONNECTED:  return "DISCONNECTED";
        case NET_CONNECTING:    return "CONNECTING";
        case NET_CONNECTED:     return "CONNECTED";
        case NET_RECONNECT_WAIT:return "RECONNECT_WAIT";
        default:                return "UNKNOWN";
    }
}

static void handle_remote_command(const remote_command_t *command)
{
    const char *name = "UNKNOWN";

    switch (command->type) {
        case CMD_START:          name = "START"; break;
        case CMD_PAUSE:          name = "PAUSE"; break;
        case CMD_RESUME:         name = "RESUME"; break;
        case CMD_STOP:           name = "STOP"; break;
        case CMD_REQUEST_STATUS: name = "REQUEST_STATUS"; break;
        default: break;
    }

    printf("MQTT command received: %s (argument=%u, sequence=%u). "
           "No motor action is bound in this IMU/MQTT test.\n",
           name, (unsigned int)command->argument,
           (unsigned int)command->sequence);
}

int main(void) {
    // 1. Initialize USB Serial for printf
    stdio_init_all();

    // Keep diagnostics visible when the monitor reconnects after a reset.
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    printf("\n=== Buddy 4: IMU LSM303DLHC Test ===\n");
    stdio_flush();
    printf("Starting Buddy 1 network initialization...\n");
    stdio_flush();

    buddy1_t telemetry;
    buddy1_init(&telemetry, to_ms_since_boot(get_absolute_time()));
    printf("Buddy 1 network initialization returned.\n");
    stdio_flush();
    if (!telemetry.initialised) {
        printf("Wi-Fi/MQTT not initialized. Check include/wifi_mqtt_config.h "
               "and the Pico W network.\n");
    } else {
        printf("Buddy 1 Wi-Fi/MQTT adapter initialized.\n");
    }

    // Initialize the IMU Driver (I2C, Registers)
    imu_driver_init();
    printf("IMU Initialized. Starting 100Hz loop...\n");

    // 3. Initialize the Processor State
    imu_state_t state;
    imu_processor_init(&state);
    
    accel_data_t accel;
    uint32_t last_print_ms = 0;
    uint32_t last_network_print_ms = 0;
    bool hump_event_latched = false;
    bool imu_alert_latched = false;
    bool shock_log_pending = false;
    bool tilt_log_pending = false;

    printf("Time(ms) | X (m/s2) | Y (m/s2) | Z (m/s2) | State\n");
    printf("------------------------------------------------------------\n");

    while (1) {
        // 1. Read raw sensor data
        imu_driver_read_accel(&accel);

        // 2. Pass data through the filter and FSM
        const float test_distance_m = 0.0f;
        imu_processor_update(&state, &accel, test_distance_m);

        uint32_t now = to_ms_since_boot(get_absolute_time());
        bool hump_event = state.hump_active && state.current_hump_m > 0.0f;
        if (hump_event) {
            hump_event_latched = true;
        }
        if (state.shock_detected) {
            imu_alert_latched = true;
            shock_log_pending = true;
            state.shock_detected = false;
        }
        if (state.tilt_detected) {
            imu_alert_latched = true;
            tilt_log_pending = true;
            state.tilt_detected = false;
        }
        bool imu_alert_active = is_imu_alert_state(state.fsm_state);

        telemetry_data_t data = {
            .message_type = MSG_TELEMETRY,
            .vehicle_state = (imu_alert_latched || imu_alert_active) ?
                             VEHICLE_FAULT : VEHICLE_RUNNING,
            .line_state = LINE_UNKNOWN,
            .obstacle_state = OBSTACLE_UNKNOWN,
            .terrain_state = (hump_event_latched || state.hump_active ||
                              is_hump_state(state.fsm_state)) ?
                             TERRAIN_HUMP : TERRAIN_FLAT,
            .fault_flags = 0U,
            .sequence = 0U,
            .battery_pct = 0U
        };
        buddy1_set_telemetry(&telemetry, &data);
        buddy1_set_faults(&telemetry,
                          (imu_alert_latched || imu_alert_active) ?
                          FAULT_IMU : 0U);
        uint32_t last_successful_publish_ms = telemetry.last_publish_ms;
        buddy1_poll(&telemetry, now);
        if (telemetry.last_publish_ms != last_successful_publish_ms) {
            hump_event_latched = false;
            imu_alert_latched = false;
        }

        remote_command_t command;
        while (buddy1_take_command(&telemetry, &command)) {
            handle_remote_command(&command);
        }

        // Print sensor data every 200 ms so it doesn't flood the terminal.
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
            if (shock_log_pending) {
                printf("  --> SUDDEN IMPACT DETECTED!\n");
                shock_log_pending = false;
            }
            if (tilt_log_pending) {
                printf("  --> TILT DETECTED!\n");
                tilt_log_pending = false;
            }
        }

        if (now - last_network_print_ms >= 1000U) {
            last_network_print_ms = now;
            printf("Buddy 1 network state: %s\n",
                   network_state_name(buddy1_network_state(&telemetry)));
        }

        // Maintain a 100 Hz sensor and network-poll loop.
        sleep_ms(10);
    }
    
    return 0;
}