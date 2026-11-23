#include "imu_task.h"

#include "imu_driver.h"
#include "imu_processor.h"
#include "imu_types.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"

#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define IMU_SAMPLE_PERIOD_MS 50U

static uint32_t s_imu_sequence;
static uint32_t s_fault_sequence;
static float s_distance_m;
static bool s_i2c_fault_reported;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void receive_latest_odometry(void)
{
    wheel_odometry_message_t odometry;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_imu_odometry, &odometry, TMO_POL);
        if (received == (INT)sizeof(odometry)) {
            if (odometry.valid != 0U) {
                s_distance_m = (float)odometry.distance_mm / 1000.0f;
            }
        } else if (received < 0 && received != E_TMOUT) {
            printf("imu: odometry receive failed (%d)\n", (int)received);
            return;
        }
    } while (received > 0);
}

static void publish_fault(uint32_t timestamp_ms, uint16_t code)
{
    fault_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_FAULT;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_fault_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.source = VEHICLE_FAULT_SENSOR;
    message.recoverable = 1U;
    message.code = code;

    result = tk_snd_mbf(g_mbf_faults, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("imu: fault message send failed (%d)\n", (int)result);
    }
}

static void publish_estimate(const imu_state_t *state, bool valid,
                             uint32_t timestamp_ms)
{
    imu_estimate_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_IMU_ESTIMATE;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_imu_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.roll_mdeg = (int32_t)(state->current_roll_deg * 1000.0f);
    message.pitch_mdeg = (int32_t)(state->current_pitch_deg * 1000.0f);
    message.validity = valid ? 1U : 0U;

    result = tk_snd_mbf(g_mbf_imu_estimates, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("imu: estimate message send failed (%d)\n", (int)result);
    }
}

void imu_task(INT stacd, void *exinf)
{
    imu_state_t state;
    accel_data_t accel;
    uint32_t timestamp_ms;

    (void)stacd;
    (void)exinf;
    memset(&state, 0, sizeof(state));
    s_imu_sequence = 0U;
    s_fault_sequence = 0U;
    s_distance_m = 0.0f;
    s_i2c_fault_reported = false;

    imu_driver_init();
    imu_processor_init(&state);

    for (;;) {
        timestamp_ms = now_ms();
        receive_latest_odometry();
        memset(&accel, 0, sizeof(accel));
        imu_driver_read_accel(&accel);

        if (!accel.i2c_ok) {
            if (!s_i2c_fault_reported) {
                publish_fault(timestamp_ms, VEHICLE_FAULT_CODE_IMU_I2C);
                s_i2c_fault_reported = true;
            }
            publish_estimate(&state, false, timestamp_ms);
        } else {
            s_i2c_fault_reported = false;
            imu_processor_update(&state, &accel, s_distance_m);
            publish_estimate(&state, true, timestamp_ms);
        }

        (void)tk_dly_tsk(IMU_SAMPLE_PERIOD_MS);
    }
}
