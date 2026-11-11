#include "line_barcode_task.h"

#include "buddy3.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"

#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define LINE_BARCODE_PERIOD_MS       20U
#define BARCODE_COUNTS_PER_MODULE    4U
#define BARCODE_TOLERANCE_PERCENT    25U

static uint32_t s_line_sequence;
static uint32_t s_barcode_sequence;
static int32_t s_encoder_count;
static bool s_encoder_valid;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void receive_latest_odometry(void)
{
    wheel_odometry_message_t odometry;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_line_odometry, &odometry,
                              TMO_POL);
        if (received == (INT)sizeof(odometry) && odometry.valid != 0U) {
            s_encoder_count = (odometry.left_count + odometry.right_count) / 2;
            s_encoder_valid = true;
        } else if (received < 0 && received != E_TMOUT) {
            printf("line/barcode: odometry receive failed (%d)\n",
                   (int)received);
            return;
        }
    } while (received > 0);
}

static void publish_line_observation(const ir_readings_t *readings,
                                     uint32_t timestamp_ms)
{
    line_observation_message_t message;
    uint8_t position;
    INT result;

    memset(&message, 0, sizeof(message));
    position = line_following_position(readings);
    message.header.type = VEHICLE_MESSAGE_LINE_OBSERVATION;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_line_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.lateral_error = (position == POS_LEFT) ? -1 :
                            (position == POS_RIGHT) ? 1 : 0;
    message.confidence = (position == POS_LOST) ? 0U : 1000U;
    message.line_detected = (position != POS_LOST) ? 1U : 0U;
    message.intersection_detected = (position == POS_JUNCTION) ? 1U : 0U;

    result = tk_snd_mbf(g_mbf_line_observations, &message,
                        (INT)sizeof(message), TMO_POL);
    if (result != E_OK) {
        printf("line/barcode: line message send failed (%d)\n", (int)result);
    }
}

static void publish_barcode_event(const uint8_t *payload, uint8_t length,
                                  uint32_t timestamp_ms)
{
    barcode_event_message_t message;
    INT result;

    if ((payload == NULL) || (length > VEHICLE_BARCODE_PAYLOAD_SIZE)) {
        return;
    }

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_BARCODE_EVENT;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_barcode_sequence;
    message.header.timestamp_ms = timestamp_ms;
    memcpy(message.payload, payload, length);
    message.length = length;
    message.valid = 1U;

    result = tk_snd_mbf(g_mbf_barcode_events, &message,
                        (INT)sizeof(message), TMO_POL);
    if (result != E_OK) {
        printf("line/barcode: barcode message send failed (%d)\n",
               (int)result);
    }
}

void line_barcode_task(INT stacd, void *exinf)
{
    barcode_scanner_t scanner;
    barcode_scanner_config_t configuration = {
        BARCODE_COUNTS_PER_MODULE,
        BARCODE_TOLERANCE_PERCENT,
        0U,
        0U,
        0U
    };
    ir_readings_t readings;
    uint8_t payload[VEHICLE_BARCODE_PAYLOAD_SIZE];
    uint8_t payload_length;
    uint32_t timestamp_ms;

    (void)stacd;
    (void)exinf;
    memset(&scanner, 0, sizeof(scanner));
    memset(&readings, 0, sizeof(readings));
    s_line_sequence = 0U;
    s_barcode_sequence = 0U;
    s_encoder_count = 0;
    s_encoder_valid = false;

    line_following_init();
    if (!barcode_scanner_init(&scanner, &configuration, false, 0)) {
        printf("line/barcode: scanner initialization failed\n");
        for (;;) {
            (void)tk_dly_tsk(LINE_BARCODE_PERIOD_MS);
        }
    }

    for (;;) {
        timestamp_ms = now_ms();
        receive_latest_odometry();

        if (!line_following_read(&readings)) {
            printf("line/barcode: sensor read failed\n");
        } else {
            publish_line_observation(&readings, timestamp_ms);

            if (s_encoder_valid &&
                barcode_scanner_update(&scanner, readings.barcode.black,
                                       s_encoder_count) &&
                barcode_scanner_take_result(&scanner, payload,
                                            (uint8_t)sizeof(payload),
                                            &payload_length)) {
                publish_barcode_event(payload, payload_length, timestamp_ms);
            }
        }

        (void)tk_dly_tsk(LINE_BARCODE_PERIOD_MS);
    }
}
