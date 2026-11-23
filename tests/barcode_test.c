#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "buddy3.h"
#include "encoder.h"
#include "hardware/gpio.h"
#include "motor_driver.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

/* Replace these placeholders with the actual encoder pins. */
#define LEFT_ENCODER_A_GPIO  16U
#define LEFT_ENCODER_B_GPIO  17U
#define RIGHT_ENCODER_A_GPIO 18U
#define RIGHT_ENCODER_B_GPIO 19U
/* Set each sign so both counts increase when the robot drives forward. */
#define LEFT_ENCODER_SIGN   (-1)
#define RIGHT_ENCODER_SIGN  (1)

/*
 * Scanner settings. Start with the module size, then copy the values from the
 * "CAL suggested" lines this test prints. A value of 0 means "use the default
 * derived from BARCODE_COUNTS_PER_MODULE" (see barcode_scanner.h).
 */
#define BARCODE_COUNTS_PER_MODULE 40U
#define BARCODE_MODULE_TOLERANCE_PERCENT 35U
#define BARCODE_MIN_RUN_COUNTS 15U
#define BARCODE_MAX_GAP_COUNTS 260U
#define BARCODE_WIDE_RATIO_PERCENT 0U

#define BARCODE_SAMPLE_PERIOD_MS 1U
#define BARCODE_MOTOR_PWM 9

/* Print one line per colour change (useful while calibrating). */
#ifndef BARCODE_TEST_PRINT_EDGES
#define BARCODE_TEST_PRINT_EDGES 1
#endif
/*
 * Set to 1 to let the test apply the scanner's suggested settings once enough
 * characters have decoded. The printed CAL lines still show what was used.
 */
#ifndef BARCODE_TEST_AUTO_CALIBRATE
#define BARCODE_TEST_AUTO_CALIBRATE 0
#endif
#define BARCODE_TEST_AUTO_CALIBRATE_CHARS 12U

#define UL(value) ((unsigned long)(value))

static void encoder_gpio_init(uint pin_a, uint pin_b)
{
    gpio_init(pin_a);
    gpio_set_dir(pin_a, GPIO_IN);
    gpio_pull_up(pin_a);

    gpio_init(pin_b);
    gpio_set_dir(pin_b, GPIO_IN);
    gpio_pull_up(pin_b);
}

static int32_t average_encoder_count(
    const encoder_t *left,
    const encoder_t *right)
{
    int64_t left_count = (int64_t)LEFT_ENCODER_SIGN *
                         encoder_get_count(left);
    int64_t right_count = (int64_t)RIGHT_ENCODER_SIGN *
                          encoder_get_count(right);
    int64_t total = left_count + right_count;

    return (int32_t)(total / 2);
}

static void wait_for_serial_monitor(void)
{
    while (!stdio_usb_connected())
    {
        sleep_ms(50U);
    }
}

static void print_edge(const barcode_edge_t *edge)
{
    printf("EDGE #%lu %s delta=%ld width=%lu\n",
           UL(edge->sequence),
           edge->to_black ? "W->B" : "B->W",
           (long)edge->delta,
           UL(edge->width));
}

static void print_event(const barcode_char_info_t *info)
{
    uint8_t i;

    printf("CHAR %s dir=%d", barcode_reject_name(info->reject),
           (int)info->direction);
    if (info->symbol != '\0')
    {
        printf(" symbol='%c'", info->symbol);
    }
    /*
     * Show what the opposite reading direction would give when this one
     * failed, or when a non-'*' character appears outside a frame: that
     * usually means the encoder sign or barcode orientation is backwards.
     */
    if ((info->alt_symbol != '\0') &&
        ((info->reject == BARCODE_REJECT_WRONG_ORIENTATION) ||
         ((info->symbol != '\0') && (info->symbol != '*') &&
          !info->in_frame)))
    {
        printf(" alt='%c'", info->alt_symbol);
    }
    if (info->raw_pattern[0] != '\0')
    {
        printf(" raw=%s used=%s", info->raw_pattern, info->pattern);
    }
    printf(" gap=%lu", UL(info->gap_counts));
    if (info->offending_counts != 0U)
    {
        printf(" offending=%lu", UL(info->offending_counts));
    }
    if (info->reject == BARCODE_REJECT_SEPARATION)
    {
        printf(" narrow_max=%lu wide_min=%lu",
               UL(info->narrow_max), UL(info->wide_min));
    }
    if (info->aborted_frame)
    {
        printf(" FRAME_ABORTED");
    }
    printf(" widths=");
    for (i = 0U; i < info->run_count; i++)
    {
        printf("%s%lu", (i == 0U) ? "" : ",", UL(info->run_counts[i]));
    }
    printf("\n");
}

static void print_calibration(const barcode_scanner_t *scanner)
{
    const barcode_stats_t *stats = barcode_scanner_get_stats(scanner);
    barcode_scanner_config_t suggested;

    printf("CAL decoded=%lu frames_ok=%lu glitch=%lu gap=%lu dir=%lu "
           "separation=%lu no_match=%lu orientation=%lu overflow=%lu "
           "dropped_events=%lu\n",
           UL(stats->rejects[BARCODE_REJECT_NONE]),
           UL(stats->frames_ok),
           UL(stats->rejects[BARCODE_REJECT_GLITCH]),
           UL(stats->rejects[BARCODE_REJECT_GAP]),
           UL(stats->rejects[BARCODE_REJECT_DIRECTION]),
           UL(stats->rejects[BARCODE_REJECT_SEPARATION]),
           UL(stats->rejects[BARCODE_REJECT_NO_MATCH]),
           UL(stats->rejects[BARCODE_REJECT_WRONG_ORIENTATION]),
           UL(stats->rejects[BARCODE_REJECT_OVERFLOW]),
           UL(stats->events_dropped));
    printf("CAL limits in use: min_run=%lu max_gap=%lu wide_ratio=%u%%\n",
           UL(barcode_scanner_min_run_counts(scanner)),
           UL(barcode_scanner_max_gap_counts(scanner)),
           (unsigned)barcode_scanner_wide_ratio_percent(scanner));

    if ((stats->narrow.count > 0U) && (stats->wide.count > 0U))
    {
        printf("CAL decoded runs: narrow min/avg/max=%lu/%lu/%lu "
               "wide min/avg/max=%lu/%lu/%lu\n",
               UL(stats->narrow.min),
               UL(stats->narrow.sum / stats->narrow.count),
               UL(stats->narrow.max),
               UL(stats->wide.min),
               UL(stats->wide.sum / stats->wide.count),
               UL(stats->wide.max));
    }
    if (stats->gap.count > 0U)
    {
        printf("CAL gaps inside frames: min/avg/max=%lu/%lu/%lu\n",
               UL(stats->gap.min),
               UL(stats->gap.sum / stats->gap.count),
               UL(stats->gap.max));
    }
    if (barcode_scanner_suggest_config(scanner, &suggested))
    {
        printf("CAL suggested: #define BARCODE_COUNTS_PER_MODULE %luU, "
               "BARCODE_MIN_RUN_COUNTS %luU, BARCODE_MAX_GAP_COUNTS %luU\n",
               UL(suggested.counts_per_module),
               UL(suggested.min_run_counts),
               UL(suggested.max_gap_counts));
    }
}

/*
 * Prints everything the scanner has recorded since the last call. Returns
 * true if a character event or a result was reported.
 */
static bool report_scanner(
    barcode_scanner_t *scanner,
    uint32_t *last_edge_sequence)
{
    barcode_char_info_t info;
    uint8_t payload[BARCODE_MAX_PAYLOAD];
    uint8_t payload_length;
    bool printed = false;
    bool reported = false;

#if BARCODE_TEST_PRINT_EDGES
    barcode_edge_t edge;

    if (barcode_scanner_get_edge(scanner, &edge) &&
        (edge.sequence != *last_edge_sequence))
    {
        print_edge(&edge);
        *last_edge_sequence = edge.sequence;
        printed = true;
    }
#else
    (void)last_edge_sequence;
#endif

    if (barcode_scanner_take_event(scanner, &info))
    {
        print_event(&info);
        printed = true;
        reported = true;
    }

    if (barcode_scanner_take_result(
            scanner, payload, sizeof(payload), &payload_length))
    {
        uint8_t i;

        printf("BARCODE \"");
        for (i = 0U; i < payload_length; i++)
        {
            printf("%c", (char)payload[i]);
        }
        printf("\" hex=");
        for (i = 0U; i < payload_length; i++)
        {
            printf("%02X", payload[i]);
        }
        printf(" length=%u\n", (unsigned)payload_length);
        printed = true;
        reported = true;
    }

#if BARCODE_TEST_AUTO_CALIBRATE
    {
        static bool applied = false;
        barcode_scanner_config_t suggested;

        if (!applied &&
            (barcode_scanner_get_stats(scanner)->rejects[BARCODE_REJECT_NONE] >=
             BARCODE_TEST_AUTO_CALIBRATE_CHARS) &&
            barcode_scanner_suggest_config(scanner, &suggested) &&
            barcode_scanner_set_config(scanner, &suggested))
        {
            applied = true;
            printf("CAL applied suggested settings.\n");
            printed = true;
            reported = true;
        }
    }
#endif

    if (printed)
    {
        stdio_flush();
    }
    return reported;
}

int main(void)
{
    encoder_t left_encoder;
    encoder_t right_encoder;
    barcode_scanner_t scanner;
    barcode_scanner_config_t config =
    {
        .counts_per_module = BARCODE_COUNTS_PER_MODULE,
        .module_tolerance_percent = BARCODE_MODULE_TOLERANCE_PERCENT,
        .min_run_counts = BARCODE_MIN_RUN_COUNTS,
        .max_gap_counts = BARCODE_MAX_GAP_COUNTS,
        .wide_ratio_percent = BARCODE_WIDE_RATIO_PERCENT
    };
    ir_readings_t readings;
    uint32_t last_status_ms = 0U;
    uint32_t last_edge_sequence = 0U;
    bool calibration_dirty = false;

    stdio_init_all();

    line_following_init();
    encoder_gpio_init(LEFT_ENCODER_A_GPIO, LEFT_ENCODER_B_GPIO);
    encoder_gpio_init(RIGHT_ENCODER_A_GPIO, RIGHT_ENCODER_B_GPIO);
    motor_driver_init();

    /*
     * Keep the motor driver's emergency stop active until the USB serial
     * monitor has connected. This guarantees that opening the monitor is
     * required before the test can start commanding the motors.
     */
    wait_for_serial_monitor();

    printf("\n=== Buddy 3 Code 39 barcode test ===\n");
    printf("Connect the left IR sensor and both wheel encoders.\n");
    printf("Serial monitor connected; starting forward motion.\n");
    stdio_flush();

    motor_clear_emergency_stop();

    encoder_init(
        &left_encoder,
        gpio_get(LEFT_ENCODER_A_GPIO),
        gpio_get(LEFT_ENCODER_B_GPIO));
    encoder_init(
        &right_encoder,
        gpio_get(RIGHT_ENCODER_A_GPIO),
        gpio_get(RIGHT_ENCODER_B_GPIO));

    if (!line_following_read(&readings) ||
        !barcode_scanner_init(
            &scanner,
            &config,
            readings.barcode.black,
            average_encoder_count(&left_encoder, &right_encoder)) ||
        motor_is_emergency_stopped())
    {
        printf("Failed to initialize barcode scanner.\n");
        stdio_flush();
        return 1;
    }

    printf("Barcode scanner ready. Driving forward; keep the barcode under "
           "the left IR sensor.\n");
    printf("Count deltas on EDGE lines should be positive while driving "
           "forward.\n");
    print_calibration(&scanner);
    stdio_flush();

    while (true)
    {
        int32_t count;
        uint32_t now_ms;

        encoder_update(
            &left_encoder,
            gpio_get(LEFT_ENCODER_A_GPIO),
            gpio_get(LEFT_ENCODER_B_GPIO));
        encoder_update(
            &right_encoder,
            gpio_get(RIGHT_ENCODER_A_GPIO),
            gpio_get(RIGHT_ENCODER_B_GPIO));
        count = average_encoder_count(&left_encoder, &right_encoder);

        if (!line_following_read(&readings))
        {
            motor_emergency_stop();
            printf("IR sensor read failed.\n");
            barcode_scanner_reset(&scanner);
        }
        else
        {
            motor_set_speed(MOTOR_1, BARCODE_MOTOR_PWM);
            motor_set_speed(MOTOR_2, -BARCODE_MOTOR_PWM);

            (void)barcode_scanner_update(
                &scanner, readings.barcode.black, count);

            /* All printing happens after the sample has been processed. */
            if (report_scanner(&scanner, &last_edge_sequence))
            {
                calibration_dirty = true;
            }
        }

        now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - last_status_ms) >= 1000U)
        {
            last_status_ms = now_ms;
            printf("IR=%u/%u enc_left=%ld enc_right=%ld distance=%ld\n",
                   (unsigned)readings.barcode.raw,
                   readings.barcode.black ? 1U : 0U,
                   (long)encoder_get_count(&left_encoder),
                   (long)encoder_get_count(&right_encoder),
                   (long)count);
            if (calibration_dirty)
            {
                print_calibration(&scanner);
                calibration_dirty = false;
            }
            stdio_flush();
        }

        sleep_ms(BARCODE_SAMPLE_PERIOD_MS);
    }
}