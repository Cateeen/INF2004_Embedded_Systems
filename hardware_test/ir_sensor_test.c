/**
 * @file    ir_sensor_test.c
 * @brief   Standalone bring-up test for the 3 IR sensors (Buddy 3).
 *
 *          Prints raw ADC values, a digital black/white decision and a
 *          simple line-position estimate over USB serial at 10 Hz.
 *
 *          Sensor roles (per project brief):
 *            LEFT   - barcode scanning
 *            CENTER - line detection
 *            RIGHT  - line detection
 *
 * NOTE:   Pin numbers below are PLACEHOLDERS. Check the Robo Pico
 *         schematic / your wiring and edit IR_*_GPIO before running.
 *         The Pico ADC only exists on GPIO 26, 27 and 28.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */
#define IR_BARCODE_GPIO         (28U)   /* ADC2 - EDIT to match wiring */
#define IR_CENTER_GPIO          (27U)   /* ADC1 - EDIT to match wiring */
#define IR_RIGHT_GPIO           (26U)   /* ADC0 - EDIT to match wiring */

#define ADC_GPIO_BASE           (26U)   /* GPIO26 == ADC input 0       */
#define NUM_SENSORS             (3U)
#define ADC_MAX_COUNTS          (4095U)

/* Raw ADC above this = dark surface (black line). Calibrate by
 * reading values on white and on black, then set midpoint here.
 * Flip IR_BLACK_IS_HIGH if your module outputs LOW on black.        */
#define IR_BARCODE_THRESHOLD    (1000U)
#define IR_CENTER_THRESHOLD     (1000U)
#define IR_RIGHT_THRESHOLD      (1000U)
#define IR_BLACK_IS_HIGH        (true)

#define SAMPLE_PERIOD_MS        (100U)
#define USB_SETTLE_MS           (2000U)
#define OVERSAMPLE_COUNT        (8U)
#define CALIBRATION_WINDOW_MS   (1000U)

/* Line position: 0 = centred, +1 = to the right */
#define POS_CENTER              (0)
#define POS_RIGHT               (1)
#define POS_LOST                (99)

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */
typedef enum
{
    IR_BARCODE = 0,
    IR_CENTER,
    IR_RIGHT
} ir_index_t;

/* ------------------------------------------------------------------ */
/* File-scope data                                                     */
/* ------------------------------------------------------------------ */
static const uint8_t s_ir_gpio[NUM_SENSORS] =
{
    IR_BARCODE_GPIO,
    IR_CENTER_GPIO,
    IR_RIGHT_GPIO
};

static const char * const s_ir_name[NUM_SENSORS] =
{
    "BARCODE", "CENTER", "RIGHT"
};

static const uint16_t s_ir_threshold[NUM_SENSORS] =
{
    IR_BARCODE_THRESHOLD,
    IR_CENTER_THRESHOLD,
    IR_RIGHT_THRESHOLD
};

typedef struct
{
    uint16_t minimum;
    uint16_t maximum;
    uint32_t sum;
    uint32_t samples;
} calibration_stats_t;

/* ------------------------------------------------------------------ */
/* Private functions                                                   */
/* ------------------------------------------------------------------ */
static void ir_init(void)
{
    uint8_t i;

    adc_init();
    for (i = 0U; i < NUM_SENSORS; i++)
    {
        adc_gpio_init(s_ir_gpio[i]);
    }
}

/* Averaged raw reading (0..4095) for one sensor. */
static uint16_t ir_read_raw(ir_index_t sensor)
{
    uint32_t sum = 0U;
    uint8_t  n;

    adc_select_input((uint)(s_ir_gpio[sensor] - ADC_GPIO_BASE));

    for (n = 0U; n < OVERSAMPLE_COUNT; n++)
    {
        sum += adc_read();
    }

    return (uint16_t)(sum / OVERSAMPLE_COUNT);
}

static bool ir_is_black(ir_index_t sensor, uint16_t raw)
{
    bool black;

    if (IR_BLACK_IS_HIGH)
    {
        black = (raw > s_ir_threshold[sensor]);
    }
    else
    {
        black = (raw < s_ir_threshold[sensor]);
    }

    return black;
}

/* Position estimate from the centre and right line sensors. */
static int8_t ir_line_position(bool center_black, bool right_black)
{
    int8_t pos;

    if (center_black)
    {
        pos = POS_CENTER;
    }
    else if (right_black)
    {
        pos = POS_RIGHT;
    }
    else
    {
        pos = POS_LOST;
    }

    return pos;
}

static void calibration_stats_update(
    calibration_stats_t *stats,
    uint16_t raw)
{
    if (stats->samples == 0U)
    {
        stats->minimum = raw;
        stats->maximum = raw;
    }
    else
    {
        if (raw < stats->minimum)
        {
            stats->minimum = raw;
        }

        if (raw > stats->maximum)
        {
            stats->maximum = raw;
        }
    }

    stats->sum += raw;
    stats->samples++;
}

static void calibration_stats_reset(calibration_stats_t *stats)
{
    stats->minimum = 0U;
    stats->maximum = 0U;
    stats->sum = 0U;
    stats->samples = 0U;
}

static void print_calibration_stats(
    ir_index_t sensor,
    const calibration_stats_t *stats)
{
    uint32_t average = 0U;

    if (stats->samples != 0U)
    {
        average = stats->sum / stats->samples;
    }

    printf(
        "Calibration %s: min=%u max=%u average=%u samples=%lu threshold=%u\r\n",
        s_ir_name[sensor],
        (unsigned)stats->minimum,
        (unsigned)stats->maximum,
        (unsigned)average,
        (unsigned long)stats->samples,
        (unsigned)s_ir_threshold[sensor]);
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */
int main(void)
{
    uint16_t raw[NUM_SENSORS];
    bool     black[NUM_SENSORS];
    calibration_stats_t calibration[NUM_SENSORS];
    uint8_t  i;
    int8_t   pos;
    uint32_t last_calibration_print_ms;

    stdio_init_all();
    sleep_ms(USB_SETTLE_MS);   /* give the USB serial terminal time to attach */

    ir_init();
    for (i = 0U; i < NUM_SENSORS; i++)
    {
        calibration_stats_reset(&calibration[i]);
    }

    printf("\r\n=== IR sensor test (Buddy 3) ===\r\n");
    printf("Thresholds: barcode=%u center=%u right=%u, black_is_high=%d\r\n",
           (unsigned)IR_BARCODE_THRESHOLD,
           (unsigned)IR_CENTER_THRESHOLD,
           (unsigned)IR_RIGHT_THRESHOLD,
           (int)IR_BLACK_IS_HIGH);
    printf("Move each sensor over white then black and use the calibration statistics.\r\n");
    printf("The barcode sensor is reported separately; centre and right determine line position.\r\n\r\n");

    last_calibration_print_ms =
        to_ms_since_boot(get_absolute_time());

    for (;;)
    {
        for (i = 0U; i < NUM_SENSORS; i++)
        {
            raw[i]   = ir_read_raw((ir_index_t)i);
            black[i] = ir_is_black((ir_index_t)i, raw[i]);
            calibration_stats_update(&calibration[i], raw[i]);
        }

        pos = ir_line_position(black[IR_CENTER], black[IR_RIGHT]);

        for (i = 0U; i < NUM_SENSORS; i++)
        {
            printf("%s: %4u (%s)  ",
                   s_ir_name[i],
                   (unsigned)raw[i],
                   black[i] ? "BLK" : "WHT");
        }

        printf("| barcode: %s ", black[IR_BARCODE] ? "BLK" : "WHT");

        if (pos == POS_LOST)
        {
            printf("| line: LOST\r\n");
        }
        else
        {
            printf("| line: %d\r\n", (int)pos);
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - last_calibration_print_ms) >=
            CALIBRATION_WINDOW_MS)
        {
            for (i = 0U; i < NUM_SENSORS; i++)
            {
                print_calibration_stats(
                    (ir_index_t)i,
                    &calibration[i]);
                calibration_stats_reset(&calibration[i]);
            }

            last_calibration_print_ms = now_ms;
        }

        sleep_ms(SAMPLE_PERIOD_MS);
    }

    return 0;
}
