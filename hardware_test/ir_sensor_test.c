/**
 * @file    ir_sensor_test.c
 * @brief   Standalone bring-up test for the 3 IR sensors (Buddy 3).
 *
 *          Prints raw ADC values, a digital black/white decision and a
 *          simple line-position estimate over USB serial at 10 Hz.
 *
 *          Sensor roles (per project brief):
 *            LEFT   - line detection
 *            CENTER - line detection
 *            RIGHT  - barcode reading
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
#define IR_LEFT_GPIO            (26U)   /* ADC0 - EDIT to match wiring */
#define IR_CENTER_GPIO          (27U)   /* ADC1 - EDIT to match wiring */
#define IR_RIGHT_GPIO           (28U)   /* ADC2 - EDIT to match wiring */

#define ADC_GPIO_BASE           (26U)   /* GPIO26 == ADC input 0       */
#define NUM_SENSORS             (3U)
#define ADC_MAX_COUNTS          (4095U)

/* Raw ADC above this = dark surface (black line). Calibrate by
 * reading values on white and on black, then set midpoint here.
 * Flip IR_BLACK_IS_HIGH if your module outputs LOW on black.        */
#define IR_THRESHOLD            (2000U)
#define IR_BLACK_IS_HIGH        (true)

#define SAMPLE_PERIOD_MS        (100U)
#define USB_SETTLE_MS           (2000U)
#define OVERSAMPLE_COUNT        (8U)

/* Line position: -1 = line to the left, 0 = centred, +1 = to the right */
#define POS_LEFT                (-1)
#define POS_CENTER              (0)
#define POS_RIGHT               (1)
#define POS_LOST                (99)

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */
typedef enum
{
    IR_LEFT = 0,
    IR_CENTER,
    IR_RIGHT
} ir_index_t;

/* ------------------------------------------------------------------ */
/* File-scope data                                                     */
/* ------------------------------------------------------------------ */
static const uint8_t s_ir_gpio[NUM_SENSORS] =
{
    IR_LEFT_GPIO,
    IR_CENTER_GPIO,
    IR_RIGHT_GPIO
};

static const char * const s_ir_name[NUM_SENSORS] =
{
    "LEFT", "CENTER", "RIGHT"
};

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

static bool ir_is_black(uint16_t raw)
{
    bool black;

    if (IR_BLACK_IS_HIGH)
    {
        black = (raw > IR_THRESHOLD);
    }
    else
    {
        black = (raw < IR_THRESHOLD);
    }

    return black;
}

/* Crude position estimate from the two line sensors only. */
static int8_t ir_line_position(bool left_black, bool center_black)
{
    int8_t pos;

    if (left_black && center_black)
    {
        pos = POS_LEFT;     /* line between left and centre */
    }
    else if (left_black)
    {
        pos = POS_LEFT;
    }
    else if (center_black)
    {
        pos = POS_CENTER;
    }
    else
    {
        pos = POS_LOST;
    }

    return pos;
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */
int main(void)
{
    uint16_t raw[NUM_SENSORS];
    bool     black[NUM_SENSORS];
    uint8_t  i;
    int8_t   pos;

    stdio_init_all();
    sleep_ms(USB_SETTLE_MS);   /* give the USB serial terminal time to attach */

    ir_init();

    printf("\r\n=== IR sensor test (Buddy 3) ===\r\n");
    printf("Threshold=%u, black_is_high=%d\r\n",
           (unsigned)IR_THRESHOLD, (int)IR_BLACK_IS_HIGH);
    printf("Move each sensor over white then black and note the raw values.\r\n\r\n");

    for (;;)
    {
        for (i = 0U; i < NUM_SENSORS; i++)
        {
            raw[i]   = ir_read_raw((ir_index_t)i);
            black[i] = ir_is_black(raw[i]);
        }

        pos = ir_line_position(black[IR_LEFT], black[IR_CENTER]);

        for (i = 0U; i < NUM_SENSORS; i++)
        {
            printf("%s: %4u (%s)  ",
                   s_ir_name[i],
                   (unsigned)raw[i],
                   black[i] ? "BLK" : "WHT");
        }

        if (pos == POS_LOST)
        {
            printf("| line: LOST\r\n");
        }
        else
        {
            printf("| line: %d\r\n", (int)pos);
        }

        sleep_ms(SAMPLE_PERIOD_MS);
    }

    return 0;
}
