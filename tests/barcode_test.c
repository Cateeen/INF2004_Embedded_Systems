#include <stdio.h>
#include <stdint.h>

#include "buddy3.h"
#include "encoder.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

/* Replace these placeholders with the encoder pins used by the barcode wheel. */
#define BARCODE_ENCODER_A_GPIO 12U
#define BARCODE_ENCODER_B_GPIO 13U

/*
 * Calibrate this value by moving one barcode module past the sensor and
 * measuring the encoder-count change. It is intentionally not inferred from
 * wheel diameter because gearbox and quadrature resolution vary by hardware.
 */
#define BARCODE_COUNTS_PER_MODULE 5U
#define BARCODE_MODULE_TOLERANCE_PERCENT 35U
#define BARCODE_SAMPLE_PERIOD_MS 1U

static void encoder_gpio_init(void)
{
    gpio_init(BARCODE_ENCODER_A_GPIO);
    gpio_set_dir(BARCODE_ENCODER_A_GPIO, GPIO_IN);
    gpio_pull_up(BARCODE_ENCODER_A_GPIO);

    gpio_init(BARCODE_ENCODER_B_GPIO);
    gpio_set_dir(BARCODE_ENCODER_B_GPIO, GPIO_IN);
    gpio_pull_up(BARCODE_ENCODER_B_GPIO);
}

int main(void)
{
    encoder_t encoder;
    barcode_scanner_t scanner;
    barcode_scanner_config_t config =
    {
        .counts_per_module = BARCODE_COUNTS_PER_MODULE,
        .module_tolerance_percent = BARCODE_MODULE_TOLERANCE_PERCENT
    };
    ir_readings_t readings;
    uint8_t payload[BARCODE_MAX_PAYLOAD];
    uint8_t payload_length;

    stdio_init_all();
    sleep_ms(2000U);
    line_following_init();
    encoder_gpio_init();

    encoder_init(
        &encoder,
        gpio_get(BARCODE_ENCODER_A_GPIO),
        gpio_get(BARCODE_ENCODER_B_GPIO));

    if (!line_following_read(&readings) ||
        !barcode_scanner_init(
            &scanner,
            &config,
            readings.barcode.black,
            encoder_get_count(&encoder)))
    {
        printf("Failed to initialize barcode scanner.\n");
        return 1;
    }

    printf("Barcode scanner ready. Left IR sensor is used for barcode data.\n");

    while (true)
    {
        int32_t count;

        encoder_update(
            &encoder,
            gpio_get(BARCODE_ENCODER_A_GPIO),
            gpio_get(BARCODE_ENCODER_B_GPIO));

        if (!line_following_read(&readings))
        {
            printf("IR sensor read failed.\n");
            barcode_scanner_reset(&scanner);
        }
        else
        {
            count = encoder_get_count(&encoder);
            (void)barcode_scanner_update(
                &scanner, readings.barcode.black, count);

            if (barcode_scanner_take_result(
                    &scanner, payload, &payload_length))
            {
                uint8_t i;

                printf("Barcode: ");
                for (i = 0U; i < payload_length; i++)
                {
                    printf("%02X", payload[i]);
                }
                printf(" (length=%u)\n", (unsigned)payload_length);
            }
        }

        sleep_ms(BARCODE_SAMPLE_PERIOD_MS);
    }
}
