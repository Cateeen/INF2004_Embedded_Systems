#include "encoder.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <stdio.h>

#define LEFT_ENCODER_A_GPIO   16U
#define LEFT_ENCODER_B_GPIO   17U
#define RIGHT_ENCODER_A_GPIO  18U
#define RIGHT_ENCODER_B_GPIO  19U
#define REPORT_PERIOD_MS      500U

static void encoder_gpio_init(uint gpio)
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

int main(void)
{
    encoder_t left;
    encoder_t right;
    uint32_t last_report_ms;

    stdio_init_all();
    sleep_ms(2000U);

    encoder_gpio_init(LEFT_ENCODER_A_GPIO);
    encoder_gpio_init(LEFT_ENCODER_B_GPIO);
    encoder_gpio_init(RIGHT_ENCODER_A_GPIO);
    encoder_gpio_init(RIGHT_ENCODER_B_GPIO);

    encoder_init(&left,
                 (uint8_t)gpio_get(LEFT_ENCODER_A_GPIO),
                 (uint8_t)gpio_get(LEFT_ENCODER_B_GPIO));
    encoder_init(&right,
                 (uint8_t)gpio_get(RIGHT_ENCODER_A_GPIO),
                 (uint8_t)gpio_get(RIGHT_ENCODER_B_GPIO));

    printf("\nEncoder calibration test\n");
    printf("Pins: left A/B=%u/%u, right A/B=%u/%u\n",
           LEFT_ENCODER_A_GPIO, LEFT_ENCODER_B_GPIO,
           RIGHT_ENCODER_A_GPIO, RIGHT_ENCODER_B_GPIO);
    printf("Rotate each wheel exactly one full revolution and record counts.\n");
    printf("This test does not enable or command motors.\n\n");
    stdio_flush();

    last_report_ms = to_ms_since_boot(get_absolute_time());
    for (;;) {
        encoder_update(&left,
                       (uint8_t)gpio_get(LEFT_ENCODER_A_GPIO),
                       (uint8_t)gpio_get(LEFT_ENCODER_B_GPIO));
        encoder_update(&right,
                       (uint8_t)gpio_get(RIGHT_ENCODER_A_GPIO),
                       (uint8_t)gpio_get(RIGHT_ENCODER_B_GPIO));

        {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if ((uint32_t)(now_ms - last_report_ms) >= REPORT_PERIOD_MS) {
                last_report_ms = now_ms;
                printf("left=%ld right=%ld average=%ld\n",
                       (long)encoder_get_count(&left),
                       (long)encoder_get_count(&right),
                       (long)((encoder_get_count(&left) +
                               encoder_get_count(&right)) / 2));
                stdio_flush();
            }
        }
        sleep_us(500U);
    }
}
