#include <stdio.h>
#include <stdint.h>

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
 * Calibrate this value by moving one barcode module past the sensor and
 * measuring the encoder-count change. It is intentionally not inferred from
 * wheel diameter because gearbox and quadrature resolution vary by hardware.
 */
#define BARCODE_COUNTS_PER_MODULE 35U
#define BARCODE_MODULE_TOLERANCE_PERCENT 35U
#define BARCODE_SAMPLE_PERIOD_MS 1U
#define BARCODE_MOTOR_PWM 10

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

int main(void)
{
    encoder_t left_encoder;
    encoder_t right_encoder;
    barcode_scanner_t scanner;
    barcode_scanner_config_t config =
    {
        .counts_per_module = BARCODE_COUNTS_PER_MODULE,
        .module_tolerance_percent = BARCODE_MODULE_TOLERANCE_PERCENT
    };
    ir_readings_t readings;
    uint8_t payload[BARCODE_MAX_PAYLOAD];
    uint8_t payload_length;
    uint32_t last_status_ms = 0U;
    bool previous_barcode_black;
    int32_t previous_barcode_count;

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

    previous_barcode_black = readings.barcode.black;
    previous_barcode_count =
        average_encoder_count(&left_encoder, &right_encoder);

    printf("Barcode scanner ready. Driving forward; keep the barcode under "
           "the left IR sensor.\n");
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

            if (readings.barcode.black != previous_barcode_black)
            {
                int32_t transition_delta = count - previous_barcode_count;
                int32_t transition_width = (transition_delta < 0) ?
                                           -transition_delta :
                                           transition_delta;

                printf("IR transition %u->%u: count_delta=%ld "
                       "width=%ld counts\n",
                       previous_barcode_black ? 1U : 0U,
                       readings.barcode.black ? 1U : 0U,
                       (long)transition_delta,
                       (long)transition_width);
                stdio_flush();
                previous_barcode_black = readings.barcode.black;
                previous_barcode_count = count;
            }

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
            stdio_flush();
        }

        sleep_ms(BARCODE_SAMPLE_PERIOD_MS);
    }
}
