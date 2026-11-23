#include "vehicle_odometry_task.h"

#include "vehicle_messages.h"
#include "vehicle_system.h"
#include "wheel_feedback.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define ODOMETRY_PERIOD_MS          50U
#define ODOMETRY_LEFT_A_GPIO        16U
#define ODOMETRY_LEFT_B_GPIO        17U
#define ODOMETRY_RIGHT_A_GPIO       18U
#define ODOMETRY_RIGHT_B_GPIO       19U
#ifndef PICOCAR_ODOMETRY_COUNTS_PER_REV
#define PICOCAR_ODOMETRY_COUNTS_PER_REV 0U
#endif

#ifndef PICOCAR_ODOMETRY_WHEEL_CIRCUMFERENCE_CM
#define PICOCAR_ODOMETRY_WHEEL_CIRCUMFERENCE_CM 20.0f
#endif

static wheel_feedback_t s_left;
static wheel_feedback_t s_right;
static bool s_initialised;
static uint32_t s_sequence;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void configure_encoder_gpio(uint gpio)
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

void vehicle_odometry_left_edge(uint8_t channel_a, uint8_t channel_b)
{
    if (s_initialised) {
        wheel_feedback_update_encoder(&s_left, channel_a, channel_b);
    }
}

void vehicle_odometry_right_edge(uint8_t channel_a, uint8_t channel_b)
{
    if (s_initialised) {
        wheel_feedback_update_encoder(&s_right, channel_a, channel_b);
    }
}

static bool initialise_feedback(void)
{
    wheel_odometry_config_t configuration = {
        PICOCAR_ODOMETRY_COUNTS_PER_REV,
        PICOCAR_ODOMETRY_WHEEL_CIRCUMFERENCE_CM
    };
    uint64_t timestamp_us = (uint64_t)time_us_64();

    if (configuration.counts_per_revolution == 0U) {
        printf("odometry: waiting for counts-per-revolution calibration\n");
        return false;
    }

    configure_encoder_gpio(ODOMETRY_LEFT_A_GPIO);
    configure_encoder_gpio(ODOMETRY_LEFT_B_GPIO);
    configure_encoder_gpio(ODOMETRY_RIGHT_A_GPIO);
    configure_encoder_gpio(ODOMETRY_RIGHT_B_GPIO);

    if (!wheel_feedback_init(
               &s_left, &configuration,
               (uint8_t)gpio_get(ODOMETRY_LEFT_A_GPIO),
               (uint8_t)gpio_get(ODOMETRY_LEFT_B_GPIO),
               timestamp_us) ||
        !wheel_feedback_init(
               &s_right, &configuration,
               (uint8_t)gpio_get(ODOMETRY_RIGHT_A_GPIO),
               (uint8_t)gpio_get(ODOMETRY_RIGHT_B_GPIO),
               timestamp_us)) {
        return false;
    }

    gpio_set_irq_enabled(ODOMETRY_LEFT_A_GPIO,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ODOMETRY_LEFT_B_GPIO,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ODOMETRY_RIGHT_A_GPIO,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ODOMETRY_RIGHT_B_GPIO,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    return true;
}

static bool publish_odometry(ID buffer, const wheel_odometry_message_t *message)
{
    INT result = tk_snd_mbf(buffer, message, (INT)sizeof(*message), TMO_POL);

    if (result != E_OK) {
        printf("odometry: message send failed (%d)\n", (int)result);
        return false;
    }
    return true;
}

static void publish_sample(uint32_t timestamp_ms)
{
    wheel_odometry_message_t message;
    int32_t left_count;
    int32_t right_count;
    float distance_cm;

    if (!wheel_feedback_sample(&s_left, (uint64_t)time_us_64()) ||
        !wheel_feedback_sample(&s_right, (uint64_t)time_us_64())) {
        return;
    }

    left_count = encoder_get_count(&s_left.encoder);
    right_count = encoder_get_count(&s_right.encoder);
    distance_cm = (wheel_feedback_get_distance(&s_left) +
                   wheel_feedback_get_distance(&s_right)) / 2.0f;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.left_count = left_count;
    message.right_count = right_count;
    message.distance_mm = (int32_t)(distance_cm * 10.0f);
    message.valid = 1U;

    (void)publish_odometry(g_mbf_line_odometry, &message);
    (void)publish_odometry(g_mbf_imu_odometry, &message);
}

void vehicle_odometry_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    memset(&s_left, 0, sizeof(s_left));
    memset(&s_right, 0, sizeof(s_right));
    s_initialised = false;
    s_sequence = 0U;

    s_initialised = initialise_feedback();
    for (;;) {
        if (s_initialised) {
            publish_sample(now_ms());
        }
        (void)tk_dly_tsk(ODOMETRY_PERIOD_MS);
    }
}
