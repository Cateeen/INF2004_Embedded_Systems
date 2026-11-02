
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "wheel_feedback.h"

#define TOLERANCE 0.001f

static bool near(float actual, float expected)
{
    return fabsf(actual - expected) < TOLERANCE;
}

static wheel_odometry_config_t test_config(void)
{
    wheel_odometry_config_t config =
    {
        400U,  /* Example encoder counts per wheel revolution */
        20.0f  /* Example wheel circumference in cm */
    };

    return config;
}

/*
 * Simulate one positive quadrature cycle:
 * 00 -> 01 -> 11 -> 10 -> 00
 *
 * Expected result: +4 encoder counts.
 */
static void forward_cycle(wheel_feedback_t *wheel)
{
    wheel_feedback_update_encoder(wheel, 0U, 1U);
    wheel_feedback_update_encoder(wheel, 1U, 1U);
    wheel_feedback_update_encoder(wheel, 1U, 0U);
    wheel_feedback_update_encoder(wheel, 0U, 0U);
}

/*
 * Simulate one negative quadrature cycle:
 * 00 -> 10 -> 11 -> 01 -> 00
 *
 * Expected result: -4 encoder counts.
 */
static void reverse_cycle(wheel_feedback_t *wheel)
{
    wheel_feedback_update_encoder(wheel, 1U, 0U);
    wheel_feedback_update_encoder(wheel, 1U, 1U);
    wheel_feedback_update_encoder(wheel, 0U, 1U);
    wheel_feedback_update_encoder(wheel, 0U, 0U);
}

static void test_initialization(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    assert(wheel.initialized);
    assert(near(wheel_feedback_get_distance(&wheel), 0.0f));

    printf("PASS: Wheel feedback initialization\n");
}

static void test_forward_distance(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    /* 10 cycles = 40 counts = 2 cm. */
    for (int i = 0; i < 10; ++i)
    {
        forward_cycle(&wheel);
    }

    assert(wheel_feedback_sample(&wheel, 100000ULL));

    wheel_odometry_result_t result = {0};

    assert(wheel_feedback_get_result(&wheel, &result));
    assert(near(result.distance_cm, 2.0f));
    assert(near(result.speed_cm_per_second, 20.0f));
    assert(near(result.rpm, 60.0f));

    printf("PASS: Forward encoder-to-odometry\n");
}

static void test_reverse_distance(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    for (int i = 0; i < 10; ++i)
    {
        reverse_cycle(&wheel);
    }

    assert(wheel_feedback_sample(&wheel, 100000ULL));

    wheel_odometry_result_t result = {0};

    assert(wheel_feedback_get_result(&wheel, &result));
    assert(near(result.distance_cm, -2.0f));
    assert(near(result.speed_cm_per_second, -20.0f));
    assert(near(result.rpm, -60.0f));

    printf("PASS: Reverse encoder-to-odometry\n");
}

static void test_accumulated_distance(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    for (int i = 0; i < 5; ++i)
    {
        forward_cycle(&wheel);
    }

    assert(wheel_feedback_sample(&wheel, 100000ULL));
    assert(near(wheel_feedback_get_distance(&wheel), 1.0f));

    for (int i = 0; i < 5; ++i)
    {
        forward_cycle(&wheel);
    }

    assert(wheel_feedback_sample(&wheel, 200000ULL));
    assert(near(wheel_feedback_get_distance(&wheel), 2.0f));

    printf("PASS: Accumulated wheel distance\n");
}

static void test_stationary_wheel(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    assert(wheel_feedback_sample(&wheel, 100000ULL));

    wheel_odometry_result_t result = {0};

    assert(wheel_feedback_get_result(&wheel, &result));
    assert(near(result.distance_cm, 0.0f));
    assert(near(result.speed_cm_per_second, 0.0f));
    assert(near(result.rpm, 0.0f));

    printf("PASS: Stationary wheel\n");
}

static void test_reset(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    forward_cycle(&wheel);
    assert(wheel_feedback_sample(&wheel, 100000ULL));

    wheel_feedback_reset(&wheel, 100000ULL);

    assert(near(wheel_feedback_get_distance(&wheel), 0.0f));
    assert(encoder_get_count(&wheel.encoder) == 0);

    wheel_odometry_result_t result = {0};
    assert(wheel_feedback_get_result(&wheel, &result));
    assert(near(result.rpm, 0.0f));

    printf("PASS: Wheel feedback reset\n");
}

static void test_invalid_sample_time(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 100000ULL));

    assert(!wheel_feedback_sample(&wheel, 100000ULL));
    assert(!wheel_feedback_sample(&wheel, 99999ULL));

    printf("PASS: Invalid sample time rejected\n");
}

static void test_invalid_initialization(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(!wheel_feedback_init(
        NULL, &config, 0U, 0U, 0ULL));

    config.counts_per_revolution = 0U;

    assert(!wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    printf("PASS: Invalid initialization rejected\n");
}

static void test_null_parameters(void)
{
    wheel_feedback_t wheel = {0};
    wheel_odometry_config_t config = test_config();

    assert(wheel_feedback_init(
        &wheel, &config, 0U, 0U, 0ULL));

    assert(!wheel_feedback_sample(NULL, 100000ULL));
    assert(!wheel_feedback_get_result(&wheel, NULL));
    assert(near(wheel_feedback_get_distance(NULL), 0.0f));

    printf("PASS: Null parameters handled\n");
}

int main(void)
{
    test_initialization();
    test_forward_distance();
    test_reverse_distance();
    test_accumulated_distance();
    test_stationary_wheel();
    test_reset();
    test_invalid_sample_time();
    test_invalid_initialization();
    test_null_parameters();

    printf("\nAll 9 wheel feedback tests passed!\n");

    return 0;
}
