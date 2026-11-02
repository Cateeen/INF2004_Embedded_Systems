
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "wheel_odometry.h"

#define TOLERANCE 0.001f

static bool approximately_equal(float actual, float expected)
{
    return fabsf(actual - expected) < TOLERANCE;
}

static wheel_odometry_config_t test_config(void)
{
    wheel_odometry_config_t config =
    {
        400U,   /* Example: 400 counts per wheel revolution */
        20.0f   /* Example: 20 cm wheel circumference */
    };

    return config;
}

static void test_forward_distance(void)
{
    wheel_odometry_config_t config = test_config();
    float distance = 0.0f;

    assert(wheel_odometry_distance(&config, 200, &distance));
    assert(approximately_equal(distance, 10.0f));

    printf("PASS: Forward distance calculation\n");
}

static void test_reverse_distance(void)
{
    wheel_odometry_config_t config = test_config();
    float distance = 0.0f;

    assert(wheel_odometry_distance(&config, -200, &distance));
    assert(approximately_equal(distance, -10.0f));

    printf("PASS: Reverse distance calculation\n");
}

static void test_full_revolution(void)
{
    wheel_odometry_config_t config = test_config();
    float distance = 0.0f;

    assert(wheel_odometry_distance(&config, 400, &distance));
    assert(approximately_equal(distance, 20.0f));

    printf("PASS: Full wheel revolution\n");
}

static void test_rpm(void)
{
    wheel_odometry_config_t config = test_config();
    wheel_odometry_result_t result = {0};

    /*
     * 200 counts in 0.5 seconds:
     * - Half a revolution
     * - 60 RPM
     * - 20 cm/s
     * - 10 cm distance
     */
    assert(wheel_odometry_calculate(
        &config, 200, 0.5f, &result));

    assert(approximately_equal(result.distance_cm, 10.0f));
    assert(approximately_equal(result.rpm, 60.0f));
    assert(approximately_equal(result.speed_cm_per_second, 20.0f));

    printf("PASS: RPM and wheel speed\n");
}

static void test_reverse_rpm(void)
{
    wheel_odometry_config_t config = test_config();
    wheel_odometry_result_t result = {0};

    assert(wheel_odometry_calculate(
        &config, -200, 0.5f, &result));

    assert(approximately_equal(result.distance_cm, -10.0f));
    assert(approximately_equal(result.rpm, -60.0f));
    assert(approximately_equal(result.speed_cm_per_second, -20.0f));

    printf("PASS: Reverse RPM and wheel speed\n");
}

static void test_zero_movement(void)
{
    wheel_odometry_config_t config = test_config();
    wheel_odometry_result_t result = {0};

    assert(wheel_odometry_calculate(
        &config, 0, 0.5f, &result));

    assert(approximately_equal(result.distance_cm, 0.0f));
    assert(approximately_equal(result.rpm, 0.0f));
    assert(approximately_equal(result.speed_cm_per_second, 0.0f));

    printf("PASS: Zero wheel movement\n");
}

static void test_invalid_calibration(void)
{
    wheel_odometry_config_t config = test_config();
    float distance = 0.0f;

    config.counts_per_revolution = 0U;

    assert(!wheel_odometry_distance(&config, 200, &distance));

    config = test_config();
    config.wheel_circumference_cm = 0.0f;

    assert(!wheel_odometry_distance(&config, 200, &distance));

    config = test_config();
    config.wheel_circumference_cm = NAN;

    assert(!wheel_odometry_distance(&config, 200, &distance));

    printf("PASS: Invalid calibration rejected\n");
}

static void test_invalid_time(void)
{
    wheel_odometry_config_t config = test_config();
    wheel_odometry_result_t result = {0};

    assert(!wheel_odometry_calculate(
        &config, 200, 0.0f, &result));

    assert(!wheel_odometry_calculate(
        &config, 200, -1.0f, &result));

    assert(!wheel_odometry_calculate(
        &config, 200, NAN, &result));

    printf("PASS: Invalid time rejected\n");
}

static void test_null_parameters(void)
{
    wheel_odometry_config_t config = test_config();
    wheel_odometry_result_t result = {0};
    float distance = 0.0f;

    assert(!wheel_odometry_distance(
        NULL, 200, &distance));

    assert(!wheel_odometry_distance(
        &config, 200, NULL));

    assert(!wheel_odometry_calculate(
        NULL, 200, 0.5f, &result));

    assert(!wheel_odometry_calculate(
        &config, 200, 0.5f, NULL));

    printf("PASS: Null parameters rejected\n");
}

int main(void)
{
    test_forward_distance();
    test_reverse_distance();
    test_full_revolution();
    test_rpm();
    test_reverse_rpm();
    test_zero_movement();
    test_invalid_calibration();
    test_invalid_time();
    test_null_parameters();

    printf("\nAll 9 wheel odometry tests passed!\n");

    return 0;
}
