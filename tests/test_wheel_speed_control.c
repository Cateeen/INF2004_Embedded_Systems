
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "wheel_speed_control.h"

static wheel_speed_config_t test_config(void)
{
    wheel_speed_config_t config =
    {
        2.0f,   /* Kp */
        0.0f,   /* Ki */
        0.0f,   /* Kd */
        10.0f,  /* Target: 10 cm/s */
        20U,    /* Base PWM */
        60U     /* Maximum PWM */
    };

    return config;
}

static void test_initialization(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(controller.initialized);

    printf("PASS: PID initialization\n");
}

static void test_equal_speeds(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {0};

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(wheel_speed_control_update(
        &controller, 10.0f, 10.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 20U);
    assert(output.right_pwm_percent == 20U);

    printf("PASS: Equal wheel speeds\n");
}

static void test_slower_right_wheel(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {0};

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(wheel_speed_control_update(
        &controller, 10.0f, 8.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 20U);
    assert(output.right_pwm_percent == 24U);

    printf("PASS: Slower right wheel correction\n");
}

static void test_slower_left_wheel(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {0};

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(wheel_speed_control_update(
        &controller, 7.0f, 10.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 26U);
    assert(output.right_pwm_percent == 20U);

    printf("PASS: Slower left wheel correction\n");
}

static void test_faster_wheel(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {0};

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(wheel_speed_control_update(
        &controller, 15.0f, 10.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 10U);
    assert(output.right_pwm_percent == 20U);

    printf("PASS: Faster wheel PWM reduction\n");
}

static void test_pwm_upper_limit(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {0};

    config.kp = 100.0f;

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(wheel_speed_control_update(
        &controller, 0.0f, 0.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 60U);
    assert(output.right_pwm_percent == 60U);

    printf("PASS: Maximum PWM limit\n");
}

static void test_pwm_lower_limit(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {0};

    config.kp = 100.0f;

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(wheel_speed_control_update(
        &controller, 100.0f, 100.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 0U);
    assert(output.right_pwm_percent == 0U);

    printf("PASS: Minimum PWM limit\n");
}

static void test_invalid_feedback(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {5U, 5U};

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(!wheel_speed_control_update(
        &controller, NAN, 10.0f, 0.1f, &output));

    assert(output.left_pwm_percent == 0U);
    assert(output.right_pwm_percent == 0U);

    assert(!wheel_speed_control_update(
        &controller, -1.0f, 10.0f, 0.1f, &output));

    printf("PASS: Invalid feedback rejected\n");
}

static void test_stale_update(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();
    wheel_speed_output_t output = {5U, 5U};

    assert(wheel_speed_control_init(
        &controller, &config));

    assert(!wheel_speed_control_update(
        &controller, 10.0f, 10.0f, 0.5f, &output));

    assert(output.left_pwm_percent == 0U);
    assert(output.right_pwm_percent == 0U);

    printf("PASS: Stale control interval rejected\n");
}

static void test_invalid_config(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();

    config.max_pwm_percent = 101U;

    assert(!wheel_speed_control_init(
        &controller, &config));

    config = test_config();
    config.base_pwm_percent = 61U;

    assert(!wheel_speed_control_init(
        &controller, &config));

    config = test_config();
    config.target_speed_cm_s = NAN;

    assert(!wheel_speed_control_init(
        &controller, &config));

    printf("PASS: Invalid PID configuration\n");
}

static void test_reset(void)
{
    wheel_speed_control_t controller = {0};
    wheel_speed_config_t config = test_config();

    assert(wheel_speed_control_init(
        &controller, &config));

    wheel_speed_control_reset(&controller);

    assert(controller.left_pid.first_update);
    assert(controller.right_pid.first_update);

    printf("PASS: PID reset\n");
}

int main(void)
{
    test_initialization();
    test_equal_speeds();
    test_slower_right_wheel();
    test_slower_left_wheel();
    test_faster_wheel();
    test_pwm_upper_limit();
    test_pwm_lower_limit();
    test_invalid_feedback();
    test_stale_update();
    test_invalid_config();
    test_reset();

    printf(
        "\nAll 11 wheel-speed control tests passed!\n");

    return 0;
}
