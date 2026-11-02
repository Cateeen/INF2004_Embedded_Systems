
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "wheel_feedback_pid.h"

static wheel_feedback_t left_wheel = {0};
static wheel_feedback_t right_wheel = {0};
static wheel_feedback_pid_t controller = {0};

static void forward_cycle(wheel_feedback_t *wheel)
{
    wheel_feedback_update_encoder(wheel, 0U, 1U);
    wheel_feedback_update_encoder(wheel, 1U, 1U);
    wheel_feedback_update_encoder(wheel, 1U, 0U);
    wheel_feedback_update_encoder(wheel, 0U, 0U);
}

static void reverse_cycle(wheel_feedback_t *wheel)
{
    wheel_feedback_update_encoder(wheel, 1U, 0U);
    wheel_feedback_update_encoder(wheel, 1U, 1U);
    wheel_feedback_update_encoder(wheel, 0U, 1U);
    wheel_feedback_update_encoder(wheel, 0U, 0U);
}

static void setup(void)
{
    wheel_odometry_config_t odometry =
    {
        400U,
        20.0f
    };

    wheel_speed_config_t speed =
    {
        2.0f, 0.0f, 0.0f,
        10.0f, 20U, 60U
    };

    assert(wheel_feedback_init(
        &left_wheel, &odometry, 0U, 0U, 0ULL));

    assert(wheel_feedback_init(
        &right_wheel, &odometry, 0U, 0U, 0ULL));

    assert(wheel_feedback_pid_init(
        &controller,
        &left_wheel,
        &right_wheel,
        &speed,
        true,
        true,
        0ULL));
}

static void test_forward_equal_speed(void)
{
    setup();

    for (int i = 0; i < 5; ++i)
    {
        forward_cycle(&left_wheel);
        forward_cycle(&right_wheel);
    }

    wheel_feedback_pid_output_t output = {0};

    assert(wheel_feedback_pid_update(
        &controller,
        MOTION_FORWARD,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == 20);
    assert(output.motor_2_pwm == -20);

    printf("PASS: Forward equal-speed feedback\n");
}

static void test_forward_speed_correction(void)
{
    setup();

    for (int i = 0; i < 5; ++i)
    {
        forward_cycle(&left_wheel);
    }

    for (int i = 0; i < 4; ++i)
    {
        forward_cycle(&right_wheel);
    }

    wheel_feedback_pid_output_t output = {0};

    assert(wheel_feedback_pid_update(
        &controller,
        MOTION_FORWARD,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == 20);
    assert(output.motor_2_pwm == -24);

    printf("PASS: Slower right wheel corrected\n");
}

static void test_backward_direction(void)
{
    setup();

    for (int i = 0; i < 5; ++i)
    {
        reverse_cycle(&left_wheel);
        reverse_cycle(&right_wheel);
    }

    wheel_feedback_pid_output_t output = {0};

    assert(wheel_feedback_pid_update(
        &controller,
        MOTION_BACKWARD,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == -20);
    assert(output.motor_2_pwm == 20);

    printf("PASS: Backward signed PWM\n");
}

static void test_left_turn_direction(void)
{
    setup();

    for (int i = 0; i < 5; ++i)
    {
        reverse_cycle(&left_wheel);
        forward_cycle(&right_wheel);
    }

    wheel_feedback_pid_output_t output = {0};

    assert(wheel_feedback_pid_update(
        &controller,
        MOTION_LEFT,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == -20);
    assert(output.motor_2_pwm == -20);

    printf("PASS: Left turn signed PWM\n");
}

static void test_right_turn_direction(void)
{
    setup();

    for (int i = 0; i < 5; ++i)
    {
        forward_cycle(&left_wheel);
        reverse_cycle(&right_wheel);
    }

    wheel_feedback_pid_output_t output = {0};

    assert(wheel_feedback_pid_update(
        &controller,
        MOTION_RIGHT,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == 20);
    assert(output.motor_2_pwm == 20);

    printf("PASS: Right turn signed PWM\n");
}

static void test_stale_feedback(void)
{
    setup();

    wheel_feedback_pid_output_t output = {20, -20};

    assert(!wheel_feedback_pid_update(
        &controller,
        MOTION_FORWARD,
        500000ULL,
        &output));

    assert(output.motor_1_pwm == 0);
    assert(output.motor_2_pwm == 0);

    printf("PASS: Stale feedback rejected\n");
}

static void test_wrong_encoder_direction(void)
{
    setup();

    reverse_cycle(&left_wheel);
    forward_cycle(&right_wheel);

    wheel_feedback_pid_output_t output = {20, -20};

    assert(!wheel_feedback_pid_update(
        &controller,
        MOTION_FORWARD,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == 0);
    assert(output.motor_2_pwm == 0);

    printf("PASS: Unexpected encoder direction rejected\n");
}

static void test_invalid_direction(void)
{
    setup();

    wheel_feedback_pid_output_t output = {20, -20};

    assert(!wheel_feedback_pid_update(
        &controller,
        (motion_direction_t)99,
        100000ULL,
        &output));

    assert(output.motor_1_pwm == 0);
    assert(output.motor_2_pwm == 0);

    printf("PASS: Invalid motion direction rejected\n");
}

int main(void)
{
    test_forward_equal_speed();
    test_forward_speed_correction();
    test_backward_direction();
    test_left_turn_direction();
    test_right_turn_direction();
    test_stale_feedback();
    test_wrong_encoder_direction();
    test_invalid_direction();

    printf(
        "\nAll 8 feedback-to-PID tests passed!\n");

    return 0;
}
