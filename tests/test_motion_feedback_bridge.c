
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "motion_feedback_bridge.h"

static wheel_feedback_t left_wheel = {0};
static wheel_feedback_t right_wheel = {0};

static wheel_feedback_pid_t feedback_pid = {0};
static motion_feedback_bridge_t bridge = {0};

static void forward_cycle(wheel_feedback_t *wheel)
{
    wheel_feedback_update_encoder(wheel, 0U, 1U);
    wheel_feedback_update_encoder(wheel, 1U, 1U);
    wheel_feedback_update_encoder(wheel, 1U, 0U);
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
        2.0f,
        0.0f,
        0.0f,
        10.0f,
        20U,
        60U
    };

    assert(wheel_feedback_init(
        &left_wheel,
        &odometry,
        0U,
        0U,
        0ULL));

    assert(wheel_feedback_init(
        &right_wheel,
        &odometry,
        0U,
        0U,
        0ULL));

    assert(wheel_feedback_pid_init(
        &feedback_pid,
        &left_wheel,
        &right_wheel,
        &speed,
        true,
        true,
        0ULL));

    assert(motion_feedback_bridge_init(
        &bridge,
        &feedback_pid));
}

static void test_starting_commands(void)
{
    setup();

    int16_t motor_1 = 0;
    int16_t motor_2 = 0;

    assert(motion_feedback_bridge_update(
        &bridge,
        MOTION_FORWARD,
        0ULL,
        true,
        &motor_1,
        &motor_2));

    assert(motor_1 == 20);
    assert(motor_2 == -20);

    printf("PASS: Initial base PWM\n");
}

static void test_pid_correction(void)
{
    setup();

    int16_t motor_1 = 0;
    int16_t motor_2 = 0;

    assert(motion_feedback_bridge_update(
        &bridge,
        MOTION_FORWARD,
        0ULL,
        true,
        &motor_1,
        &motor_2));

    for (int i = 0; i < 5; ++i)
    {
        forward_cycle(&left_wheel);
    }

    for (int i = 0; i < 4; ++i)
    {
        forward_cycle(&right_wheel);
    }

    assert(motion_feedback_bridge_update(
        &bridge,
        MOTION_FORWARD,
        100000ULL,
        false,
        &motor_1,
        &motor_2));

    assert(motor_1 == 20);
    assert(motor_2 == -24);

    printf("PASS: Encoder-based PID correction\n");
}

static void test_no_encoder_timeout(void)
{
    setup();

    int16_t motor_1 = 0;
    int16_t motor_2 = 0;

    assert(motion_feedback_bridge_update(
        &bridge,
        MOTION_FORWARD,
        0ULL,
        true,
        &motor_1,
        &motor_2));

    /*
     * No encoder transitions occur.
     * The bridge should eventually reject
     * continued movement.
     */
    for (uint64_t now = 100000ULL;
         now < 500000ULL;
         now += 100000ULL)
    {
        assert(motion_feedback_bridge_update(
            &bridge,
            MOTION_FORWARD,
            now,
            false,
            &motor_1,
            &motor_2));
    }

    assert(!motion_feedback_bridge_update(
        &bridge,
        MOTION_FORWARD,
        500000ULL,
        false,
        &motor_1,
        &motor_2));

    assert(motor_1 == 0);
    assert(motor_2 == 0);

    printf("PASS: Missing encoder feedback rejected\n");
}

static void test_invalid_direction(void)
{
    setup();

    int16_t motor_1 = 0;
    int16_t motor_2 = 0;

    assert(!motion_feedback_bridge_update(
        &bridge,
        (motion_direction_t)99,
        0ULL,
        true,
        &motor_1,
        &motor_2));

    printf("PASS: Invalid direction rejected\n");
}

int main(void)
{
    test_starting_commands();
    test_pid_correction();
    test_no_encoder_timeout();
    test_invalid_direction();

    printf(
        "\nAll 4 motion feedback bridge tests passed!\n");

    return 0;
}
