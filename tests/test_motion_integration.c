
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "motion_control.h"
#include "motor_driver.h"

/*
 * Desktop-only test.
 *
 * Uses tests/mocks/pico/time.h and
 * simulated motor hardware.
 */

static uint64_t fake_time_us = 0ULL;

static int16_t left_pwm = 0;
static int16_t right_pwm = 0;

static bool emergency_stopped = true;

typedef struct
{
    int16_t left_command;
    int16_t right_command;
    bool should_fail;
    unsigned int calls;
} fake_feedback_t;

uint64_t time_us_64(void)
{
    return fake_time_us;
}

void motor_driver_init(void)
{
    left_pwm = 0;
    right_pwm = 0;
    emergency_stopped = true;
}

void motor_set_speed(motor_id_t motor, int16_t speed)
{
    if (emergency_stopped)
    {
        return;
    }

    if (motor == MOTOR_1)
    {
        left_pwm = speed;
    }
    else if (motor == MOTOR_2)
    {
        right_pwm = speed;
    }
}

void motor_brake_all(void)
{
    left_pwm = 0;
    right_pwm = 0;
}

void motor_emergency_stop(void)
{
    emergency_stopped = true;
    motor_brake_all();
}

bool motor_is_emergency_stopped(void)
{
    return emergency_stopped;
}

void motor_clear_emergency_stop(void)
{
    motor_brake_all();
    emergency_stopped = false;
}

static bool fake_feedback(
    void *context,
    motion_direction_t direction,
    uint64_t now_us,
    bool starting,
    int16_t *motor_1,
    int16_t *motor_2)
{
    (void)direction;
    (void)now_us;
    (void)starting;

    fake_feedback_t *feedback =
        (fake_feedback_t *)context;

    feedback->calls++;

    if (feedback->should_fail)
    {
        return false;
    }

    *motor_1 = feedback->left_command;
    *motor_2 = feedback->right_command;

    return true;
}

static void advance_ms(uint32_t milliseconds)
{
    fake_time_us +=
        (uint64_t)milliseconds * 1000ULL;
}

static void reset_test(void)
{
    fake_time_us = 0ULL;
    motion_init();

    assert(!motion_is_armed());
    assert(emergency_stopped);
}

static void test_feedback_output(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        20, -24, false, 0U
    };

    assert(motion_set_feedback(
        fake_feedback, &feedback));

    assert(motion_arm());
    assert(!motion_move_forward(5.0f));

    motion_config_t config =
    {
        1000U, 1000U, 1000U, 1000U, 20U
    };

    assert(motion_configure(&config));
    assert(motion_move_forward(5.0f));

    advance_ms(60U);
    motion_update();

    assert(motion_get_status() == MOTION_RUNNING);
    assert(left_pwm == 20);
    assert(right_pwm == -24);
    assert(feedback.calls == 1U);

    advance_ms(100U);
    motion_update();

    assert(feedback.calls == 2U);

    printf("PASS: Feedback PWM applied\n");
}

static void test_invalid_pwm_polarity(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        -20, 20, false, 0U
    };

    assert(motion_set_feedback(
        fake_feedback, &feedback));

    assert(motion_arm());
    assert(motion_turn_left(90.0f));

    advance_ms(60U);
    motion_update();

    assert(motion_get_status() == MOTION_ERROR);
    assert(!motion_is_armed());
    assert(emergency_stopped);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Invalid motor polarity rejected\n");
}

static void test_feedback_failure(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        -20, -20, false, 0U
    };

    assert(motion_set_feedback(
        fake_feedback, &feedback));

    assert(motion_arm());
    assert(motion_turn_left(90.0f));

    advance_ms(60U);
    motion_update();

    assert(motion_get_status() == MOTION_RUNNING);

    feedback.should_fail = true;

    advance_ms(100U);
    motion_update();

    assert(motion_get_status() == MOTION_ERROR);
    assert(emergency_stopped);
    assert(!motion_is_armed());
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Failed feedback triggers stop\n");
}

static void test_control_timeout(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        -20, -20, false, 0U
    };

    assert(motion_set_feedback(
        fake_feedback, &feedback));

    assert(motion_arm());
    assert(motion_turn_left(90.0f));

    advance_ms(60U);
    motion_update();

    advance_ms(300U);
    motion_update();

    assert(motion_get_status() == MOTION_ERROR);
    assert(emergency_stopped);
    assert(!motion_is_armed());

    printf("PASS: Delayed control update rejected\n");
}

static void test_feedback_completion(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        20, 20, false, 0U
    };

    assert(motion_set_feedback(
        fake_feedback, &feedback));

    assert(motion_arm());
    assert(motion_turn_right(90.0f));

    advance_ms(60U);
    motion_update();

    for (int i = 0; i < 10; ++i)
    {
        advance_ms(100U);
        motion_update();
    }

    assert(motion_get_status() == MOTION_COMPLETE);
    assert(left_pwm == 0);
    assert(right_pwm == 0);
    assert(motion_is_armed());

    printf("PASS: Feedback movement completes\n");
}

static void test_external_emergency(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        20, 20, false, 0U
    };

    assert(motion_set_feedback(
        fake_feedback, &feedback));

    assert(motion_arm());
    assert(motion_turn_right(90.0f));

    advance_ms(60U);
    motion_update();

    motor_emergency_stop();
    motion_update();

    assert(motion_get_status() == MOTION_ERROR);
    assert(!motion_is_armed());
    assert(emergency_stopped);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: External emergency remains latched\n");
}

static void test_feedback_change_while_armed(void)
{
    reset_test();

    fake_feedback_t feedback =
    {
        20, -20, false, 0U
    };

    assert(motion_arm());

    assert(!motion_set_feedback(
        fake_feedback, &feedback));

    printf("PASS: Armed feedback changes rejected\n");
}

int main(void)
{
    test_feedback_output();
    test_invalid_pwm_polarity();
    test_feedback_failure();
    test_control_timeout();
    test_feedback_completion();
    test_external_emergency();
    test_feedback_change_while_armed();

    printf(
        "\nAll 7 motion integration tests passed!\n");

    return 0;
}
