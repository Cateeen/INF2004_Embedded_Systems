
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "motion_control.h"
#include "motor_driver.h"

/* Test-only simulated Pico time and motor hardware. */

static uint64_t fake_time_us = 0ULL;
static int16_t left_pwm = 0;
static int16_t right_pwm = 0;
static bool emergency_stopped = true;

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

static void advance_ms(uint32_t milliseconds)
{
    fake_time_us +=
        (uint64_t)milliseconds * 1000ULL;
}

static void reset_test(void)
{
    fake_time_us = 0ULL;
    motion_init();

    assert(motion_get_status() == MOTION_IDLE);
    assert(!motion_is_armed());
    assert(!motion_is_busy());
    assert(emergency_stopped);
    assert(left_pwm == 0);
    assert(right_pwm == 0);
}

static motion_config_t test_config(void)
{
    motion_config_t config =
    {
        1000U, /* 10 cm/s forward */
        1000U, /* 10 cm/s backward */
        1000U, /* 90-degree left turn */
        1000U, /* 90-degree right turn */
        20U
    };

    return config;
}

static void begin_motion(void)
{
    advance_ms(60U);
    motion_update();

    assert(motion_get_status() == MOTION_RUNNING);
}

static void test_safe_startup(void)
{
    reset_test();

    assert(!motion_move_forward(5.0f));
    assert(!motion_turn_left(90.0f));
    assert(emergency_stopped);

    printf("PASS: Safe startup\n");
}

static void test_explicit_arming(void)
{
    reset_test();

    assert(motion_arm());
    assert(motion_is_armed());
    assert(!emergency_stopped);

    printf("PASS: Explicit arming\n");
}

static void test_uncalibrated_distance(void)
{
    reset_test();
    assert(motion_arm());

    assert(!motion_move_forward(5.0f));
    assert(!motion_move_backward(5.0f));
    assert(!motion_is_busy());

    printf("PASS: Uncalibrated distance rejected\n");
}

static void test_left_turn(void)
{
    reset_test();
    assert(motion_arm());

    assert(motion_turn_left(90.0f));
    assert(motion_get_status() == MOTION_WAITING);

    assert(!motion_turn_right(90.0f));

    advance_ms(59U);
    motion_update();

    assert(left_pwm == 0);
    assert(right_pwm == 0);

    advance_ms(1U);
    motion_update();

    assert(left_pwm == -20);
    assert(right_pwm == -20);

    advance_ms(999U);
    motion_update();

    assert(motion_get_status() == MOTION_RUNNING);

    advance_ms(1U);
    motion_update();

    assert(motion_get_status() == MOTION_COMPLETE);
    assert(motion_is_armed());
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Left turn and completion\n");
}

static void test_right_turn(void)
{
    reset_test();
    assert(motion_arm());

    assert(motion_turn_right(90.0f));
    begin_motion();

    assert(left_pwm == 20);
    assert(right_pwm == 20);

    advance_ms(1000U);
    motion_update();

    assert(motion_get_status() == MOTION_COMPLETE);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Right turn and completion\n");
}

static void test_forward_distance(void)
{
    reset_test();

    motion_config_t config = test_config();

    assert(motion_configure(&config));
    assert(motion_arm());
    assert(motion_move_forward(5.0f));

    begin_motion();

    assert(left_pwm == 20);
    assert(right_pwm == -20);

    advance_ms(500U);
    motion_update();

    assert(motion_get_status() == MOTION_COMPLETE);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Forward distance calculation\n");
}

static void test_backward_distance(void)
{
    reset_test();

    motion_config_t config = test_config();

    assert(motion_configure(&config));
    assert(motion_arm());
    assert(motion_move_backward(5.0f));

    begin_motion();

    assert(left_pwm == -20);
    assert(right_pwm == 20);

    advance_ms(500U);
    motion_update();

    assert(motion_get_status() == MOTION_COMPLETE);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Backward distance calculation\n");
}

static void test_normal_stop(void)
{
    reset_test();
    assert(motion_arm());

    assert(motion_turn_left(90.0f));
    begin_motion();

    motion_stop();

    assert(motion_get_status() == MOTION_STOPPED);
    assert(!motion_is_busy());
    assert(motion_is_armed());
    assert(!emergency_stopped);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    /* A new request is permitted after normal stop. */
    assert(motion_turn_right(90.0f));

    printf("PASS: Normal stop\n");
}

static void test_emergency_stop(void)
{
    reset_test();
    assert(motion_arm());

    assert(motion_turn_left(90.0f));
    begin_motion();

    motion_emergency_stop();

    assert(!motion_is_armed());
    assert(emergency_stopped);
    assert(!motion_is_busy());
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    assert(!motion_turn_right(90.0f));

    advance_ms(2000U);
    motion_update();

    assert(emergency_stopped);
    assert(left_pwm == 0);
    assert(right_pwm == 0);

    printf("PASS: Emergency stop remains latched\n");
}

static void test_external_emergency_stop(void)
{
    reset_test();
    assert(motion_arm());

    assert(motion_turn_right(90.0f));
    begin_motion();

    /* Simulate a safety task stopping the motors. */
    motor_emergency_stop();

    motion_update();

    assert(!motion_is_armed());
    assert(!motion_is_busy());
    assert(emergency_stopped);
    assert(motion_get_status() == MOTION_ERROR);

    assert(!motion_turn_left(90.0f));

    printf("PASS: External emergency stop detected\n");
}

static void test_rearming(void)
{
    reset_test();
    assert(motion_arm());

    motion_emergency_stop();

    assert(!motion_is_armed());
    assert(!motion_turn_left(90.0f));

    /* Simulate safety approval to rearm. */
    assert(motion_arm());

    assert(motion_is_armed());
    assert(!emergency_stopped);
    assert(motion_turn_left(90.0f));

    printf("PASS: Explicit rearming\n");
}

static void test_invalid_requests(void)
{
    reset_test();
    assert(motion_arm());

    assert(!motion_turn_left(0.0f));
    assert(!motion_turn_right(-90.0f));
    assert(!motion_turn_left(200.0f));
    assert(!motion_turn_left(NAN));
    assert(!motion_start(
        (motion_direction_t)99, 90.0f));

    assert(!motion_is_busy());

    printf("PASS: Invalid movement requests\n");
}

static void test_invalid_configuration(void)
{
    reset_test();

    motion_config_t config = test_config();

    assert(!motion_configure(NULL));

    config.pwm_percent = 101U;
    assert(!motion_configure(&config));

    config = test_config();
    config.left_turn_ms_per_90 = 0U;
    assert(!motion_configure(&config));

    printf("PASS: Invalid calibration rejected\n");
}

int main(void)
{
    test_safe_startup();
    test_explicit_arming();
    test_uncalibrated_distance();
    test_left_turn();
    test_right_turn();
    test_forward_distance();
    test_backward_distance();
    test_normal_stop();
    test_emergency_stop();
    test_external_emergency_stop();
    test_rearming();
    test_invalid_requests();
    test_invalid_configuration();

    printf(
        "\nAll 13 Buddy 2 motion-control tests passed!\n"
    );

    return 0;
}
