#include "motor_driver.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <stddef.h>
#include <stdint.h>

/* Cytron Robo Pico motor GPIO assignments. */
#define MOTOR_1_A_PIN           8U
#define MOTOR_1_B_PIN           9U
#define MOTOR_2_A_PIN           10U
#define MOTOR_2_B_PIN           11U

/* PWM frequency must not exceed 20 kHz. */
#define MOTOR_PWM_FREQUENCY_HZ  20000U

/*
 * Minimum braking interval before reversing direction.
 *
 * This does not guarantee that the wheel has physically
 * stopped. The higher-level controller must manage
 * deceleration and verify motion using encoder feedback.
 */
#define MOTOR_REVERSE_BRAKE_US  50000ULL

typedef struct
{
    int8_t last_direction;
    bool braking;
    uint64_t brake_started_us;
} motor_state_t;

static bool motor_initialized = false;
static bool emergency_stopped = true;

static uint16_t pwm_period_ticks = 0U;

static motor_state_t motor_states[2] = {0};

/*
 * Write both PWM channels belonging to one motor.
 *
 * GP8/GP9 share one PWM slice.
 * GP10/GP11 share another PWM slice.
 */
static void motor_write_pair(uint pin_a,
                             uint16_t duty_a,
                             uint16_t duty_b)
{
    uint slice = pwm_gpio_to_slice_num(pin_a);

    pwm_set_both_levels(slice, duty_a, duty_b);
}

/* Identify the GPIO pins belonging to a motor. */
static void motor_get_pins(motor_id_t motor,
                           uint *pin_a,
                           uint *pin_b)
{
    if (motor == MOTOR_1)
    {
        *pin_a = MOTOR_1_A_PIN;
        *pin_b = MOTOR_1_B_PIN;
    }
    else
    {
        *pin_a = MOTOR_2_A_PIN;
        *pin_b = MOTOR_2_B_PIN;
    }
}

/* Apply braking while preserving direction history. */
static void motor_brake(motor_id_t motor)
{
    uint pin_a;
    uint pin_b;

    motor_get_pins(motor, &pin_a, &pin_b);

    motor_write_pair(pin_a, 0U, 0U);

    motor_state_t *state = &motor_states[(uint)motor];

    if (!state->braking)
    {
        state->braking = true;
        state->brake_started_us = time_us_64();
    }
}

/* Initialize motor driver hardware. */
void motor_driver_init(void)
{
    const uint pins[] =
    {
        MOTOR_1_A_PIN,
        MOTOR_1_B_PIN,
        MOTOR_2_A_PIN,
        MOTOR_2_B_PIN
    };

    emergency_stopped = true;
    motor_initialized = false;
    pwm_period_ticks = 0U;

    /*
     * First configure every pin as a LOW GPIO output.
     * The Robo Pico uses LOW/LOW for braking.
     */
    for (size_t i = 0U; i < 4U; ++i)
    {
        gpio_init(pins[i]);
        gpio_put(pins[i], false);
        gpio_set_dir(pins[i], GPIO_OUT);
    }

    uint32_t clock_hz = clock_get_hz(clk_sys);

    /*
     * Round up the clock ticks so the resulting PWM
     * frequency does not exceed 20 kHz.
     *
     * Divider = 1.
     * PWM frequency = clock_hz / period_ticks.
     */
    uint32_t period_ticks =
        (clock_hz + MOTOR_PWM_FREQUENCY_HZ - 1U) /
        MOTOR_PWM_FREQUENCY_HZ;

    /*
     * Keep the period within the 16-bit compare range.
     * On the standard Pico W clock this is valid.
     */
    if ((period_ticks < 2U) ||
        (period_ticks > UINT16_MAX))
    {
        return;
    }

    pwm_period_ticks = (uint16_t)period_ticks;

    uint slice_1 = pwm_gpio_to_slice_num(MOTOR_1_A_PIN);
    uint slice_2 = pwm_gpio_to_slice_num(MOTOR_2_A_PIN);

    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 1.0f);

    pwm_config_set_wrap(
        &config,
        (uint16_t)(pwm_period_ticks - 1U)
    );

    /*
     * Configure the two PWM slices without
     * starting their counters.
     */
    pwm_init(slice_1, &config, false);
    pwm_init(slice_2, &config, false);

    motor_write_pair(MOTOR_1_A_PIN, 0U, 0U);
    motor_write_pair(MOTOR_2_A_PIN, 0U, 0U);

    /* Connect GPIO pins to PWM hardware. */
    for (size_t i = 0U; i < 4U; ++i)
    {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
    }

    /* Start PWM with both motors braked. */
    pwm_set_enabled(slice_1, true);
    pwm_set_enabled(slice_2, true);

    /* Reset internal motor state. */
    for (size_t i = 0U; i < 2U; ++i)
    {
        motor_states[i].last_direction = 0;
        motor_states[i].braking = true;
        motor_states[i].brake_started_us = time_us_64();
    }

    motor_initialized = true;

    /*
     * Emergency stop remains active until explicitly
     * cleared by the authorized controller.
     */
}

/* Command motor speed between -100% and +100%. */
void motor_set_speed(motor_id_t motor,
                     int16_t speed_percent)
{
    if ((!motor_initialized) || emergency_stopped)
    {
        return;
    }

    if ((motor != MOTOR_1) && (motor != MOTOR_2))
    {
        return;
    }

    /* Clamp input speed. */
    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    else if (speed_percent < -100)
    {
        speed_percent = -100;
    }

    if (speed_percent == 0)
    {
        motor_brake(motor);
        return;
    }

    uint pin_a;
    uint pin_b;

    motor_get_pins(motor, &pin_a, &pin_b);

    motor_state_t *state = &motor_states[(uint)motor];

    int8_t requested_direction =
        (speed_percent > 0) ? 1 : -1;

    /*
     * Prevent immediate reversal.
     *
     * A first request in the opposite direction
     * applies the brakes and returns.
     *
     * Repeated requests remain blocked until
     * the minimum braking interval has elapsed.
     */
    if ((state->last_direction != 0) &&
        (requested_direction != state->last_direction))
    {
        if (!state->braking)
        {
            motor_brake(motor);
            return;
        }

        uint64_t elapsed_us =
            time_us_64() - state->brake_started_us;

        if (elapsed_us < MOTOR_REVERSE_BRAKE_US)
        {
            return;
        }
    }

    uint16_t magnitude = (uint16_t)(
        (speed_percent < 0) ?
        -speed_percent : speed_percent
    );

    /*
     * PWM compare uses a period of wrap + 1 ticks.
     *
     * A 100% command maps to the full period.
     */
    uint16_t duty = (uint16_t)(
        ((uint32_t)magnitude * pwm_period_ticks) / 100U
    );

    if (requested_direction > 0)
    {
        /* Forward: A = PWM, B = LOW. */
        motor_write_pair(pin_a, duty, 0U);
    }
    else
    {
        /* Reverse: A = LOW, B = PWM. */
        motor_write_pair(pin_a, 0U, duty);
    }

    state->last_direction = requested_direction;
    state->braking = false;
}

/* Brake both motors. */
void motor_brake_all(void)
{
    if (!motor_initialized)
    {
        return;
    }

    motor_brake(MOTOR_1);
    motor_brake(MOTOR_2);
}

/* Activate the emergency stop. */
void motor_emergency_stop(void)
{
    emergency_stopped = true;

    motor_brake_all();
}

/* Check whether emergency stop is active. */
bool motor_is_emergency_stopped(void)
{
    return emergency_stopped;
}

/* Clear emergency stop without restarting motors. */
void motor_clear_emergency_stop(void)
{
    if (!motor_initialized)
    {
        return;
    }

    motor_brake_all();

    emergency_stopped = false;
}