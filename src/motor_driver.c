
#include "motor_driver.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include <stddef.h>

#define MOTOR_1_A_PIN 8U
#define MOTOR_1_B_PIN 9U
#define MOTOR_2_A_PIN 10U
#define MOTOR_2_B_PIN 11U

#define MOTOR_PWM_FREQUENCY_HZ 20000U
#define MOTOR_PWM_DIVIDER 1.0f

static bool emergency_stopped = true;
static uint16_t pwm_wrap = 0U;

static void motor_write_pair(uint pin_a,
                             uint pin_b,
                             uint16_t duty_a,
                             uint16_t duty_b)
{
    pwm_set_gpio_level(pin_a, duty_a);
    pwm_set_gpio_level(pin_b, duty_b);
}

static void motor_configure_pin(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
}

void motor_driver_init(void)
{
    const uint pins[] = {
        MOTOR_1_A_PIN,
        MOTOR_1_B_PIN,
        MOTOR_2_A_PIN,
        MOTOR_2_B_PIN
    };

    emergency_stopped = true;

    /*
     * Set the outputs to brake before enabling PWM.
     * Robo Pico: IN1=0, IN2=0 -> brake.
     */
    for (size_t i = 0U; i < 4U; ++i)
    {
        gpio_init(pins[i]);
        gpio_put(pins[i], 0);
        gpio_set_dir(pins[i], GPIO_OUT);
    }

    uint32_t clock_hz = clock_get_hz(clk_sys);

    pwm_wrap = (uint16_t)(
        (clock_hz / MOTOR_PWM_FREQUENCY_HZ) - 1U
    );

    for (size_t i = 0U; i < 4U; ++i)
    {
        motor_configure_pin(pins[i]);

        uint slice = pwm_gpio_to_slice_num(pins[i]);

        pwm_set_clkdiv(slice, MOTOR_PWM_DIVIDER);
        pwm_set_wrap(slice, pwm_wrap);
        pwm_set_gpio_level(pins[i], 0U);
        pwm_set_enabled(slice, true);
    }
}

void motor_set_speed(motor_id_t motor, int16_t speed_percent)
{
    if (emergency_stopped)
    {
        return;
    }

    if ((motor != MOTOR_1) && (motor != MOTOR_2))
    {
        return;
    }

    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    else if (speed_percent < -100)
    {
        speed_percent = -100;
    }

    uint pin_a = (motor == MOTOR_1) ?
        MOTOR_1_A_PIN : MOTOR_2_A_PIN;

    uint pin_b = (motor == MOTOR_1) ?
        MOTOR_1_B_PIN : MOTOR_2_B_PIN;

    uint16_t magnitude = (uint16_t)(
        speed_percent < 0 ? -speed_percent : speed_percent
    );

    uint16_t duty = (uint16_t)(
        ((uint32_t)magnitude * pwm_wrap) / 100U
    );

    /*
     * Brake before changing direction.
     */
    motor_write_pair(pin_a, pin_b, 0U, 0U);

    if (speed_percent > 0)
    {
        motor_write_pair(pin_a, pin_b, duty, 0U);
    }
    else if (speed_percent < 0)
    {
        motor_write_pair(pin_a, pin_b, 0U, duty);
    }
}

void motor_brake_all(void)
{
    motor_write_pair(MOTOR_1_A_PIN, MOTOR_1_B_PIN, 0U, 0U);
    motor_write_pair(MOTOR_2_A_PIN, MOTOR_2_B_PIN, 0U, 0U);
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
