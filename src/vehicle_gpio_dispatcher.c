#include "vehicle_gpio_dispatcher.h"

#include "obs_cfg.h"
#include "obs_port.h"
#include "vehicle_odometry_task.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"

#define ODOMETRY_LEFT_A_GPIO  16U
#define ODOMETRY_LEFT_B_GPIO  17U
#define ODOMETRY_RIGHT_A_GPIO 18U
#define ODOMETRY_RIGHT_B_GPIO 19U

static void gpio_dispatcher(uint gpio, uint32_t events)
{
    switch (gpio) {
        case OBS_PIN_ECHO:
            obs_echo_irq(gpio, events);
            break;
        case ODOMETRY_LEFT_A_GPIO:
        case ODOMETRY_LEFT_B_GPIO:
            vehicle_odometry_left_edge(
                (uint8_t)gpio_get(ODOMETRY_LEFT_A_GPIO),
                (uint8_t)gpio_get(ODOMETRY_LEFT_B_GPIO));
            break;
        case ODOMETRY_RIGHT_A_GPIO:
        case ODOMETRY_RIGHT_B_GPIO:
            vehicle_odometry_right_edge(
                (uint8_t)gpio_get(ODOMETRY_RIGHT_A_GPIO),
                (uint8_t)gpio_get(ODOMETRY_RIGHT_B_GPIO));
            break;
        default:
            break;
    }
}

void vehicle_gpio_dispatcher_init(void)
{
    gpio_set_irq_callback(gpio_dispatcher);
    irq_set_enabled(IO_IRQ_BANK0, true);
}
