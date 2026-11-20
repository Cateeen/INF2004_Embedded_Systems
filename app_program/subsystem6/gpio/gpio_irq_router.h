#ifndef GPIO_IRQ_ROUTER_H
#define GPIO_IRQ_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_IRQ_GROUPS 4U
#define GPIO_IRQ_CLIENTS 4U

/* GPIO n is in group n/8, with falling/rising bits 2/3 of nibble n%8.
 * The event array is valid only for the duration of the callback. */
typedef void (*gpio_irq_callback_t)(const uint32_t events[GPIO_IRQ_GROUPS],
                                    uint32_t pin_levels, void *context);

typedef struct {
	uint32_t mask[GPIO_IRQ_GROUPS];
	gpio_irq_callback_t callback;
	void *context;
} gpio_irq_client_t;

typedef struct {
	gpio_irq_client_t clients[GPIO_IRQ_CLIENTS];
	uint32_t claimed[GPIO_IRQ_GROUPS];
	uint8_t count;
} gpio_irq_router_t;

/* Zero-initialise and configure before delivery starts. Each GPIO has one owner. */
bool gpio_irq_router_add(gpio_irq_router_t *router,
                         const uint32_t mask[GPIO_IRQ_GROUPS],
                         gpio_irq_callback_t callback, void *context);
bool gpio_irq_router_dispatch(const gpio_irq_router_t *router,
                              const uint32_t events[GPIO_IRQ_GROUPS],
                              uint32_t pin_levels);

#endif
