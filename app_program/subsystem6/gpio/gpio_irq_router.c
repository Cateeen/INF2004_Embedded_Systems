#include "gpio_irq_router.h"

#include <stddef.h>

bool gpio_irq_router_add(gpio_irq_router_t *router,
                         const uint32_t mask[GPIO_IRQ_GROUPS],
                         gpio_irq_callback_t callback, void *context)
{
	unsigned group;
	bool any = false;
	gpio_irq_client_t *client;

	if (router == NULL || mask == NULL || callback == NULL ||
	    router->count >= GPIO_IRQ_CLIENTS) return false;
	for (group = 0U; group < GPIO_IRQ_GROUPS; group++) {
		unsigned shift;
		if ((mask[group] & 0x33333333U) != 0U ||
		    (group == 3U && (mask[group] & 0xff000000U) != 0U))
			return false;
		for (shift = 0U; shift < 32U; shift += 4U) {
			uint32_t pin_mask = 0xfU << shift;
			if ((mask[group] & pin_mask) != 0U &&
			    (router->claimed[group] & pin_mask) != 0U)
				return false;
		}
		any = any || mask[group] != 0U;
	}
	if (!any) return false;

	client = &router->clients[router->count];
	for (group = 0U; group < GPIO_IRQ_GROUPS; group++) {
		client->mask[group] = mask[group];
		router->claimed[group] |= mask[group];
	}
	client->callback = callback;
	client->context = context;
	router->count++;
	return true;
}

bool gpio_irq_router_dispatch(const gpio_irq_router_t *router,
                              const uint32_t events[GPIO_IRQ_GROUPS],
                              uint32_t pin_levels)
{
	unsigned client_index;

	if (router == NULL || events == NULL || router->count > GPIO_IRQ_CLIENTS)
		return false;
	for (client_index = 0U; client_index < router->count; client_index++) {
		const gpio_irq_client_t *client = &router->clients[client_index];
		uint32_t selected[GPIO_IRQ_GROUPS];
		unsigned group;
		bool any = false;
		for (group = 0U; group < GPIO_IRQ_GROUPS; group++) {
			selected[group] = events[group] & client->mask[group];
			any = any || selected[group] != 0U;
		}
		if (any) client->callback(selected, pin_levels, client->context);
	}
	return true;
}
