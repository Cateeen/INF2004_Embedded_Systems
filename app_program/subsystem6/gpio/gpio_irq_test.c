#include "../../app.h"
#include "gpio_irq_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "gpio_irq_router.h"

typedef struct {
	uint32_t events[GPIO_IRQ_GROUPS];
	uint32_t pins;
	unsigned calls;
} captured_t;

static void capture(const uint32_t events[GPIO_IRQ_GROUPS],
                    uint32_t pins, void *context)
{
	captured_t *result = context;
	unsigned i;
	for (i = 0U; i < GPIO_IRQ_GROUPS; i++) result->events[i] = events[i];
	result->pins = pins;
	result->calls++;
}

static BOOL routing_test(void)
{
	gpio_irq_router_t router = {0};
	captured_t encoder = {0}, echo = {0};
	const uint32_t encoder_mask[GPIO_IRQ_GROUPS] = {0U, 0U, 0x0000ccccU, 0U};
	const uint32_t echo_mask[GPIO_IRQ_GROUPS] = {0x000c0000U, 0U, 0U, 0U};
	const uint32_t events[GPIO_IRQ_GROUPS] = {0x000c0000U, 0U, 0x000000ccU, 0U};

	return gpio_irq_router_add(&router, encoder_mask, capture, &encoder) &&
	       gpio_irq_router_add(&router, echo_mask, capture, &echo) &&
	       gpio_irq_router_dispatch(&router, events, 0x1234U) &&
	       encoder.calls == 1U && encoder.events[2] == 0xccU &&
	       encoder.events[0] == 0U && encoder.pins == 0x1234U &&
	       echo.calls == 1U && echo.events[0] == 0x000c0000U &&
	       echo.events[2] == 0U && echo.pins == 0x1234U;
}

static BOOL ownership_test(void)
{
	gpio_irq_router_t router = {0};
	captured_t captured = {0};
	const uint32_t original[GPIO_IRQ_GROUPS] = {0U, 0U, 0x00000004U, 0U};
	const uint32_t same_pin[GPIO_IRQ_GROUPS] = {0U, 0U, 0x00000008U, 0U};
	const uint32_t invalid_level[GPIO_IRQ_GROUPS] = {1U, 0U, 0U, 0U};
	const uint32_t invalid_pin[GPIO_IRQ_GROUPS] = {0U, 0U, 0U, 0x04000000U};
	const uint32_t empty[GPIO_IRQ_GROUPS] = {0U, 0U, 0U, 0U};
	const uint32_t unrelated[GPIO_IRQ_GROUPS] = {0U, 0U, 0x00000080U, 0U};
	const uint32_t second[GPIO_IRQ_GROUPS] = {4U, 0U, 0U, 0U};
	const uint32_t third[GPIO_IRQ_GROUPS] = {0U, 4U, 0U, 0U};
	const uint32_t fourth[GPIO_IRQ_GROUPS] = {0U, 0U, 0U, 4U};
	const uint32_t fifth[GPIO_IRQ_GROUPS] = {0x40U, 0U, 0U, 0U};

	return gpio_irq_router_add(&router, original, capture, &captured) &&
	       !gpio_irq_router_add(&router, same_pin, capture, &captured) &&
	       !gpio_irq_router_add(&router, invalid_level, capture, &captured) &&
	       !gpio_irq_router_add(&router, invalid_pin, capture, &captured) &&
	       !gpio_irq_router_add(&router, empty, capture, &captured) &&
	       !gpio_irq_router_add(&router, original, NULL, &captured) &&
	       gpio_irq_router_dispatch(&router, unrelated, 0U) &&
	       captured.calls == 0U &&
	       gpio_irq_router_add(&router, second, capture, &captured) &&
	       gpio_irq_router_add(&router, third, capture, &captured) &&
	       gpio_irq_router_add(&router, fourth, capture, &captured) &&
	       !gpio_irq_router_add(&router, fifth, capture, &captured);
}

BOOL gpio_irq_test(void)
{
	BOOL routing_ok;
	BOOL ownership_ok;
	TEST_INIT();

	routing_ok = routing_test();
	ownership_ok = ownership_test();
	TEST_CHECK("GPIO-U01", routing_ok, "simultaneous events routed to owners");
	TEST_CHECK("GPIO-U02", ownership_ok, "pin conflicts and invalid masks rejected");
	TEST_SUMMARY("gpio routing");
	return routing_ok && ownership_ok;
}
#endif
