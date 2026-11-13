#include "../app.h"
#include "gpio_dispatcher.h"

#include <sys/sysdef.h>
#include <tk/syslib.h>

#define GPIO_BANK_IRQ 13U
#define GPIO_INTR_BASE (IO_BANK0_BASE + 0x0f0U)
#define GPIO_PROC0_INTE_BASE (IO_BANK0_BASE + 0x100U)
#define GPIO_PROC0_INTF_BASE (IO_BANK0_BASE + 0x110U)
#define GPIO_PROC0_INTS_BASE (IO_BANK0_BASE + 0x120U)
#define GPIO_PROC1_INTE_BASE (IO_BANK0_BASE + 0x130U)
#define GPIO_PROC1_INTF_BASE (IO_BANK0_BASE + 0x140U)

static gpio_irq_router_t s_router;
static bool s_started;

bool gpio_dispatcher_is_started(void)
{
	return s_started;
}

static void gpio_dispatcher_irq(UINT intno)
{
	uint32_t events[GPIO_IRQ_GROUPS];
	uint32_t pins;
	unsigned i;

	(void)intno;
	for (i = 0U; i < GPIO_IRQ_GROUPS; i++) {
		events[i] = in_w(GPIO_PROC0_INTS_BASE + 4U * i) &
			    s_router.claimed[i];
		if (events[i] != 0U)
			out_w(GPIO_INTR_BASE + 4U * i, events[i]);
	}
	pins = in_w(GPIO_IN);
	(void)gpio_irq_router_dispatch(&s_router, events, pins);
}

ER gpio_dispatcher_register(const uint32_t mask[GPIO_IRQ_GROUPS],
                            gpio_irq_callback_t callback, void *context)
{
#if TK_SUPPORT_SMP
	if (tk_get_prc() != 1) return E_CTX;
#endif
	if (s_started) return E_OBJ;
	return gpio_irq_router_add(&s_router, mask, callback, context) ? E_OK : E_PAR;
}

ER gpio_dispatcher_start(void)
{
	T_DINT interrupt = {0};
	unsigned i;
	ER er;

#if TK_SUPPORT_SMP
	if (tk_get_prc() != 1) return E_CTX;
#endif
	if (s_started || s_router.count == 0U) return E_OBJ;
	if ((in_w(NVIC_ISER(GPIO_BANK_IRQ)) & (1U << GPIO_BANK_IRQ)) != 0U)
		return E_OBJ;
	for (i = 0U; i < GPIO_IRQ_GROUPS; i++) {
		if (in_w(GPIO_PROC0_INTE_BASE + 4U * i) != 0U ||
		    in_w(GPIO_PROC0_INTF_BASE + 4U * i) != 0U ||
		    in_w(GPIO_PROC1_INTE_BASE + 4U * i) != 0U ||
		    in_w(GPIO_PROC1_INTF_BASE + 4U * i) != 0U)
			return E_OBJ;
	}
	interrupt.intatr = TA_HLNG;
	interrupt.inthdr = (FP)gpio_dispatcher_irq;
	er = tk_def_int(GPIO_BANK_IRQ, &interrupt);
	if (er < E_OK) return er;

	s_started = true;
	for (i = 0U; i < GPIO_IRQ_GROUPS; i++) {
		out_w(GPIO_INTR_BASE + 4U * i, s_router.claimed[i]);
		out_w(GPIO_PROC0_INTE_BASE + 4U * i, s_router.claimed[i]);
	}
	EnableInt(GPIO_BANK_IRQ, 2);
	return E_OK;
}
