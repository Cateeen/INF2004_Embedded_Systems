#include "../../app.h"
#include "../gpio/gpio_dispatcher.h"
#include "encoder_source.h"

#include <sys/sysdef.h>
#include <tk/syslib.h>

static encoder_pair_t s_pair;
static bool s_registered;

static void encoder_irq(const uint32_t events[GPIO_IRQ_GROUPS],
			uint32_t pins, void *context)
{
	(void)context;
	encoder_pair_update(&s_pair, events[2], pins);
}

ER encoder_source_register(void)
{
	const uint32_t mask[GPIO_IRQ_GROUPS] = {0U, 0U, ENCODER_EDGE_MASK, 0U};
	UINT pin;
	ER er;

#if TK_SUPPORT_SMP
	if (tk_get_prc() != TP_PRC1) return E_CTX;
#endif
	if (s_registered) return E_OBJ;
	for (pin = ENCODER_LEFT_A_PIN; pin <= ENCODER_RIGHT_B_PIN; ++pin) {
		er = gpio_set_pin(pin, GPIO_MODE_IN);
		if (er < E_OK) return er;
		clr_w(GPIO(pin), GPIO_PDE);
		set_w(GPIO(pin), GPIO_PUE);
	}
	encoder_pair_init(&s_pair, in_w(GPIO_IN));
	er = gpio_dispatcher_register(mask, encoder_irq, NULL);
	if (er < E_OK) return er;
	s_registered = true;
	return E_OK;
}

ER encoder_source_snapshot(encoder_snapshot_t *snapshot)
{
	UINT saved;

	if (snapshot == NULL) return E_PAR;
#if TK_SUPPORT_SMP
	if (tk_get_prc() != TP_PRC1) return E_CTX;
#endif
	if (!s_registered || !gpio_dispatcher_is_started()) return E_OBJ;
	DI(saved);
	*snapshot = encoder_pair_snapshot(&s_pair);
	EI(saved);
	return E_OK;
}
