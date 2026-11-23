#ifndef GPIO_DISPATCHER_H
#define GPIO_DISPATCHER_H

#include <tk/tkernel.h>
#include "gpio_irq_router.h"

/* One startup task pinned to core 0 registers every client before start.
 * No other code may install IRQ 13 or enable GPIO interrupts on either core.
 * Registrations stay fixed and callback contexts must remain valid while the
 * IRQ is enabled. Callbacks run in interrupt context and must not block.
 * Pending bits do not preserve edge order or timestamps; clients must treat
 * simultaneous opposite edges on one pin as ambiguous. */
ER gpio_dispatcher_register(const uint32_t mask[GPIO_IRQ_GROUPS],
                            gpio_irq_callback_t callback, void *context);
ER gpio_dispatcher_start(void);

/* Read only from the same core-0 startup owner or a core-0-pinned task. */
bool gpio_dispatcher_is_started(void);

#endif
