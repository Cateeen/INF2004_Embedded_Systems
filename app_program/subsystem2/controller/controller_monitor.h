#ifndef CONTROLLER_MONITOR_H
#define CONTROLLER_MONITOR_H

#include <tk/tkernel.h>
#include "controller_state.h"

typedef struct {
	car_safety_config_t safety;
	RELTIM period_ms;
} controller_monitor_config_t;

/* Opt-in diagnostic only; exclusively consumes the controller's line, IMU,
 * obstacle and fault buffers. Start before their producers, not alongside
 * another consumer. It never issues motor, scan, or network commands and
 * cannot stop or verify the physical robot. It steps the advisory controller
 * state with go held low, so it cannot arm. Keep motor power safely disabled.
 * Only forward obstacle observations are supported; do not request scans.
 * Faults latch until restart and are reported on the console. */
ER controller_monitor_start(const controller_monitor_config_t *config);

#if APP_TEST
BOOL controller_monitor_test(void);
#endif

#endif
