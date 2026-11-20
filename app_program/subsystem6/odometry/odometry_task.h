#ifndef ODOMETRY_TASK_H
#define ODOMETRY_TASK_H

#include <tk/tkernel.h>
#include "odometry_producer.h"

/* Called once by the core-0 startup owner after buffers and the shared GPIO
 * dispatcher are ready. Calibration and signs must be physically verified.
 * The task is pinned to core 0 and is not started automatically. */
ER odometry_task_start(const odometry_producer_config_t *config, RELTIM period_ms);

/* Opt in only when the motion consumer is ready. A full motion buffer stops
 * odometry just like a delivery failure to the other consumers. */
ER odometry_task_start_with_motion(const odometry_producer_config_t *config,
				   RELTIM period_ms);

#if APP_TEST
BOOL odometry_task_test(void);
#endif

#endif
