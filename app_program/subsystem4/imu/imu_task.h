#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <tk/tkernel.h>
#include "imu_types.h"

/* Initialization runs in the task context before the first read. The
 * callbacks and context are exclusively owned by this task and must stay
 * valid for its lifetime. E_OK from read requires a fresh sample and its
 * acquisition timestamp on the tk_get_otm clock. No network APIs. */
typedef ER (*imu_init_t)(void *context);
typedef ER (*imu_read_t)(void *context, accel_data_t *sample,
			 uint32_t *timestamp_ms);

typedef struct {
	RELTIM period_ms;
	uint32_t sample_max_age_ms;
	uint32_t odometry_max_age_ms;
	imu_init_t initialize;
	imu_read_t read;
	void *read_context;
} imu_task_config_t;

/* Called by the single startup owner after buffer creation. No reader is
 * supplied and the task is not started automatically. */
ER imu_task_start(const imu_task_config_t *config);

#if APP_TEST
BOOL imu_task_test(void);
#endif

#endif
