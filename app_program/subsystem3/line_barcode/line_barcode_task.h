#ifndef LINE_BARCODE_TASK_H
#define LINE_BARCODE_TASK_H

#include <tk/tkernel.h>
#include "barcode_scanner.h"
#include "line_observation.h"

/* Initialization runs in the task context before its first sample. The
 * callbacks and context belong exclusively to this task and must remain
 * valid for its lifetime. E_OK means all readings were freshly acquired;
 * timestamp_ms is a time no later than barcode acquisition, measured on the
 * tk_get_otm clock; acquisition must follow within the configured age limit.
 * The other readings must be acquired close enough to that time for the
 * configured age limit. The callback must not use network APIs. */
typedef ER (*line_barcode_init_t)(void *context);
typedef ER (*line_barcode_read_t)(void *context, ir_readings_t *readings,
				  uint32_t *timestamp_ms);

typedef struct {
	RELTIM period_ms;
	/* Maximum age for both the sensor sample and the odometry used with it. */
	uint32_t odometry_max_age_ms;
	barcode_scanner_config_t barcode;
	line_barcode_init_t initialize;
	line_barcode_read_t read;
	void *read_context;
} line_barcode_task_config_t;

/* The single startup owner calls this after buffer creation. No hardware
 * reader or calibration is supplied by default, and startup is not automatic. */
ER line_barcode_task_start(const line_barcode_task_config_t *config);

#if APP_TEST
BOOL line_barcode_task_test(void);
#endif

#endif
