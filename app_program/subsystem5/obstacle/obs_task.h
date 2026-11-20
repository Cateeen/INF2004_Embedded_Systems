#ifndef OBS_TASK_H
#define OBS_TASK_H

#include <tk/tkernel.h>
#include "obs_profile.h"

typedef enum {
	OBS_TASK_AHEAD = 0,
	OBS_TASK_SCAN
} obs_task_mode_t;

typedef struct {
	bool ready;
	/* No later than the oldest measurement, on the tk_get_otm clock. */
	uint32_t timestamp_ms;
	uint16_t distance_mm;
	uint16_t distances[OBS_FINE_MAX_SAMPLES];
	uint8_t count;
	uint8_t angle0_deg;
	uint8_t step_deg;
} obs_task_sample_t;

/* The callback must return promptly and never use network APIs. It owns the
 * sensor and servo. A mode change to AHEAD cancels a scan and repositions the
 * sensor; report an ahead sample only after it points ahead and settles.
 * ready=false means work is pending, not a clear reading. A ready scan must
 * contain a complete ordered scan. OBS_DIST_NONE is confirmed no object,
 * never a timeout or failed measurement. A failure returns an error. */
typedef ER (*obs_poll_t)(void *context, obs_task_mode_t mode,
			 obs_task_sample_t *sample);
/* Called on a task fault; must promptly stop sensor/servo activity. */
typedef ER (*obs_stop_t)(void *context);
/* Called once in the obstacle task before its first poll. */
typedef ER (*obs_init_t)(void *context);

typedef struct {
	RELTIM period_ms;
	uint32_t ahead_max_age_ms;
	uint32_t scan_max_age_ms;
	uint32_t scan_timeout_ms;
	uint32_t command_max_age_ms;
	uint16_t emergency_distance_mm;
	obs_poll_t poll;
	obs_stop_t stop;
	void *context;
	obs_init_t initialize;
} obs_task_config_t;

/* The single controller may request a scan only after confirming the vehicle
 * stopped, and must keep it stopped through the scan and until a fresh ahead
 * observation. A scan requires a fresh ahead observation first. ABORT and
 * RESET cancel a scan but do not clear safety latches.
 * No callback, hardware or task is started automatically. */
ER obs_task_start(const obs_task_config_t *config);

#if APP_TEST
BOOL obs_task_test(void);
#endif

#endif
