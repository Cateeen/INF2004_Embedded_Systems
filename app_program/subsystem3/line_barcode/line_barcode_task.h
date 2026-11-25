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
	/* Wheel travel per odometry count (mm). > 0 runs the junction detector
	 * (straddle_junction.h: centre AND right line sensors black together,
	 * STRADDLE_JUNCTION_DEFAULT_CONFIG): the line observation of the sample
	 * where a junction is confirmed carries intersection_detected = 1. 0
	 * leaves junction detection off. */
	float junction_mm_per_count;
	/* Called once each time a barcode frame starts (the start '*' has been
	 * read), before the letter is decoded; may be NULL. Runs in this task:
	 * must not block. */
	void (*barcode_started)(void *context);
	void *barcode_started_context;
	/* Barcode colour as the bench barcode test (SENSOR_TEST=barcode) reads
	 * it. black_adc 0: use the reader's single threshold, no hysteresis.
	 * Else hysteresis on the raw value: black at >= black_adc, white at
	 * <= white_adc, in between keeps the previous colour. debounce_counts
	 * > 0: a colour change is accepted once the wheels have moved that many
	 * counts with it held, and the edge is placed where it began. */
	uint16_t barcode_black_adc;
	uint16_t barcode_white_adc;
	uint32_t barcode_debounce_counts;
	/* Line (centre, right) sensor colour. black_adc 0: the reader's single
	 * threshold. Else hysteresis on the raw value: black at >= black_adc,
	 * white at <= white_adc, in between keeps the previous colour (a sensor
	 * half over the tape edge otherwise flickers). Used for the position,
	 * the junction and the pick-up. Appended: 0 in older initializers. */
	uint16_t line_black_adc;
	uint16_t line_white_adc;
} line_barcode_task_config_t;

/* All three IR sensors black (no floor under any of them) this long: the
 * line observation reports picked_up. Normal driving never holds it this
 * long (an 18 mm junction tape passes in ~150 ms at 12 cm/s). */
#define LINE_PICKUP_MS 500U

/* The single startup owner calls this after buffer creation. No hardware
 * reader or calibration is supplied by default, and startup is not automatic. */
ER line_barcode_task_start(const line_barcode_task_config_t *config);

#if APP_TEST
BOOL line_barcode_task_test(void);
#endif

#endif
