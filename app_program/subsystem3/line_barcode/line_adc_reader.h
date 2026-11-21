#ifndef LINE_ADC_READER_H
#define LINE_ADC_READER_H

#include <tk/tkernel.h>
#include "line_barcode_task.h"
#include "line_observation.h"

typedef struct {
	uint8_t channel;
	uint16_t threshold;
	bool black_above;
} line_adc_channel_t;

typedef struct {
	line_adc_channel_t barcode;
	line_adc_channel_t center;
	line_adc_channel_t right;
	uint32_t max_sample_span_ms;
} line_adc_config_t;

typedef struct {
	line_adc_config_t config;
	ID device;
	UW pending_value;
	bool initialized;
	bool faulted;
} line_adc_reader_t;

/* Import the original line/barcode ADC mapping and thresholds. The acquisition
 * span remains caller-selected because it depends on the chosen task period. */
ER line_adc_reader_default_config(line_adc_config_t *config,
				  uint32_t max_sample_span_ms);

/* Zero-initialize the reader and keep it alive until close or reboot. A
 * single owner must reserve the ADC and pins before calling init. Channels
 * 0..2 map to GP26..28; GP29 is prohibited. */
ER line_adc_reader_init(line_adc_reader_t *reader, const line_adc_config_t *config);

/* Starts the line/barcode task with a static-lifetime ADC reader. ADC
 * ownership is acquired inside that task; this function is not called by
 * default. */
ER line_adc_reader_start_task(const line_adc_config_t *reader_config,
			      const line_barcode_task_config_t *task_config);

/* Uses the imported sensor mapping/thresholds with a caller-selected
 * acquisition span, then starts the line/barcode task. */
ER line_adc_reader_start_default_task(uint32_t max_sample_span_ms,
				      const line_barcode_task_config_t *task_config);

/* Compatible with line_barcode_read_t. On error, outputs are unchanged and
 * the reader is latched faulted; its ADC ownership is retained until reboot. */
ER line_adc_reader_read(void *context, ir_readings_t *readings,
			uint32_t *timestamp_ms);

/* Bench reads: only sample the selected inputs; other fields are zeroed. */
ER line_adc_reader_read_line(line_adc_reader_t *reader, ir_readings_t *readings,
			     uint32_t *timestamp_ms);
ER line_adc_reader_read_barcode(line_adc_reader_t *reader, ir_readings_t *readings,
				uint32_t *timestamp_ms);

/* Close only an unfaulted reader that no task is using. Pins remain analog. */
ER line_adc_reader_close(line_adc_reader_t *reader);

#if APP_TEST
BOOL line_adc_reader_test(void);
#endif

#endif
