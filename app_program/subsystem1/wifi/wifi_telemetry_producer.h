#ifndef WIFI_TELEMETRY_PRODUCER_H
#define WIFI_TELEMETRY_PRODUCER_H

#include <tk/tkernel.h>
#include "wifi_telemetry.h"

typedef struct {
	uint32_t next_sequence;
	uint32_t max_age_ms;
	uint32_t last_clock_ms;
	uint32_t last_timestamp_ms;
	bool initialized;
	bool have_sent;
	bool clock_fault;
} wifi_telemetry_producer_t;

/* Initialize a zeroed state after vehicle_buffers_init(). Exactly one task
 * owns this state and is the sole producer of g_mbf_telemetry. max_age_ms
 * must not exceed the service's telemetry age limit. Start the service first. */
ER wifi_telemetry_producer_init(wifi_telemetry_producer_t *producer,
				uint32_t first_sequence, uint32_t max_age_ms);

/* timestamp_ms is the operating-clock timestamp of the oldest input used.
 * The caller supplies verified values, including battery percentage. E_OK
 * means queued, not delivered; E_TMOUT means the caller may retry without
 * advancing sequence. Handle all other errors. A clock fault latches until
 * restart. This function does not start a task or call network APIs. */
ER wifi_telemetry_producer_send(wifi_telemetry_producer_t *producer,
				const wifi_telemetry_data_t *data, uint32_t timestamp_ms);

#endif
