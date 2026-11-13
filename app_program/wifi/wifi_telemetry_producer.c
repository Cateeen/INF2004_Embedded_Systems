#include "../app.h"
#include "../vehicle_buffers.h"
#include "wifi_telemetry_producer.h"

ER wifi_telemetry_producer_init(wifi_telemetry_producer_t *producer,
				uint32_t first_sequence, uint32_t max_age_ms)
{
	SYSTIM now;
	ER result;

	if (producer == NULL || max_age_ms == 0U || max_age_ms > INT32_MAX) return E_PAR;
	if (producer->initialized || g_mbf_telemetry <= 0) return E_OBJ;
	result = tk_get_otm(&now);
	if (result != E_OK) return result < E_OK ? result : E_SYS;
	*producer = (wifi_telemetry_producer_t){0};
	producer->next_sequence = first_sequence;
	producer->max_age_ms = max_age_ms;
	producer->last_clock_ms = now.lo;
	producer->initialized = true;
	return E_OK;
}

ER wifi_telemetry_producer_send(wifi_telemetry_producer_t *producer,
				const wifi_telemetry_data_t *data, uint32_t timestamp_ms)
{
	telemetry_message_t message;
	SYSTIM now;
	ER result;

	if (producer == NULL || data == NULL) return E_PAR;
	if (!producer->initialized || producer->clock_fault) return E_OBJ;
	result = tk_get_otm(&now);
	if (result != E_OK || now.lo - producer->last_clock_ms > INT32_MAX) {
		producer->clock_fault = true;
		return result < E_OK ? result : E_SYS;
	}
	producer->last_clock_ms = now.lo;
	if (now.lo - timestamp_ms > producer->max_age_ms ||
	    (producer->have_sent &&
	     timestamp_ms - producer->last_timestamp_ms > INT32_MAX) ||
	    !wifi_telemetry_make(data, producer->next_sequence, timestamp_ms, &message))
		return E_PAR;
	result = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL);
	if (result == E_OK) {
		producer->last_timestamp_ms = timestamp_ms;
		producer->have_sent = true;
		producer->next_sequence++;
	}
	return result;
}
