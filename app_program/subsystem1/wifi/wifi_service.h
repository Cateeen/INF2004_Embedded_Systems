#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <tk/tkernel.h>
#include "../../vehicle_messages.h"

typedef struct {
	bool connected;
	bool mqtt_ready;
	uint8_t retry_count;
	uint8_t error_code;
} wifi_service_link_t;

/* These callbacks run in the application task and must return
 * promptly. They must not call MQTT, lwIP, CYW43 or other network APIs: the
 * template's built-in cyw43_utk_task owns the network stack. poll reports link
 * state; a non-E_OK result is logged and retried next period. publish must
 * copy any data needed beyond the call;
 * E_OK means it accepted the whole message, E_TMOUT means nothing was accepted
 * and the service should retry. Other results drop that message. stop is kept
 * for startup clean-up by the caller; the running service never stops itself.
 * Context and callbacks must remain valid for the task's entire lifetime.
 *
 * Communication is best-effort: faults are logged (rate-limited) and the
 * service keeps running. It never sends to g_mbf_faults, because the vehicle
 * controller latches a safe stop on any fault and telemetry loss must not
 * stop the car. Link status in g_mbf_network_status is keep-latest. */
typedef ER (*wifi_service_poll_t)(void *context, wifi_service_link_t *link);
typedef ER (*wifi_service_publish_t)(void *context, const telemetry_message_t *message);
typedef ER (*wifi_service_stop_t)(void *context);

typedef struct {
	RELTIM period_ms;
	uint32_t status_period_ms;
	uint32_t telemetry_max_age_ms;
	wifi_service_poll_t poll;
	wifi_service_publish_t publish;
	wifi_service_stop_t stop;
	void *context;
} wifi_service_config_t;

/* Opt-in service. Telemetry comes from vehicle_telemetry_report() (called by
 * the vehicle controller). Dashboard events and robot/command are handled by
 * dashboard_link. Callback stalls require independent supervision. */
ER wifi_service_start(const wifi_service_config_t *config);

/* Starts the outbound MQTT bridge and service after buffers and tests. The
 * owner snapshot must be no older than owner_max_age_ms. Startup must be
 * serialized by one owner; a failure after bridge creation requires reboot.
 * The bridge accepts only pre-packed legacy five-byte telemetry messages. */
ER wifi_service_start_mqtt(RELTIM period_ms, uint32_t status_period_ms,
			   uint32_t telemetry_max_age_ms, uint32_t owner_max_age_ms);

#if APP_TEST
BOOL wifi_service_test(void);
#endif

#endif
