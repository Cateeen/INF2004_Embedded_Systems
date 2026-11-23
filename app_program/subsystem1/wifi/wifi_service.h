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
 * state and any non-E_OK result stops the service. publish must copy any data
 * needed beyond the call;
 * E_OK means it accepted the whole message, E_TMOUT means nothing was accepted
 * and the service should retry. Other results are failures. stop requests that
 * the backend stop promptly. Context and callbacks must remain valid for the
 * task's entire lifetime. */
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

/* Opt-in service. The controller must consume status and faults and handle
 * queue-full errors. Remote commands are not implemented. Callback stalls
 * require independent supervision. */
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
