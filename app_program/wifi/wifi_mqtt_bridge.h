#ifndef WIFI_MQTT_BRIDGE_H
#define WIFI_MQTT_BRIDGE_H

#include <tk/tkernel.h>
#include "../vehicle_messages.h"

/* Call once from one startup owner after vehicle_buffers_init() and startup
 * tests. NET=lwip MQTT=1 is required. The bridge remains enabled until reboot.
 * No task is created. max_age_ms uses the operating clock and must be <= INT32_MAX. */
ER wifi_mqtt_bridge_start(uint32_t max_age_ms);

/* Copies a telemetry message with a pre-packed legacy five-byte telemetry
 * payload to a separate outbound queue. event_type and priority are ignored.
 * E_OK means queued, not delivered; E_TMOUT means the queue is full.
 * Only the application WiFi service should submit. */
ER wifi_mqtt_bridge_submit(const telemetry_message_t *message);

/* Read an owner-published link snapshot, or return an error if the bridge has
 * faulted, stopped, or the owner has not polled within max_age_ms. */
ER wifi_mqtt_bridge_link(uint32_t max_age_ms, bool *connected, bool *mqtt_ready);

/* Request shutdown without waiting for the network owner. An already-running
 * publish may still complete; queued data is abandoned until reboot. */
ER wifi_mqtt_bridge_stop(void);

#if APP_TEST
BOOL wifi_mqtt_bridge_test(void);
#endif

#endif
