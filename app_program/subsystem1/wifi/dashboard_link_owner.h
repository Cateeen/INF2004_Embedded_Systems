/*
 *	dashboard_link_owner.h
 *	Network-owner side of dashboard_link. Called ONLY from the task that
 *	owns lwIP/MQTT (cyw43_utk_task via lib/liblwip/.../mqtt_utk.c).
 *
 *	Kept free of kernel headers, like wifi_mqtt_bridge_owner.h, because
 *	mqtt_utk.c avoids tk/tkernel.h.
 */

#ifndef DASHBOARD_LINK_OWNER_H
#define DASHBOARD_LINK_OWNER_H

#include <stdbool.h>
#include <stdint.h>

#define DASHBOARD_COMMAND_TOPIC   "robot/command"
#define DASHBOARD_HEARTBEAT_TOPIC "robot/heartbeat"

/* Returns WIFI_MQTT_PUBLISH_OK, _BUSY (keep and retry next poll) or _ERROR
 * (drop this message), from wifi_mqtt_bridge_owner.h. */
typedef int (*dashboard_publish_t)(const char *topic, const uint8_t *payload,
				   uint16_t length);

/* Publishes up to a few queued messages per call while mqtt_ready. */
void dashboard_link_owner_poll(dashboard_publish_t publish, int mqtt_ready);

/* Hand every complete inbound MQTT message here; non-command topics and
 * malformed packets are ignored. */
void dashboard_link_owner_on_message(const char *topic, const uint8_t *data,
				     uint16_t length);

/* Fills the 2-byte robot/heartbeat payload. False if the link is not started. */
bool dashboard_link_owner_heartbeat(uint8_t out[2]);

#endif /* DASHBOARD_LINK_OWNER_H */
