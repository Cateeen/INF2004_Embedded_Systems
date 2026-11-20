#ifndef WIFI_MQTT_BRIDGE_OWNER_H
#define WIFI_MQTT_BRIDGE_OWNER_H

#include <stdint.h>

enum {
	WIFI_MQTT_PUBLISH_OK,
	WIFI_MQTT_PUBLISH_BUSY,
	WIFI_MQTT_PUBLISH_ERROR
};

typedef int (*wifi_mqtt_bridge_publish_t)(const uint8_t *payload);
void wifi_mqtt_bridge_poll(wifi_mqtt_bridge_publish_t publish, int link_up, int mqtt_ready);
void wifi_mqtt_bridge_publish_failed(void);

#endif
