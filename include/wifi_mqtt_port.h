#ifndef WIFI_MQTT_PORT_H
#define WIFI_MQTT_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Board/library adapter contract.
 * Implement these functions using the actual Wi-Fi hardware, MQTT library,
 * broker address, credentials and TLS policy selected by your team.
 * Functions should return promptly; do not put long waits inside them.
 */

typedef void (*mqtt_payload_callback_t)(const char *topic,
                                        const uint8_t *payload,
                                        size_t length);

bool wifi_mqtt_port_init(void);
bool wifi_mqtt_port_wifi_connected(void);
bool wifi_mqtt_port_mqtt_connected(void);
bool wifi_mqtt_port_connect(void);
void wifi_mqtt_port_disconnect(void);
bool wifi_mqtt_port_subscribe(const char *topic);
bool wifi_mqtt_port_publish(const char *topic,
                            const uint8_t *payload,
                            size_t length,
                            uint8_t qos,
                            bool retain);
void wifi_mqtt_port_set_callback(mqtt_payload_callback_t callback);
void wifi_mqtt_port_poll(void);

#endif
