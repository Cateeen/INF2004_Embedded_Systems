#ifndef WIFI_MQTT_PORT_H
#define WIFI_MQTT_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Pico W adapter contract. Connection is asynchronous: call poll regularly
 * while the application runs. Credentials and broker settings belong in the
 * ignored local include/wifi_mqtt_config.h file.
 */

typedef void (*mqtt_payload_callback_t)(const char *topic,
                                        const uint8_t *payload,
                                        size_t length);

bool wifi_mqtt_port_init(void);
bool wifi_mqtt_port_wifi_connected(void);
bool wifi_mqtt_port_mqtt_connected(void);
bool wifi_mqtt_port_connection_failed(void);
bool wifi_mqtt_port_connect(void);
void wifi_mqtt_port_disconnect(void);
bool wifi_mqtt_port_subscribe(const char *topic);
/* True only after the broker acknowledges the subscription. */
bool wifi_mqtt_port_subscribed(void);
bool wifi_mqtt_port_publish(const char *topic,
                            const uint8_t *payload,
                            size_t length,
                            uint8_t qos,
                            bool retain);
void wifi_mqtt_port_set_callback(mqtt_payload_callback_t callback);
void wifi_mqtt_port_poll(void);

#endif
