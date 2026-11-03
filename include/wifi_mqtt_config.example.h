#ifndef WIFI_MQTT_CONFIG_EXAMPLE_H
#define WIFI_MQTT_CONFIG_EXAMPLE_H

/*
 * Copy this file to wifi_mqtt_config.h and fill in local settings.
 * Do not commit wifi_mqtt_config.h: it is ignored by git.
 * This adapter uses unencrypted MQTT (typically port 1883); do not use it
 * across an untrusted network.
 */
#define WIFI_SSID           ""
#define WIFI_PASSWORD       ""
#define WIFI_AUTH           CYW43_AUTH_WPA2_AES_PSK

#define MQTT_BROKER_HOST    ""
#define MQTT_BROKER_PORT    1883U
#define MQTT_CLIENT_ID      "pico-w-buddy1"
#define MQTT_USERNAME       ""
#define MQTT_PASSWORD       ""

#endif
