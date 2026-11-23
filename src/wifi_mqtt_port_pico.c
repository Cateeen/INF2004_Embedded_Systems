#include "wifi_mqtt_port.h"

#if defined(__has_include)
#if __has_include("wifi_mqtt_config.h")
#include "wifi_mqtt_config.h"
#else
#include "wifi_mqtt_config.example.h"
#endif
#else
#include "wifi_mqtt_config.example.h"
#endif

#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define MQTT_COMMAND_PAYLOAD_SIZE 2U
#define MQTT_COMMAND_TOPIC_SIZE  64U
#define MQTT_KEEPALIVE_SECONDS   30U

static mqtt_client_t *mqtt_client;
static struct mqtt_connect_client_info_t mqtt_client_info;
static mqtt_payload_callback_t payload_callback;
static ip_addr_t broker_address;
static char incoming_topic[MQTT_COMMAND_TOPIC_SIZE];
static char subscription_topic[MQTT_COMMAND_TOPIC_SIZE];
static uint8_t incoming_payload[MQTT_COMMAND_PAYLOAD_SIZE];
static u32_t incoming_expected_length;
static size_t incoming_received_length;
static bool initialized;
static bool attempt_active;
static bool dns_pending;
static bool broker_resolved;
static bool mqtt_connecting;
static bool connection_failed;
static bool discard_incoming;
static bool subscription_pending;
static bool subscription_active;

static void mqtt_incoming_publish_callback(void *arg, const char *topic,
                                           u32_t total_length);
static void mqtt_incoming_data_callback(void *arg, const u8_t *data,
                                        u16_t length, u8_t flags);

static bool config_is_complete(void)
{
    return (WIFI_SSID[0] != '\0') &&
           (MQTT_BROKER_HOST[0] != '\0') &&
           (MQTT_BROKER_PORT > 0U) &&
           (MQTT_CLIENT_ID[0] != '\0') &&
           ((MQTT_USERNAME[0] == '\0') == (MQTT_PASSWORD[0] == '\0'));
}

static void reset_broker_resolution(void)
{
    dns_pending = false;
    broker_resolved = false;
    mqtt_connecting = false;
    connection_failed = false;
    subscription_pending = false;
    subscription_active = false;
    subscription_topic[0] = '\0';
}

static void mqtt_connection_callback(mqtt_client_t *client, void *arg,
                                     mqtt_connection_status_t status)
{
    (void)arg;
    printf("MQTT connection callback status: %d\n", (int)status);
    stdio_flush();
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_connecting = false;
        mqtt_set_inpub_callback(client, mqtt_incoming_publish_callback,
                                 mqtt_incoming_data_callback, NULL);
        return;
    }

    mqtt_connecting = false;
    attempt_active = false;
    connection_failed = true;
}

static void broker_dns_found_callback(const char *name,
                                      const ip_addr_t *address, void *arg)
{
    (void)name;
    (void)arg;
    dns_pending = false;
    if (address == NULL) {
        connection_failed = true;
        return;
    }

    ip_addr_copy(broker_address, *address);
    broker_resolved = true;
}

static void mqtt_incoming_publish_callback(void *arg, const char *topic,
                                           u32_t total_length)
{
    (void)arg;
    incoming_expected_length = total_length;
    incoming_received_length = 0U;
    discard_incoming = (topic == NULL) ||
                       (strlen(topic) >= sizeof(incoming_topic)) ||
                       (total_length != MQTT_COMMAND_PAYLOAD_SIZE);

    if (!discard_incoming) {
        strcpy(incoming_topic, topic);
    }
}

static void mqtt_incoming_data_callback(void *arg, const u8_t *data,
                                        u16_t length, u8_t flags)
{
    (void)arg;

    if ((data == NULL) ||
        (length > (MQTT_COMMAND_PAYLOAD_SIZE - incoming_received_length))) {
        discard_incoming = true;
    } else if (!discard_incoming) {
        memcpy(&incoming_payload[incoming_received_length], data, length);
        incoming_received_length += length;
    }

    if ((flags & MQTT_DATA_FLAG_LAST) != 0U) {
        if (!discard_incoming &&
            (incoming_received_length == incoming_expected_length) &&
            (payload_callback != NULL)) {
            payload_callback(incoming_topic, incoming_payload,
                             incoming_received_length);
        }
        discard_incoming = false;
    }
}

static void mqtt_subscribe_callback(void *arg, err_t result)
{
    (void)arg;
    printf("MQTT subscription callback result: %d\n", (int)result);
    stdio_flush();
    subscription_pending = false;
    if (result == ERR_OK) {
        subscription_active = true;
    } else {
        connection_failed = true;
    }
}

static void mqtt_request_callback(void *arg, err_t result)
{
    (void)arg;
    if (result != ERR_OK) {
        connection_failed = true;
    }
}

static void start_mqtt_connect(void)
{
    err_t result;

    if (!broker_resolved || mqtt_connecting ||
        (mqtt_client == NULL) || !attempt_active ||
        (mqtt_client_is_connected(mqtt_client) != 0U)) {
        return;
    }

    mqtt_connecting = true;
    printf("MQTT connecting to %s:%u\n", MQTT_BROKER_HOST,
           (unsigned int)MQTT_BROKER_PORT);
    stdio_flush();
    result = mqtt_client_connect(mqtt_client, &broker_address,
                                 (u16_t)MQTT_BROKER_PORT,
                                 mqtt_connection_callback, NULL,
                                 &mqtt_client_info);
    if (result != ERR_OK) {
        mqtt_connecting = false;
        connection_failed = true;
        attempt_active = false;
    }
}

static void resolve_broker(void)
{
    err_t result;

    if (broker_resolved || dns_pending || connection_failed) {
        return;
    }

    if (ipaddr_aton(MQTT_BROKER_HOST, &broker_address)) {
        broker_resolved = true;
        start_mqtt_connect();
        return;
    }

    dns_pending = true;
    result = dns_gethostbyname(MQTT_BROKER_HOST, &broker_address,
                               broker_dns_found_callback, NULL);
    if (result == ERR_OK) {
        dns_pending = false;
        broker_resolved = true;
        start_mqtt_connect();
    } else if (result != ERR_INPROGRESS) {
        dns_pending = false;
        connection_failed = true;
        attempt_active = false;
    }
}

bool wifi_mqtt_port_init(void)
{
    int cyw43_result;

    if (initialized) {
        printf("MQTT adapter: already initialized.\n");
        stdio_flush();
        return true;
    }
    printf("MQTT adapter: checking local configuration...\n");
    stdio_flush();
    if (!config_is_complete()) {
        printf("MQTT adapter: configuration is incomplete or invalid.\n");
        stdio_flush();
        return false;
    }

    printf("MQTT adapter: initializing Pico W networking...\n");
    stdio_flush();
    cyw43_result = cyw43_arch_init();
    if (cyw43_result != 0) {
        printf("MQTT adapter: Pico W networking init failed (%d).\n",
               cyw43_result);
        stdio_flush();
        return false;
    }

    cyw43_arch_enable_sta_mode();

    printf("MQTT adapter: Pico W networking initialized; allocating client...\n");
    stdio_flush();
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("MQTT adapter: MQTT client allocation failed.\n");
        stdio_flush();
        cyw43_arch_deinit();
        return false;
    }

    memset(&mqtt_client_info, 0, sizeof(mqtt_client_info));
    mqtt_client_info.client_id = MQTT_CLIENT_ID;
    mqtt_client_info.client_user =
        (MQTT_USERNAME[0] == '\0') ? NULL : MQTT_USERNAME;
    mqtt_client_info.client_pass =
        (MQTT_PASSWORD[0] == '\0') ? NULL : MQTT_PASSWORD;
    mqtt_client_info.keep_alive = MQTT_KEEPALIVE_SECONDS;
    initialized = true;
    printf("MQTT adapter: initialization complete.\n");
    stdio_flush();
    return true;
}

bool wifi_mqtt_port_wifi_connected(void)
{
    return initialized &&
           (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) ==
            CYW43_LINK_UP);
}

bool wifi_mqtt_port_mqtt_connected(void)
{
    return initialized && (mqtt_client != NULL) &&
           (mqtt_client_is_connected(mqtt_client) != 0U);
}

bool wifi_mqtt_port_connection_failed(void)
{
    return connection_failed;
}

bool wifi_mqtt_port_connect(void)
{
    int result;
    int link_status;

    if (!initialized || connection_failed) {
        return false;
    }
    if (attempt_active) {
        return true;
    }

    reset_broker_resolution();
    attempt_active = true;
    link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status != CYW43_LINK_UP) {
        result = cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                               WIFI_AUTH);
        if (result != 0) {
            attempt_active = false;
            connection_failed = true;
            return false;
        }
    }
    return true;
}

void wifi_mqtt_port_disconnect(void)
{
    if ((mqtt_client != NULL) &&
        ((mqtt_client_is_connected(mqtt_client) != 0U) || mqtt_connecting)) {
        mqtt_disconnect(mqtt_client);
    }
    attempt_active = false;
    reset_broker_resolution();
}

bool wifi_mqtt_port_subscribe(const char *topic)
{
    size_t topic_length;

    if ((topic == NULL) || (mqtt_client == NULL) ||
        (mqtt_client_is_connected(mqtt_client) == 0U)) {
        return false;
    }

    topic_length = strlen(topic);
    if (topic_length >= sizeof(subscription_topic)) {
        return false;
    }

    if (subscription_pending || subscription_active) {
        return strcmp(topic, subscription_topic) == 0;
    }

    memcpy(subscription_topic, topic, topic_length + 1U);
    subscription_pending = true;
    if (mqtt_subscribe(mqtt_client, topic, 0U,
                       mqtt_subscribe_callback, NULL) != ERR_OK) {
        subscription_pending = false;
        connection_failed = true;
        return false;
    }

    return true;
}

bool wifi_mqtt_port_subscribed(void)
{
    return initialized && subscription_active && (mqtt_client != NULL) &&
           (mqtt_client_is_connected(mqtt_client) != 0U);
}

bool wifi_mqtt_port_publish(const char *topic, const uint8_t *payload,
                            size_t length, uint8_t qos, bool retain)
{
    if ((topic == NULL) || ((payload == NULL) && (length != 0U)) ||
        (mqtt_client == NULL) || (length > UINT16_MAX) ||
        (mqtt_client_is_connected(mqtt_client) == 0U)) {
        return false;
    }

    return mqtt_publish(mqtt_client, topic, payload, (u16_t)length, qos,
                        retain ? 1U : 0U, mqtt_request_callback, NULL) == ERR_OK;
}

void wifi_mqtt_port_set_callback(mqtt_payload_callback_t callback)
{
    payload_callback = callback;
}

void wifi_mqtt_port_poll(void)
{
    if (!initialized) {
        return;
    }

    cyw43_arch_poll();

    if (connection_failed) {
        if ((mqtt_client != NULL) &&
            (mqtt_client_is_connected(mqtt_client) != 0U)) {
            mqtt_disconnect(mqtt_client);
        }
        attempt_active = false;
        return;
    }

    if (attempt_active && wifi_mqtt_port_wifi_connected()) {
        resolve_broker();
        start_mqtt_connect();
    }

}
