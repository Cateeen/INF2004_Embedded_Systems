#include "wifi_service_task.h"

#include "telemetry_protocol.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"
#include "wifi_mqtt_port.h"

#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define WIFI_SERVICE_PERIOD_MS       20U
#define WIFI_TELEMETRY_TOPIC         "robot/telemetry"
#define WIFI_COMMAND_TOPIC           "robot/command"
#define WIFI_HEARTBEAT_TOPIC         "robot/heartbeat"
#define WIFI_FAULT_TOPIC             "robot/fault"
#define WIFI_MQTT_QOS                0U
#define WIFI_COMMAND_QUEUE_CAPACITY  4U

typedef struct {
    uint8_t payload[COMMAND_PACKET_SIZE];
    size_t length;
} pending_command_t;

static pending_command_t s_commands[WIFI_COMMAND_QUEUE_CAPACITY];
static uint8_t s_command_head;
static uint8_t s_command_tail;
static uint8_t s_command_count;
static uint32_t s_sequence;
static bool s_initialised;
static bool s_last_wifi_connected;
static bool s_last_mqtt_connected;
static bool s_fault_reported;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static bool command_allowed(uint8_t command)
{
    return command >= CMD_START && command <= CMD_JUNCTION;
}

static void mqtt_payload_received(const char *topic, const uint8_t *payload,
                                  size_t length)
{
    pending_command_t *pending;

    if ((topic == NULL) || (payload == NULL) ||
        (strcmp(topic, WIFI_COMMAND_TOPIC) != 0) ||
        (length != COMMAND_PACKET_SIZE) ||
        (s_command_count >= WIFI_COMMAND_QUEUE_CAPACITY)) {
        return;
    }

    if (!command_allowed((uint8_t)(payload[0] >> 5U))) {
        return;
    }

    pending = &s_commands[s_command_tail];
    memcpy(pending->payload, payload, COMMAND_PACKET_SIZE);
    pending->length = COMMAND_PACKET_SIZE;
    s_command_tail = (uint8_t)((s_command_tail + 1U) %
                               WIFI_COMMAND_QUEUE_CAPACITY);
    s_command_count++;
}

static void publish_fault(uint32_t timestamp_ms, uint16_t code)
{
    fault_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_FAULT;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.source = VEHICLE_FAULT_COMMUNICATION;
    message.recoverable = 1U;
    message.code = code;

    result = tk_snd_mbf(g_mbf_faults, &message, (INT)sizeof(message), TMO_POL);
    if (result != E_OK) {
        printf("wifi: fault message send failed (%d)\n", (int)result);
    }
}

static void publish_network_status(uint32_t timestamp_ms, bool wifi_connected,
                                   bool mqtt_connected)
{
    network_status_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_NETWORK_STATUS;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.connected = wifi_connected ? 1U : 0U;
    message.mqtt_ready = mqtt_connected ? 1U : 0U;
    message.retry_count = 0U;
    message.error_code = (!wifi_connected || !mqtt_connected) ? 1U : 0U;

    result = tk_snd_mbf(g_mbf_network_status, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("wifi: network status send failed (%d)\n", (int)result);
    }
}

static void forward_commands(uint32_t timestamp_ms)
{
    remote_command_t command;
    remote_command_message_t message;
    pending_command_t *pending;
    INT result;

    while (s_command_count > 0U) {
        pending = &s_commands[s_command_head];
        if (!command_unpack(pending->payload, &command) ||
            pending->length != COMMAND_PACKET_SIZE) {
            s_command_head = (uint8_t)((s_command_head + 1U) %
                                       WIFI_COMMAND_QUEUE_CAPACITY);
            s_command_count--;
            continue;
        }

        memset(&message, 0, sizeof(message));
        message.header.type = VEHICLE_MESSAGE_REMOTE_COMMAND;
        message.header.version = VEHICLE_MESSAGE_VERSION;
        message.header.sequence = command.sequence;
        message.header.timestamp_ms = timestamp_ms;
        message.command = command.type;
        message.value = command.argument;

        result = tk_snd_mbf(g_mbf_remote_commands, &message,
                            (INT)sizeof(message), TMO_POL);
        if (result != E_OK) {
            printf("wifi: remote command send failed (%d)\n", (int)result);
            break;
        }

        s_command_head = (uint8_t)((s_command_head + 1U) %
                                   WIFI_COMMAND_QUEUE_CAPACITY);
        s_command_count--;
    }
}

static void publish_telemetry_message(const telemetry_message_t *message)
{
    if ((message->payload_length > sizeof(message->payload)) ||
        !wifi_mqtt_port_publish(WIFI_TELEMETRY_TOPIC,
                                (const uint8_t *)message->payload,
                                message->payload_length, WIFI_MQTT_QOS, false)) {
        printf("wifi: telemetry publish failed\n");
    }
}

static void publish_heartbeat(const telemetry_message_t *message)
{
    uint8_t heartbeat[2];

    heartbeat[0] = (uint8_t)((MSG_HEARTBEAT << 5U) |
                             ((message->priority & 0x07U) << 2U));
    heartbeat[1] = (uint8_t)(message->header.sequence & 0x7FU);
    if (!wifi_mqtt_port_publish(WIFI_HEARTBEAT_TOPIC, heartbeat,
                                sizeof(heartbeat), WIFI_MQTT_QOS, false)) {
        printf("wifi: heartbeat publish failed\n");
    }
}

static void publish_pending_telemetry(void)
{
    telemetry_message_t message;
    INT received;

    if (!wifi_mqtt_port_mqtt_connected()) {
        return;
    }

    do {
        received = tk_rcv_mbf(g_mbf_telemetry, &message, TMO_POL);
        if (received == (INT)sizeof(message)) {
            publish_telemetry_message(&message);
            publish_heartbeat(&message);
        }
    } while (received > 0);
}

static void publish_network_fault(uint32_t timestamp_ms, bool mqtt_connected)
{
    if (!mqtt_connected && !s_fault_reported) {
        publish_fault(timestamp_ms, 1U);
        s_fault_reported = true;
    } else if (mqtt_connected) {
        s_fault_reported = false;
    }
}

void wifi_service_task(INT stacd, void *exinf)
{
    uint32_t timestamp_ms;
    bool wifi_connected;
    bool mqtt_connected;

    (void)stacd;
    (void)exinf;
    memset(s_commands, 0, sizeof(s_commands));
    s_command_head = 0U;
    s_command_tail = 0U;
    s_command_count = 0U;
    s_sequence = 0U;
    s_initialised = false;
    s_last_wifi_connected = false;
    s_last_mqtt_connected = false;
    s_fault_reported = false;

#if defined(PICOCAR_ENABLE_MQTT) && PICOCAR_ENABLE_MQTT
    s_initialised = wifi_mqtt_port_init();
    if (s_initialised) {
        wifi_mqtt_port_set_callback(mqtt_payload_received);
    } else {
        publish_fault(now_ms(), 2U);
    }
#else
    publish_fault(now_ms(), 2U);
#endif

    for (;;) {
        timestamp_ms = now_ms();

#if defined(PICOCAR_ENABLE_MQTT) && PICOCAR_ENABLE_MQTT
        if (s_initialised) {
            wifi_mqtt_port_poll();
            if (!wifi_mqtt_port_mqtt_connected()) {
                (void)wifi_mqtt_port_connect();
            }
            forward_commands(timestamp_ms);
            publish_pending_telemetry();
        }
        wifi_connected = s_initialised && wifi_mqtt_port_wifi_connected();
        mqtt_connected = s_initialised && wifi_mqtt_port_mqtt_connected();
#else
        wifi_connected = false;
        mqtt_connected = false;
#endif

        if ((wifi_connected != s_last_wifi_connected) ||
            (mqtt_connected != s_last_mqtt_connected)) {
            publish_network_status(timestamp_ms, wifi_connected, mqtt_connected);
            s_last_wifi_connected = wifi_connected;
            s_last_mqtt_connected = mqtt_connected;
        }
        publish_network_fault(timestamp_ms, mqtt_connected);
        (void)tk_dly_tsk(WIFI_SERVICE_PERIOD_MS);
    }
}
