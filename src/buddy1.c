#include "buddy1.h"
#include "wifi_mqtt_port.h"

#include <string.h>

#define TELEMETRY_TOPIC "robot/telemetry"
#define HEARTBEAT_TOPIC "robot/heartbeat"
#define COMMAND_TOPIC   "robot/command"
#define FAULT_TOPIC     "robot/fault"

#define PUBLISH_INTERVAL_MS 1000U
#define RETRY_INTERVAL_MS   1000U
#define CONNECTION_TIMEOUT_MS 15000U
#define MQTT_QOS             0U

static buddy1_t *g_instance;

static bool time_reached(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target) >= 0;
}

static bool command_allowed(uint8_t type)
{
    switch (type) {
        case CMD_START:
        case CMD_PAUSE:
        case CMD_RESUME:
        case CMD_STOP:
        case CMD_REQUEST_STATUS:
        case CMD_JUNCTION:
            return true;
        default:
            return false;
    }
}

static void receive_mqtt(const char *topic, const uint8_t *payload,
                         size_t length)
{
    if ((g_instance == NULL) || (topic == NULL) || (payload == NULL)) return;
    if (strcmp(topic, COMMAND_TOPIC) != 0) return;
    if (length != COMMAND_PACKET_SIZE) return;

    remote_command_t cmd;
    if (!command_unpack(payload, &cmd)) return;
    if (!command_allowed(cmd.type)) return;

    buddy1_t *b = g_instance;
    if (b->command_count >= BUDDY1_COMMAND_QUEUE_CAPACITY) {
        /* Queue overflow: reject newest command, never overwrite queued data. */
        return;
    }
    b->command_queue[b->command_tail] = cmd;
    b->command_tail = (uint8_t)((b->command_tail + 1U) %
                                BUDDY1_COMMAND_QUEUE_CAPACITY);
    b->command_count++;
}

void buddy1_init(buddy1_t *b, uint32_t now_ms)
{
    if (b == NULL) return;
    memset(b, 0, sizeof(*b));
    b->network_state = NET_DISCONNECTED;
    b->next_retry_ms = now_ms;
    b->latest.message_type = MSG_TELEMETRY;
    b->latest.vehicle_state = VEHICLE_IDLE;
    b->latest.line_state = LINE_UNKNOWN;
    b->latest.obstacle_state = OBSTACLE_UNKNOWN;
    b->latest.terrain_state = TERRAIN_UNKNOWN;
    b->latest.battery_pct = 0U;
    b->initialised = wifi_mqtt_port_init();
    g_instance = b;
    wifi_mqtt_port_set_callback(receive_mqtt);
}

void buddy1_set_telemetry(buddy1_t *b, const telemetry_data_t *data)
{
    if ((b == NULL) || (data == NULL)) return;
    /* Validate by attempting to pack; do not accept out-of-range fields. */
    uint8_t packet[TELEMETRY_PACKET_SIZE];
    if (telemetry_pack(data, packet)) {
        uint8_t sequence = b->latest.sequence;
        b->latest = *data;
        b->latest.sequence = sequence;
    }
}

void buddy1_set_faults(buddy1_t *b, uint8_t fault_flags)
{
    if (b == NULL) return;
    b->fault_flags = (uint32_t)(fault_flags & FAULT_ALL);
    b->latest.fault_flags = (uint8_t)(fault_flags & FAULT_ALL);
}

static void try_connect(buddy1_t *b, uint32_t now_ms)
{
    b->network_state = NET_CONNECTING;
    if (wifi_mqtt_port_connect()) {
        b->next_retry_ms = now_ms + CONNECTION_TIMEOUT_MS;
    } else {
        wifi_mqtt_port_disconnect();
        b->network_state = NET_RECONNECT_WAIT;
        b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
        b->wifi_fault_pending = true;
        b->latest.fault_flags |= FAULT_WIFI_LOSS;
    }
}

static void publish_status(buddy1_t *b, uint32_t now_ms)
{
    uint8_t packet[TELEMETRY_PACKET_SIZE];
    b->latest.message_type = MSG_TELEMETRY;
    b->latest.sequence = (uint8_t)((b->latest.sequence + 1U) & 0x7FU);
    b->latest.fault_flags = (uint8_t)(b->fault_flags & FAULT_ALL);
    if (b->wifi_fault_pending) {
        b->latest.fault_flags |= FAULT_WIFI_LOSS;
    }

    if (!telemetry_pack(&b->latest, packet)) return;
    if (!wifi_mqtt_port_publish(TELEMETRY_TOPIC, packet,
                                TELEMETRY_PACKET_SIZE, MQTT_QOS, false)) {
        wifi_mqtt_port_disconnect();
        b->network_state = NET_RECONNECT_WAIT;
        b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
        b->wifi_fault_pending = true;
        b->latest.fault_flags |= FAULT_WIFI_LOSS;
        return;
    }

    uint8_t heartbeat[2] = {
        (uint8_t)((MSG_HEARTBEAT << 5U) |
                  ((b->latest.vehicle_state & 0x07U) << 2U)),
        b->latest.sequence
    };
    if (!wifi_mqtt_port_publish(HEARTBEAT_TOPIC, heartbeat,
                                sizeof(heartbeat), MQTT_QOS, false)) {
        wifi_mqtt_port_disconnect();
        b->network_state = NET_RECONNECT_WAIT;
        b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
        b->wifi_fault_pending = true;
        b->latest.fault_flags |= FAULT_WIFI_LOSS;
        return;
    }
    if (b->latest.fault_flags != 0U) {
        if (!wifi_mqtt_port_publish(FAULT_TOPIC, &b->latest.fault_flags,
                                    sizeof(b->latest.fault_flags),
                                    MQTT_QOS, false)) {
            wifi_mqtt_port_disconnect();
            b->network_state = NET_RECONNECT_WAIT;
            b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
            b->wifi_fault_pending = true;
            b->latest.fault_flags |= FAULT_WIFI_LOSS;
            return;
        }
    }
    b->wifi_fault_pending = false;
    b->last_publish_ms = now_ms;
}

void buddy1_poll(buddy1_t *b, uint32_t now_ms)
{
    if ((b == NULL) || !b->initialised) return;

    /* Must be a short, cooperative poll; adapter must not busy-wait. */
    wifi_mqtt_port_poll();

    if (wifi_mqtt_port_connection_failed()) {
        wifi_mqtt_port_disconnect();
        b->latest.fault_flags |= FAULT_WIFI_LOSS;
        b->wifi_fault_pending = true;
        b->network_state = NET_RECONNECT_WAIT;
        b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
    }

    if (b->network_state == NET_DISCONNECTED) {
        b->next_retry_ms = now_ms;
        b->network_state = NET_RECONNECT_WAIT;
    }

    if (b->network_state == NET_CONNECTED &&
        (!wifi_mqtt_port_wifi_connected() ||
         !wifi_mqtt_port_mqtt_connected())) {
        wifi_mqtt_port_disconnect();
        b->latest.fault_flags |= FAULT_WIFI_LOSS;
        b->wifi_fault_pending = true;
        b->network_state = NET_RECONNECT_WAIT;
        b->next_retry_ms = now_ms;
    }

    if (b->network_state == NET_CONNECTING) {
        if (wifi_mqtt_port_wifi_connected() &&
            wifi_mqtt_port_mqtt_connected()) {
            if (wifi_mqtt_port_subscribe(COMMAND_TOPIC)) {
                if (wifi_mqtt_port_subscribed()) {
                    b->network_state = NET_CONNECTED;
                    b->last_publish_ms = now_ms - PUBLISH_INTERVAL_MS;
                }
            } else {
                wifi_mqtt_port_disconnect();
                b->network_state = NET_RECONNECT_WAIT;
                b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
                b->wifi_fault_pending = true;
                b->latest.fault_flags |= FAULT_WIFI_LOSS;
            }
        } else if (time_reached(now_ms, b->next_retry_ms)) {
            wifi_mqtt_port_disconnect();
            b->network_state = NET_RECONNECT_WAIT;
            b->next_retry_ms = now_ms + RETRY_INTERVAL_MS;
            b->wifi_fault_pending = true;
            b->latest.fault_flags |= FAULT_WIFI_LOSS;
        }
    }

    if (b->network_state == NET_RECONNECT_WAIT &&
        time_reached(now_ms, b->next_retry_ms)) {
        try_connect(b, now_ms);
        return;
    }

    if (b->network_state == NET_CONNECTED &&
        time_reached(now_ms, b->last_publish_ms + PUBLISH_INTERVAL_MS)) {
        publish_status(b, now_ms);
    }
}

bool buddy1_take_command(buddy1_t *b, remote_command_t *out)
{
    if ((b == NULL) || (out == NULL) || (b->command_count == 0U)) return false;
    *out = b->command_queue[b->command_head];
    b->command_head = (uint8_t)((b->command_head + 1U) %
                                BUDDY1_COMMAND_QUEUE_CAPACITY);
    b->command_count--;
    return true;
}

network_state_t buddy1_network_state(const buddy1_t *b)
{
    return (b == NULL) ? NET_DISCONNECTED : b->network_state;
}
