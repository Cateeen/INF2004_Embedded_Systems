#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "buddy1.h"
#include "wifi_mqtt_port.h"

static bool fake_wifi_connected;
static bool fake_mqtt_connected;
static bool fake_connection_failed;
static bool fake_subscription_pending;
static bool fake_subscribed;
static unsigned int connect_calls;
static unsigned int subscribe_calls;
static unsigned int telemetry_publish_calls;
static uint8_t last_telemetry_packet[TELEMETRY_PACKET_SIZE];
static mqtt_payload_callback_t registered_callback;

bool wifi_mqtt_port_init(void)
{
    return true;
}

bool wifi_mqtt_port_wifi_connected(void)
{
    return fake_wifi_connected;
}

bool wifi_mqtt_port_mqtt_connected(void)
{
    return fake_mqtt_connected;
}

bool wifi_mqtt_port_connection_failed(void)
{
    return fake_connection_failed;
}

bool wifi_mqtt_port_connect(void)
{
    connect_calls++;
    return true;
}

void wifi_mqtt_port_disconnect(void)
{
    fake_mqtt_connected = false;
    fake_connection_failed = false;
    fake_subscription_pending = false;
    fake_subscribed = false;
}

bool wifi_mqtt_port_subscribe(const char *topic)
{
    assert(strcmp(topic, "robot/command") == 0);
    if (!fake_subscription_pending && !fake_subscribed) {
        subscribe_calls++;
        fake_subscription_pending = true;
    }
    return true;
}

bool wifi_mqtt_port_subscribed(void)
{
    return fake_subscribed;
}

bool wifi_mqtt_port_publish(const char *topic, const uint8_t *payload,
                            size_t length, uint8_t qos, bool retain)
{
    (void)qos;
    (void)retain;
    if (strcmp(topic, "robot/telemetry") == 0) {
        assert(length == TELEMETRY_PACKET_SIZE);
        memcpy(last_telemetry_packet, payload, length);
        telemetry_publish_calls++;
    }
    return true;
}

void wifi_mqtt_port_set_callback(mqtt_payload_callback_t callback)
{
    registered_callback = callback;
}

void wifi_mqtt_port_poll(void)
{
}

int main(void)
{
    buddy1_t buddy;
    telemetry_data_t telemetry = {
        .message_type = MSG_TELEMETRY,
        .vehicle_state = VEHICLE_RUNNING,
        .line_state = LINE_UNKNOWN,
        .obstacle_state = OBSTACLE_UNKNOWN,
        .terrain_state = TERRAIN_HUMP,
        .fault_flags = 0U,
        .sequence = 0U,
        .battery_pct = 0U
    };

    buddy1_init(&buddy, 0U);
    assert(buddy.initialised);
    assert(registered_callback != NULL);

    buddy1_poll(&buddy, 0U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTING);
    assert(connect_calls == 1U);

    fake_wifi_connected = true;
    fake_mqtt_connected = true;
    buddy1_poll(&buddy, 10U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTING);
    assert(subscribe_calls == 1U);
    assert(telemetry_publish_calls == 0U);

    fake_subscribed = true;
    buddy1_poll(&buddy, 20U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTED);
    assert(subscribe_calls == 1U);
    assert(telemetry_publish_calls == 1U);

    telemetry.vehicle_state = VEHICLE_FAULT;
    telemetry.fault_flags = FAULT_IMU;
    buddy1_set_telemetry(&buddy, &telemetry);
    buddy1_set_faults(&buddy, FAULT_IMU);
    buddy1_poll(&buddy, 1020U);

    telemetry_data_t decoded;
    assert(telemetry_unpack(last_telemetry_packet, &decoded));
    assert(decoded.sequence == 2U);
    assert(decoded.vehicle_state == VEHICLE_FAULT);
    assert(decoded.terrain_state == TERRAIN_HUMP);
    assert(decoded.fault_flags == FAULT_IMU);

    remote_command_t command = {
        .type = CMD_STOP,
        .argument = 0U,
        .sequence = 42U
    };
    uint8_t command_packet[COMMAND_PACKET_SIZE];
    remote_command_t received;
    assert(command_pack(&command, command_packet));
    registered_callback("robot/command", command_packet,
                        sizeof(command_packet));
    assert(buddy1_take_command(&buddy, &received));
    assert(received.type == CMD_STOP);
    assert(received.sequence == 42U);
    assert(!buddy1_take_command(&buddy, &received));

    fake_mqtt_connected = false;
    buddy1_poll(&buddy, 1030U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTING);
    assert(connect_calls == 2U);

    fake_connection_failed = true;
    buddy1_poll(&buddy, 1031U);
    assert(buddy1_network_state(&buddy) == NET_RECONNECT_WAIT);
    buddy1_poll(&buddy, 2031U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTING);
    assert(connect_calls == 3U);

    fake_mqtt_connected = true;
    buddy1_poll(&buddy, 2040U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTING);
    fake_subscribed = true;
    buddy1_poll(&buddy, 2050U);
    assert(buddy1_network_state(&buddy) == NET_CONNECTED);
    assert(telemetry_unpack(last_telemetry_packet, &decoded));
    assert(decoded.fault_flags == (FAULT_IMU | FAULT_WIFI_LOSS));

    buddy1_poll(&buddy, 3050U);
    assert(telemetry_unpack(last_telemetry_packet, &decoded));
    assert(decoded.fault_flags == FAULT_IMU);

    fake_connection_failed = true;
    buddy1_poll(&buddy, 3051U);
    assert(buddy1_network_state(&buddy) == NET_RECONNECT_WAIT);

    puts("PASS: Buddy 1 connect/retry, telemetry, and command handling");
    return 0;
}
