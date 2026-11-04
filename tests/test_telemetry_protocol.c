#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "telemetry_protocol.h"

int main(void)
{
    telemetry_data_t in = {
        .message_type = MSG_TELEMETRY,
        .vehicle_state = VEHICLE_RUNNING,
        .line_state = LINE_FOUND,
        .obstacle_state = OBSTACLE_DETECTED,
        .terrain_state = TERRAIN_HUMP,
        .fault_flags = FAULT_IMU | FAULT_WIFI_LOSS,
        .sequence = 127U,
        .battery_pct = 87U
    };
    telemetry_data_t out;
    uint8_t packet[TELEMETRY_PACKET_SIZE];

    assert(telemetry_pack(&in, packet));
    assert(telemetry_unpack(packet, &out));
    assert(out.message_type == in.message_type);
    assert(out.vehicle_state == in.vehicle_state);
    assert(out.line_state == in.line_state);
    assert(out.obstacle_state == in.obstacle_state);
    assert(out.terrain_state == in.terrain_state);
    assert(out.fault_flags == in.fault_flags);
    assert(out.sequence == in.sequence);
    assert(out.battery_pct == in.battery_pct);
    printf("PASS: telemetry round-trip (%u bytes)\n", TELEMETRY_PACKET_SIZE);

    in.battery_pct = 101U;
    assert(!telemetry_pack(&in, packet));
    printf("PASS: invalid battery value rejected\n");
    in.battery_pct = 87U;
    in.line_state = 3U; /* reserved 2-bit encoding */
    assert(!telemetry_pack(&in, packet));
    printf("PASS: reserved telemetry state rejected\n");
    in.line_state = LINE_FOUND;

    remote_command_t cmd = { .type = CMD_STOP, .argument = 0U, .sequence = 44U };
    uint8_t command_packet[COMMAND_PACKET_SIZE];
    remote_command_t decoded;
    assert(command_pack(&cmd, command_packet));
    assert(command_unpack(command_packet, &decoded));
    assert(decoded.type == cmd.type && decoded.argument == cmd.argument &&
           decoded.sequence == cmd.sequence);
    printf("PASS: command round-trip (%u bytes)\n", COMMAND_PACKET_SIZE);

    command_packet[0] = (uint8_t)(7U << 5U);
    assert(!command_unpack(command_packet, &decoded));
    printf("PASS: invalid command rejected\n");
    puts("All Buddy 1 protocol tests passed.");
    return 0;
}
