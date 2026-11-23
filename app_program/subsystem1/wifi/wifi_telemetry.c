#include "wifi_telemetry.h"

#include <stddef.h>

bool wifi_telemetry_make(const wifi_telemetry_data_t *data, uint32_t sequence,
			 uint32_t timestamp_ms, telemetry_message_t *message)
{
	telemetry_message_t result = {0};
	uint8_t *packet = (uint8_t *)result.payload;

	if (data == NULL || message == NULL ||
	    data->vehicle_state > WIFI_TELEMETRY_VEHICLE_COMPLETE ||
	    data->line_state > WIFI_TELEMETRY_LINE_LOST ||
	    data->obstacle_state > WIFI_TELEMETRY_OBSTACLE_DETECTED ||
	    data->terrain_state > WIFI_TELEMETRY_TERRAIN_HUMP ||
	    data->fault_flags > WIFI_TELEMETRY_FAULT_ALL || data->battery_pct > 100U)
		return false;

	result.header.type = VEHICLE_MESSAGE_TELEMETRY;
	result.header.version = VEHICLE_MESSAGE_VERSION;
	result.header.sequence = sequence;
	result.header.timestamp_ms = timestamp_ms;
	result.payload_length = WIFI_TELEMETRY_PACKET_SIZE;
	packet[0] = (uint8_t)((data->vehicle_state << 2U) | data->line_state);
	packet[1] = (uint8_t)((data->obstacle_state << 6U) |
			      (data->terrain_state << 4U) | (data->fault_flags & 0x0fU));
	packet[2] = (uint8_t)(((sequence & 0x7fU) << 1U) |
			      ((data->fault_flags >> 4U) & 1U));
	packet[3] = data->battery_pct;
	packet[4] = 0U;
	*message = result;
	return true;
}

bool wifi_telemetry_packet_valid(const uint8_t *packet, uint32_t length)
{
	return packet != NULL && length == WIFI_TELEMETRY_PACKET_SIZE &&
	       (packet[0] >> 5U) == 0U &&
	       ((packet[0] >> 2U) & 7U) <= WIFI_TELEMETRY_VEHICLE_COMPLETE &&
	       (packet[0] & 3U) <= WIFI_TELEMETRY_LINE_LOST &&
	       (packet[1] >> 6U) <= WIFI_TELEMETRY_OBSTACLE_DETECTED &&
	       ((packet[1] >> 4U) & 3U) <= WIFI_TELEMETRY_TERRAIN_HUMP &&
	       packet[3] <= 100U && packet[4] == 0U;
}
