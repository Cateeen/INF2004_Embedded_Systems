#include "wifi_command.h"

#include <stddef.h>

bool wifi_command_decode(const uint8_t *packet, uint32_t length,
			 wifi_command_t *command)
{
	wifi_command_t decoded;

	if (packet == NULL || command == NULL || length != WIFI_COMMAND_PACKET_SIZE)
		return false;
	decoded.type = packet[0] >> 5U;
	if (decoded.type < WIFI_COMMAND_START || decoded.type > WIFI_COMMAND_JUNCTION)
		return false;
	decoded.argument = packet[0] & 0x1fU;
	decoded.sequence = packet[1];
	*command = decoded;
	return true;
}
