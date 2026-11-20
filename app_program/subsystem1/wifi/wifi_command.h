#ifndef WIFI_COMMAND_H
#define WIFI_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#define WIFI_COMMAND_PACKET_SIZE 2U

typedef enum {
	WIFI_COMMAND_START = 1,
	WIFI_COMMAND_PAUSE,
	WIFI_COMMAND_RESUME,
	WIFI_COMMAND_STOP,
	WIFI_COMMAND_REQUEST_STATUS,
	WIFI_COMMAND_JUNCTION
} wifi_command_type_t;

typedef struct {
	uint8_t type;
	uint8_t argument;
	uint8_t sequence;
} wifi_command_t;

/* Decode the legacy two-byte dashboard packet. The sequence is eight bits and
 * wraps at 255. This format has no authentication or sender timestamp; a
 * successful decode does not establish freshness, authority, or permission to
 * move. Argument meaning must be validated by the vehicle controller.
 * The output is unchanged on failure. */
bool wifi_command_decode(const uint8_t *packet, uint32_t length,
			 wifi_command_t *command);

#endif
