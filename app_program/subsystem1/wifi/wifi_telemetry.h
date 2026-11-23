#ifndef WIFI_TELEMETRY_H
#define WIFI_TELEMETRY_H

#include "../../vehicle_messages.h"

#define WIFI_TELEMETRY_PACKET_SIZE 5U

typedef enum {
	WIFI_TELEMETRY_VEHICLE_IDLE = 0,
	WIFI_TELEMETRY_VEHICLE_RUNNING,
	WIFI_TELEMETRY_VEHICLE_PAUSED,
	WIFI_TELEMETRY_VEHICLE_FAULT,
	WIFI_TELEMETRY_VEHICLE_COMPLETE
} wifi_telemetry_vehicle_state_t;

typedef enum {
	WIFI_TELEMETRY_LINE_UNKNOWN = 0,
	WIFI_TELEMETRY_LINE_FOUND,
	WIFI_TELEMETRY_LINE_LOST
} wifi_telemetry_line_state_t;

typedef enum {
	WIFI_TELEMETRY_OBSTACLE_UNKNOWN = 0,
	WIFI_TELEMETRY_OBSTACLE_CLEAR,
	WIFI_TELEMETRY_OBSTACLE_DETECTED
} wifi_telemetry_obstacle_state_t;

typedef enum {
	WIFI_TELEMETRY_TERRAIN_UNKNOWN = 0,
	WIFI_TELEMETRY_TERRAIN_FLAT,
	WIFI_TELEMETRY_TERRAIN_HUMP
} wifi_telemetry_terrain_state_t;

#define WIFI_TELEMETRY_FAULT_MOTOR_STALL 0x01U
#define WIFI_TELEMETRY_FAULT_ENCODER     0x02U
#define WIFI_TELEMETRY_FAULT_IMU         0x04U
#define WIFI_TELEMETRY_FAULT_NO_ECHO     0x08U
#define WIFI_TELEMETRY_FAULT_WIFI_LOSS   0x10U
#define WIFI_TELEMETRY_FAULT_ALL         0x1fU

typedef struct {
	uint8_t vehicle_state;
	uint8_t line_state;
	uint8_t obstacle_state;
	uint8_t terrain_state;
	uint8_t fault_flags;
	uint8_t battery_pct;
} wifi_telemetry_data_t;

/* The seven-bit wire sequence is derived from the low bits of sequence.
 * timestamp_ms must be the operating-clock timestamp of the input data.
 * No unknown-battery value exists in this legacy format. The caller must
 * supply verified data and check the result; output is unchanged on failure. */
bool wifi_telemetry_make(const wifi_telemetry_data_t *data, uint32_t sequence,
			 uint32_t timestamp_ms, telemetry_message_t *message);

bool wifi_telemetry_packet_valid(const uint8_t *packet, uint32_t length);

#endif
