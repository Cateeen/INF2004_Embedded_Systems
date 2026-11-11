#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_PACKET_SIZE 5U
#define COMMAND_PACKET_SIZE   2U

/* Message types: 0..7 fit in 3 bits. */
typedef enum {
    MSG_TELEMETRY = 0,
    MSG_HEARTBEAT = 1,
    MSG_FAULT = 2
} message_type_t;

/* States are examples: confirm the shared state map with Vehicle Controller. */
typedef enum {
    VEHICLE_IDLE = 0,
    VEHICLE_RUNNING = 1,
    VEHICLE_PAUSED = 2,
    VEHICLE_FAULT = 3,
    VEHICLE_COMPLETE = 4
} vehicle_state_t;

typedef enum {
    LINE_UNKNOWN = 0,
    LINE_FOUND = 1,
    LINE_LOST = 2
} line_state_t;

typedef enum {
    OBSTACLE_UNKNOWN = 0,
    OBSTACLE_CLEAR = 1,
    OBSTACLE_DETECTED = 2
} obstacle_state_t;

typedef enum {
    TERRAIN_UNKNOWN = 0,
    TERRAIN_FLAT = 1,
    TERRAIN_HUMP = 2
} terrain_state_t;

#define FAULT_MOTOR_STALL  (1u << 0)
#define FAULT_ENCODER      (1u << 1)
#define FAULT_IMU          (1u << 2)
#define FAULT_NO_ECHO      (1u << 3)
#define FAULT_WIFI_LOSS    (1u << 4)
#define FAULT_ALL          (0x1Fu)

typedef struct {
    uint8_t message_type;   /* 0..7 */
    uint8_t vehicle_state;  /* 0..7 */
    uint8_t line_state;     /* 0..3 */
    uint8_t obstacle_state; /* 0..3 */
    uint8_t terrain_state;  /* 0..3 */
    uint8_t fault_flags;    /* 5 bits */
    uint8_t sequence;       /* 7 bits, wraps at 127 */
    uint8_t battery_pct;    /* 0..100 */
} telemetry_data_t;

typedef enum {
    CMD_NONE = 0,
    CMD_START = 1,
    CMD_PAUSE = 2,
    CMD_RESUME = 3,
    CMD_STOP = 4,
    CMD_REQUEST_STATUS = 5,
    CMD_JUNCTION = 6
} command_type_t;

typedef struct {
    uint8_t type;      /* 0..7 */
    uint8_t argument;  /* 0..31 */
    uint8_t sequence;  /* 0..255 */
} remote_command_t;

bool telemetry_pack(const telemetry_data_t *data,
                    uint8_t out[TELEMETRY_PACKET_SIZE]);
bool telemetry_unpack(const uint8_t in[TELEMETRY_PACKET_SIZE],
                      telemetry_data_t *out);
bool command_pack(const remote_command_t *cmd,
                  uint8_t out[COMMAND_PACKET_SIZE]);
bool command_unpack(const uint8_t in[COMMAND_PACKET_SIZE],
                    remote_command_t *out);

#endif
