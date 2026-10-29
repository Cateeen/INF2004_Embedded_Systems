#ifndef BUDDY1_H
#define BUDDY1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "telemetry_protocol.h"

#define BUDDY1_COMMAND_QUEUE_CAPACITY 8U

typedef enum {
    NET_DISCONNECTED = 0,
    NET_CONNECTING = 1,
    NET_CONNECTED = 2,
    NET_RECONNECT_WAIT = 3
} network_state_t;

typedef struct {
    telemetry_data_t latest;
    uint32_t fault_flags;
    network_state_t network_state;
    uint32_t next_retry_ms;
    uint32_t last_publish_ms;
    uint8_t command_head;
    uint8_t command_tail;
    uint8_t command_count;
    remote_command_t command_queue[BUDDY1_COMMAND_QUEUE_CAPACITY];
    bool wifi_fault_pending;
    bool initialised;
} buddy1_t;

void buddy1_init(buddy1_t *b, uint32_t now_ms);
void buddy1_set_telemetry(buddy1_t *b, const telemetry_data_t *data);
void buddy1_set_faults(buddy1_t *b, uint8_t fault_flags);
void buddy1_poll(buddy1_t *b, uint32_t now_ms);
bool buddy1_take_command(buddy1_t *b, remote_command_t *out);
network_state_t buddy1_network_state(const buddy1_t *b);

#endif
