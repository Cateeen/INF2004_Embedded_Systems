/* car_link.h - the two "wires" between the obstacle task (obs_core) and the Vehicle Controller (vc_core). */
#ifndef CAR_LINK_H
#define CAR_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "obs_pack.h"

/*
 * Integration-level fault bits.
 * Separate from Buddy 5's four packed OBS_FAULT_* bits
 * and Buddy 1's telemetry fault-bit definitions.
 */
#define CAR_FAULT_QUEUE_OVERFLOW (1UL << 16)

/* Create the two queues. Call once in main(), BEFORE starting vc_core. */
void car_link_init(void);

/* Vehicle Controller side: wait up to timeout_ms for one message from the obstacle task. */
bool vc_port_msg_recv(obs_msg_t *m, uint32_t timeout_ms);

/* Vehicle Controller side: send one command (OBS_CMD_*) to the obstacle task. */
void vc_port_cmd_send(uint8_t cmd);

/* Bench "go" switch: USB-serial key g = true, key x = false. Motors never move while false. */
bool vc_port_run_enabled(void);
void vc_port_set_run(bool enabled);
bool vc_port_motion_recv(uint8_t *motion);
void vc_port_motion_send(uint8_t motion);

/*
 * Cross-core safety status.
 * Fault bits remain latched until a validated reset mechanism
 * is implemented.
 */
uint32_t car_fault_bits_get(void);
bool car_obstacle_present(void);

#endif