/* car_link.h - the two "wires" between the obstacle task (obs_core) and the Vehicle Controller (vc_core). */
#ifndef CAR_LINK_H
#define CAR_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "obs_pack.h"

/* Create the two queues. Call once in main(), BEFORE starting vc_core. */
void car_link_init(void);

/* Vehicle Controller side: wait up to timeout_ms for one message from the obstacle task. */
bool vc_port_msg_recv(obs_msg_t *m, uint32_t timeout_ms);

/* Vehicle Controller side: send one command (OBS_CMD_*) to the obstacle task. */
void vc_port_cmd_send(uint8_t cmd);

/* Bench "go" switch: USB-serial key g = true, key x = false. Motors never move while false. */
bool vc_port_run_enabled(void);

#endif