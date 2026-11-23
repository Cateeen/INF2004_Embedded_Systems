#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include "car_safety.h"

typedef enum {
	CONTROLLER_WAITING = 0,
	CONTROLLER_FOLLOWING,
	CONTROLLER_BLOCKED,
	CONTROLLER_SCANNING,
	CONTROLLER_FAULTED
} controller_state_value_t;

typedef struct {
	car_safety_t safety;
	controller_state_value_t state;
	bool previous_go;
	bool initialized;
} controller_state_t;

/* Single controller owner only. Initialize a zeroed object once; do not reset
 * it during operation. The owner feeds observations into safety and calls
 * controller_state_step every cycle, including when no messages arrive. */
bool controller_state_init(controller_state_t *controller,
			   const car_safety_config_t *config, uint32_t now_ms);

/* Advisory simulation only: go must be a current trusted local input, not a
 * decoded network command. A go level already high at startup cannot arm.
 * The returned state does not send commands, stop motors, request scans or
 * confirm a physical stop; do not use it as an independent safety system. */
controller_state_value_t controller_state_step(controller_state_t *controller,
						uint32_t now_ms, bool go,
						bool emergency_stopped);

#endif
