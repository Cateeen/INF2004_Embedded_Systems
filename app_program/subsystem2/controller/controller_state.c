#include "controller_state.h"

#include <stddef.h>

bool controller_state_init(controller_state_t *controller,
			   const car_safety_config_t *config, uint32_t now_ms)
{
	controller_state_t initial = {0};

	if (controller == NULL || controller->initialized ||
	    !car_safety_init(&initial.safety, config, now_ms))
		return false;
	initial.previous_go = true;
	initial.initialized = true;
	*controller = initial;
	return true;
}

controller_state_value_t controller_state_step(controller_state_t *controller,
						uint32_t now_ms, bool go,
						bool emergency_stopped)
{
	bool rising, following;

	if (controller == NULL || !controller->initialized) return CONTROLLER_FAULTED;
	rising = go && !controller->previous_go;
	controller->previous_go = go;
	following = car_safety_follow(&controller->safety, now_ms, go, rising,
				      emergency_stopped);
	if (controller->safety.faults != 0U)
		controller->state = CONTROLLER_FAULTED;
	else if (controller->safety.scanning)
		controller->state = CONTROLLER_SCANNING;
	else if (following)
		controller->state = CONTROLLER_FOLLOWING;
	else if (go)
		controller->state = CONTROLLER_BLOCKED;
	else
		controller->state = CONTROLLER_WAITING;
	return controller->state;
}
