#include "line_observation.h"

#include <stddef.h>

uint8_t line_following_position(const ir_readings_t *readings)
{
	if (readings == NULL) {
		return POS_LOST;
	}
	if (readings->center.black && readings->right.black) {
		return POS_JUNCTION;
	}
	if (readings->center.black) {
		return POS_LEFT;
	}
	if (readings->right.black) {
		return POS_RIGHT;
	}
	return POS_LOST;
}
