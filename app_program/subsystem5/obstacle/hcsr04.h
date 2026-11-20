#ifndef HCSR04_H
#define HCSR04_H

#include <stdbool.h>
#include <stdint.h>

/* Convert the measured Echo high-pulse width to round-trip-corrected
 * millimetres. Caller supplies sound speed in mm/s (for example 343000 at
 * approximately 20 C). The output is unchanged for invalid/unrepresentable
 * input and OBS_DIST_NONE is never returned as a distance. */
bool hcsr04_distance_mm(uint32_t echo_high_us, uint32_t sound_speed_mm_s,
			uint16_t *distance_mm);

#endif
