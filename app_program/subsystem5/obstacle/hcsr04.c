#include "hcsr04.h"

#include "obs_profile_config.h"

#define HCSR04_MIN_SOUND_SPEED_MM_S 300000U
#define HCSR04_MAX_SOUND_SPEED_MM_S 400000U
#define HCSR04_ROUND_TRIP_DIVISOR 2000000ULL

bool hcsr04_distance_mm(uint32_t echo_high_us, uint32_t sound_speed_mm_s,
			uint16_t *distance_mm)
{
	uint64_t distance;

	if (distance_mm == 0 || echo_high_us == 0U ||
	    sound_speed_mm_s < HCSR04_MIN_SOUND_SPEED_MM_S ||
	    sound_speed_mm_s > HCSR04_MAX_SOUND_SPEED_MM_S)
		return false;
	distance = ((uint64_t)echo_high_us * sound_speed_mm_s +
		    HCSR04_ROUND_TRIP_DIVISOR / 2U) /
		   HCSR04_ROUND_TRIP_DIVISOR;
	if (distance == 0U || distance >= OBS_DIST_NONE ||
	    distance > UINT16_MAX)
		return false;
	*distance_mm = (uint16_t)distance;
	return true;
}
