#include "wheel_odometry.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

bool wheel_odometry_count_delta(uint32_t current, uint32_t previous, int32_t *delta)
{
	uint32_t difference = current - previous;
	if (delta == NULL || difference == 0x80000000U) return false;
	*delta = difference <= INT32_MAX ? (int32_t)difference :
		 INT32_MIN + (int32_t)(difference - 0x80000000U);
	return true;
}

static bool valid_config(const wheel_odometry_config_t *config)
{
	return config != NULL &&
	       config->counts_per_revolution > 0U &&
	       isfinite(config->wheel_circumference_cm) &&
	       config->wheel_circumference_cm > 0.0f;
}

bool wheel_odometry_distance_precise(const wheel_odometry_config_t *config,
				     int64_t counts, double *distance_cm)
{
	double distance;

	if (!valid_config(config) || distance_cm == NULL) {
		return false;
	}
	distance = ((double)counts * (double)config->wheel_circumference_cm) /
		   (double)config->counts_per_revolution;
	if (!isfinite(distance)) {
		return false;
	}
	*distance_cm = distance;
	return true;
}

bool wheel_odometry_distance(const wheel_odometry_config_t *config,
			     int64_t counts, float *distance_cm)
{
	double distance;

	if (distance_cm == NULL ||
	    !wheel_odometry_distance_precise(config, counts, &distance) ||
	    distance > (double)FLT_MAX || distance < -(double)FLT_MAX) {
		return false;
	}
	*distance_cm = (float)distance;
	return true;
}

bool wheel_odometry_calculate(const wheel_odometry_config_t *config,
			      int64_t delta_counts, float dt_seconds,
			      wheel_odometry_result_t *result)
{
	float distance;
	double revolutions;
	double rpm;
	double speed;

	if (result == NULL || !isfinite(dt_seconds) || dt_seconds <= 0.0f ||
	    !wheel_odometry_distance(config, delta_counts, &distance)) {
		return false;
	}
	revolutions = (double)delta_counts / (double)config->counts_per_revolution;
	rpm = revolutions * 60.0 / (double)dt_seconds;
	speed = (double)distance / (double)dt_seconds;
	if (!isfinite(rpm) || !isfinite(speed) ||
	    rpm > (double)FLT_MAX || rpm < -(double)FLT_MAX ||
	    speed > (double)FLT_MAX || speed < -(double)FLT_MAX) {
		return false;
	}
	result->distance_cm = distance;
	result->speed_cm_per_second = (float)speed;
	result->rpm = (float)rpm;
	return true;
}
