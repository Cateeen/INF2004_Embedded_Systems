#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Counts include all quadrature transitions and the gearbox ratio. */
typedef struct {
	uint32_t counts_per_revolution;
	float wheel_circumference_cm;
} wheel_odometry_config_t;

typedef struct {
	float distance_cm;
	float speed_cm_per_second;
	float rpm;
} wheel_odometry_result_t;

/* Reject the ambiguous half-range change between wrapping 32-bit counts. */
bool wheel_odometry_count_delta(uint32_t current, uint32_t previous, int32_t *delta);

bool wheel_odometry_distance(const wheel_odometry_config_t *config,
			     int64_t counts, float *distance_cm);
bool wheel_odometry_distance_precise(const wheel_odometry_config_t *config,
				     int64_t counts, double *distance_cm);
bool wheel_odometry_calculate(const wheel_odometry_config_t *config,
			      int64_t delta_counts, float dt_seconds,
			      wheel_odometry_result_t *result);

#endif
