
#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Buddy 2 - Wheel Odometry
 *
 * Converts quadrature encoder counts into:
 * - Wheel distance (cm)
 * - Wheel speed (RPM)
 * - Wheel linear speed (cm/s)
 *
 * No Pico SDK dependencies.
 *
 * Encoder counts per revolution must include
 * all quadrature transitions and any gearbox
 * ratio between encoder shaft and wheel.
 *
 * The direction of positive counts depends
 * on encoder wiring.
 */

typedef struct
{
    uint32_t counts_per_revolution;
    float wheel_circumference_cm;
} wheel_odometry_config_t;

typedef struct
{
    float distance_cm;
    float speed_cm_per_second;
    float rpm;
} wheel_odometry_result_t;

/*
 * Calculate one wheel's movement.
 *
 * delta_counts:
 *   Signed change in encoder count.
 *
 * dt_seconds:
 *   Time elapsed during the count change.
 *
 * Returns false for invalid calibration
 * or invalid elapsed time.
 */
bool wheel_odometry_calculate(
    const wheel_odometry_config_t *config,
    int64_t delta_counts,
    float dt_seconds,
    wheel_odometry_result_t *result);

/*
 * Convert a signed encoder count into
 * a relative distance in centimetres.
 *
 * Returns false for invalid parameters.
 */
bool wheel_odometry_distance(
    const wheel_odometry_config_t *config,
    int64_t counts,
    float *distance_cm);

#endif
