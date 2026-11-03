
#include "wheel_odometry.h"

#include <math.h>
#include <stddef.h>
#include <float.h>

static bool valid_config(
    const wheel_odometry_config_t *config)
{
    return (config != NULL) &&
           (config->counts_per_revolution > 0U) &&
           isfinite(config->wheel_circumference_cm) &&
           (config->wheel_circumference_cm > 0.0f);
}

bool wheel_odometry_distance(
    const wheel_odometry_config_t *config,
    int64_t counts,
    float *distance_cm)
{
    if ((!valid_config(config)) ||
        (distance_cm == NULL))
    {
        return false;
    }

    double distance =
        ((double)counts *
         (double)config->wheel_circumference_cm) /
        (double)config->counts_per_revolution;

    if (!isfinite(distance) ||
        (fabs(distance) > (double)FLT_MAX))
    {
        return false;
    }

    *distance_cm = (float)distance;

    return true;
}

bool wheel_odometry_calculate(
    const wheel_odometry_config_t *config,
    int64_t delta_counts,
    float dt_seconds,
    wheel_odometry_result_t *result)
{
    if ((result == NULL) ||
        (!isfinite(dt_seconds)) ||
        (dt_seconds <= 0.0f))
    {
        return false;
    }

    float distance = 0.0f;

    if (!wheel_odometry_distance(
            config,
            delta_counts,
            &distance))
    {
        return false;
    }

    double revolutions =
        (double)delta_counts /
        (double)config->counts_per_revolution;

    double rpm =
        revolutions * 60.0 /
        (double)dt_seconds;

    double speed =
        (double)distance /
        (double)dt_seconds;

    if ((!isfinite(rpm)) ||
        (!isfinite(speed)) ||
        (fabs(rpm) > (double)FLT_MAX) ||
        (fabs(speed) > (double)FLT_MAX))
    {
        return false;
    }

    result->distance_cm = distance;
    result->speed_cm_per_second = (float)speed;
    result->rpm = (float)rpm;

    return true;
}
