
#include "wheel_feedback.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <float.h>

/*
 * Buddy 2 - Wheel Feedback
 *
 * Encoder signal changes update the quadrature count.
 * Periodic sampling converts count differences into
 * signed distance, wheel speed and RPM.
 *
 * This module does not access hardware GPIOs.
 * GPIO interrupts and task synchronisation are
 * separate integration responsibilities.
 */

static bool calibration_is_valid(
    const wheel_odometry_config_t *calibration)
{
    return (calibration != NULL) &&
           (calibration->counts_per_revolution > 0U) &&
           isfinite(calibration->wheel_circumference_cm) &&
           (calibration->wheel_circumference_cm > 0.0f);
}

bool wheel_feedback_init(
    wheel_feedback_t *wheel,
    const wheel_odometry_config_t *calibration,
    uint8_t channel_a,
    uint8_t channel_b,
    uint64_t current_time_us)
{
    if ((wheel == NULL) ||
        (!calibration_is_valid(calibration)))
    {
        return false;
    }

    encoder_init(&wheel->encoder, channel_a, channel_b);

    wheel->calibration = *calibration;
    wheel->previous_count = 0;
    wheel->previous_sample_us = current_time_us;

    wheel->total_distance_cm = 0.0f;

    wheel->latest_result.distance_cm = 0.0f;
    wheel->latest_result.speed_cm_per_second = 0.0f;
    wheel->latest_result.rpm = 0.0f;

    wheel->initialized = true;

    return true;
}

void wheel_feedback_update_encoder(
    wheel_feedback_t *wheel,
    uint8_t channel_a,
    uint8_t channel_b)
{
    if ((wheel == NULL) || (!wheel->initialized))
    {
        return;
    }

    encoder_update(
        &wheel->encoder,
        channel_a,
        channel_b);
}

bool wheel_feedback_sample(
    wheel_feedback_t *wheel,
    uint64_t current_time_us)
{
    if ((wheel == NULL) || (!wheel->initialized))
    {
        return false;
    }

    if (current_time_us <= wheel->previous_sample_us)
    {
        return false;
    }

    int32_t current_count =
        encoder_get_count(&wheel->encoder);

    /*
     * Promote values before subtraction to prevent
     * signed 32-bit subtraction overflow.
     */
    int64_t delta_counts =
        (int64_t)current_count -
        (int64_t)wheel->previous_count;

    uint64_t elapsed_us =
        current_time_us - wheel->previous_sample_us;

    double elapsed_seconds =
        (double)elapsed_us / 1000000.0;

    if ((!isfinite(elapsed_seconds)) ||
        (elapsed_seconds <= 0.0) ||
        (elapsed_seconds > (double)FLT_MAX))
    {
        return false;
    }

    wheel_odometry_result_t result = {0};

    if (!wheel_odometry_calculate(
            &wheel->calibration,
            delta_counts,
            (float)elapsed_seconds,
            &result))
    {
        return false;
    }

    float updated_distance =
        wheel->total_distance_cm + result.distance_cm;

    if (!isfinite(updated_distance))
    {
        return false;
    }

    wheel->latest_result = result;
    wheel->total_distance_cm = updated_distance;

    wheel->previous_count = current_count;
    wheel->previous_sample_us = current_time_us;

    return true;
}

float wheel_feedback_get_distance(
    const wheel_feedback_t *wheel)
{
    if ((wheel == NULL) || (!wheel->initialized))
    {
        return 0.0f;
    }

    return wheel->total_distance_cm;
}

bool wheel_feedback_get_result(
    const wheel_feedback_t *wheel,
    wheel_odometry_result_t *result)
{
    if ((wheel == NULL) ||
        (!wheel->initialized) ||
        (result == NULL))
    {
        return false;
    }

    *result = wheel->latest_result;

    return true;
}

void wheel_feedback_reset(
    wheel_feedback_t *wheel,
    uint64_t current_time_us)
{
    if ((wheel == NULL) || (!wheel->initialized))
    {
        return;
    }

    encoder_reset_count(&wheel->encoder);

    wheel->previous_count = 0;
    wheel->previous_sample_us = current_time_us;

    wheel->total_distance_cm = 0.0f;

    wheel->latest_result.distance_cm = 0.0f;
    wheel->latest_result.speed_cm_per_second = 0.0f;
    wheel->latest_result.rpm = 0.0f;
}
