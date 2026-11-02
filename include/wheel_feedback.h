
#ifndef WHEEL_FEEDBACK_H
#define WHEEL_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"
#include "wheel_odometry.h"

/*
 * Buddy 2 - Wheel Feedback
 *
 * Combines quadrature encoder counts with
 * wheel odometry calculations.
 *
 * No direct GPIO access.
 * Safe to test using simulated encoder signals.
 *
 * Hardware interrupts and RTOS synchronisation
 * will be integrated separately.
 */

typedef struct
{
    encoder_t encoder;

    wheel_odometry_config_t calibration;

    int32_t previous_count;

    uint64_t previous_sample_us;

    float total_distance_cm;

    wheel_odometry_result_t latest_result;

    bool initialized;
} wheel_feedback_t;

/*
 * Initialise wheel feedback.
 *
 * Requires valid encoder calibration
 * and initial channel A/B levels.
 */
bool wheel_feedback_init(
    wheel_feedback_t *wheel,
    const wheel_odometry_config_t *calibration,
    uint8_t channel_a,
    uint8_t channel_b,
    uint64_t current_time_us);

/*
 * Process a new encoder channel state.
 *
 * Intended to be called when an encoder
 * signal changes.
 */
void wheel_feedback_update_encoder(
    wheel_feedback_t *wheel,
    uint8_t channel_a,
    uint8_t channel_b);

/*
 * Calculate wheel movement since the
 * previous sample.
 *
 * current_time_us must be greater than
 * the previous sample timestamp.
 *
 * Returns false if sampling fails.
 */
bool wheel_feedback_sample(
    wheel_feedback_t *wheel,
    uint64_t current_time_us);

/* Read accumulated wheel distance in cm. */
float wheel_feedback_get_distance(
    const wheel_feedback_t *wheel);

/* Read latest calculated RPM and speed. */
bool wheel_feedback_get_result(
    const wheel_feedback_t *wheel,
    wheel_odometry_result_t *result);

/* Reset accumulated distance and count. */
void wheel_feedback_reset(
    wheel_feedback_t *wheel,
    uint64_t current_time_us);

#endif
