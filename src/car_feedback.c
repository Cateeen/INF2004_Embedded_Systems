#include "car_feedback.h"
#include "buddy3.h"
#include "wheel_feedback.h"
#include <stddef.h>
#include <string.h>

static wheel_feedback_t s_left, s_right;
static barcode_scanner_t s_barcode;
static wheel_feedback_pid_t s_pid;
static bool s_ready;
static uint32_t s_last_ms;

void car_feedback_init(void)
{
    memset(&s_left, 0, sizeof(s_left));
    memset(&s_right, 0, sizeof(s_right));
    memset(&s_barcode, 0, sizeof(s_barcode));
    memset(&s_pid, 0, sizeof(s_pid));
    s_ready = false;
    s_last_ms = 0U;
}

bool car_feedback_configure(uint32_t counts_per_rev, float circumference_cm,
                            uint32_t barcode_counts_per_module,
                            uint8_t left_a, uint8_t left_b,
                            uint8_t right_a, uint8_t right_b,
                            uint64_t now_us, bool initial_black)
{
    wheel_odometry_config_t wheel_cfg = {counts_per_rev, circumference_cm};
    barcode_scanner_config_t bar_cfg = {barcode_counts_per_module, 25U};
    s_ready = false;
    memset(&s_pid, 0, sizeof(s_pid));
    if (!wheel_feedback_init(&s_left, &wheel_cfg, left_a, left_b, now_us) ||
        !wheel_feedback_init(&s_right, &wheel_cfg, right_a, right_b, now_us) ||
        !barcode_scanner_init(&s_barcode, &bar_cfg, initial_black, 0))
        return false;
    s_ready = true;
    return true;
}

void car_feedback_left_edge(uint8_t a, uint8_t b)
{
    if (s_ready) wheel_feedback_update_encoder(&s_left, a, b);
}
void car_feedback_right_edge(uint8_t a, uint8_t b)
{
    if (s_ready) wheel_feedback_update_encoder(&s_right, a, b);
}

void car_feedback_line_sample(bool black, uint32_t now_ms)
{
    if (!s_ready) return;
    (void)barcode_scanner_update(&s_barcode, black,
                                 encoder_get_count(&s_left.encoder));
    /* Distance sampling from microsecond timestamps; no guessed wheel calibration. */
    if ((uint32_t)(now_ms - s_last_ms) >= 50U) {
        const uint64_t now_us = (uint64_t)now_ms * 1000ULL;
        (void)wheel_feedback_sample(&s_left, now_us);
        (void)wheel_feedback_sample(&s_right, now_us);
        s_last_ms = now_ms;
    }
}

bool car_feedback_distance_m(float *out)
{
    if (!s_ready || out == NULL) return false;
    *out = 0.005f * (wheel_feedback_get_distance(&s_left) +
                     wheel_feedback_get_distance(&s_right));
    return true;
}

bool car_feedback_take_barcode(uint8_t payload[32], uint8_t *length)
{
    if (!s_ready) return false;
    return barcode_scanner_take_result(&s_barcode, payload, length);
}

bool car_feedback_pid_configure(const wheel_speed_config_t *speed,
                                bool left_forward, bool right_forward,
                                uint64_t now_us)
{
    if (!s_ready) return false;
    return wheel_feedback_pid_init(&s_pid, &s_left, &s_right,
                                   speed, left_forward, right_forward, now_us);
}

bool car_feedback_pid_compute(motion_direction_t direction, uint64_t now_us,
                               wheel_feedback_pid_output_t *out)
{
    if (!s_ready || !s_pid.initialized || out == NULL) return false;
    return wheel_feedback_pid_update(&s_pid, direction, now_us, out);
}
