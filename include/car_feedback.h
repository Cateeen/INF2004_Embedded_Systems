#ifndef CAR_FEEDBACK_H
#define CAR_FEEDBACK_H
#include <stdbool.h>
#include <stdint.h>
#include "wheel_feedback_pid.h"
/* Observer-only integration: does not command motors. Initially no encoder
   hardware pins are assigned: caller must supply both A/B channel transitions.
   Calibration required before turning on wheel/barcode distances. */
void car_feedback_init(void);
bool car_feedback_configure(uint32_t counts_per_rev, float circumference_cm,
                            uint32_t barcode_counts_per_module,
                            uint8_t initial_left_a, uint8_t initial_left_b,
                            uint8_t initial_right_a, uint8_t initial_right_b,
                            uint64_t now_us, bool initial_barcode_black);
void car_feedback_left_edge(uint8_t a, uint8_t b);
void car_feedback_right_edge(uint8_t a, uint8_t b);
void car_feedback_line_sample(bool barcode_black, uint32_t now_ms);
bool car_feedback_distance_m(float *out);
bool car_feedback_take_barcode(uint8_t payload[32], uint8_t *length);
/* PID outputs are calculated, but NEVER applied to the motors by this bridge. */
bool car_feedback_pid_configure(const wheel_speed_config_t *speed,
                                bool left_counts_forward, bool right_counts_forward,
                                uint64_t now_us);
bool car_feedback_pid_compute(motion_direction_t direction, uint64_t now_us,
                               wheel_feedback_pid_output_t *out);
#endif
