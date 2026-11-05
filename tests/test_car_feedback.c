#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "car_feedback.h"

static void forward_left(void)
{
    car_feedback_left_edge(0, 1);
    car_feedback_left_edge(1, 1);
    car_feedback_left_edge(1, 0);
    car_feedback_left_edge(0, 0);
}
static void forward_right(void)
{
    car_feedback_right_edge(0, 1);
    car_feedback_right_edge(1, 1);
    car_feedback_right_edge(1, 0);
    car_feedback_right_edge(0, 0);
}
int main(void)
{
    float dist = -1.0f;
    uint8_t payload[32] = {0};
    uint8_t length = 0;
    wheel_feedback_pid_output_t output = {0};
    wheel_speed_config_t pid = {2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U};
    car_feedback_init();
    assert(!car_feedback_distance_m(&dist));
    assert(!car_feedback_configure(0, 20.0f, 4U, 0, 0, 0, 0, 0ULL, false));
    assert(car_feedback_configure(400U, 20.0f, 4U,
                                  0, 0, 0, 0, 0ULL, false));
    assert(car_feedback_pid_configure(&pid, true, true, 0ULL));
    for (int i = 0; i < 10; i++) {
        forward_left();
        forward_right();
    }
    car_feedback_line_sample(false, 100);
    assert(car_feedback_distance_m(&dist));
    assert(dist > 0.019f && dist < 0.021f);
    /* Software PID update tests numerical path; outputs not issued to motors. */
    assert(car_feedback_pid_compute(MOTION_FORWARD, 200000ULL, &output));
    /* No barcode without properly sized alternating edges. */
    assert(!car_feedback_take_barcode(payload, &length));
    puts("PASS: simulated encoder distance, barcode bridge and PID computation");
    return 0;
}
