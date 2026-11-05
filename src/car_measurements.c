#include "car_measurements.h"
#include "pico/critical_section.h"
#include <math.h>
#include <stddef.h>
static critical_section_t s_lock;
static float s_meters;
static bool s_valid;
void car_measurements_init(void)
{
    critical_section_init(&s_lock);
    s_meters = 0.0f;
    s_valid = false;
}
void car_measurements_publish_distance(float meters)
{
    if (!isfinite(meters)) return;
    critical_section_enter_blocking(&s_lock);
    s_meters = meters;
    s_valid = true;
    critical_section_exit(&s_lock);
}
bool car_measurements_read_distance(float *meters)
{
    if (meters == NULL) return false;
    critical_section_enter_blocking(&s_lock);
    bool valid = s_valid;
    *meters = s_meters;
    critical_section_exit(&s_lock);
    return valid;
}
