#ifndef PICO_STDLIB_H
#define PICO_STDLIB_H
#include <stdint.h>
uint32_t test_imu_clock_ms(void);
typedef uint32_t absolute_time_t;
static inline absolute_time_t get_absolute_time(void) { return test_imu_clock_ms(); }
static inline uint32_t to_ms_since_boot(absolute_time_t t) { return t; }
#endif
