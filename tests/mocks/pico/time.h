
#ifndef PICO_TIME_H
#define PICO_TIME_H

#include <stdint.h>

/*
 * Test-only replacement for Pico SDK time_us_64().
 * The test program controls the simulated time.
 */
uint64_t time_us_64(void);

#endif
