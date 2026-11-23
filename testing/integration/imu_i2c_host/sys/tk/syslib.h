#ifndef IMU_HOST_SYSLIB_H
#define IMU_HOST_SYSLIB_H

#include <stdint.h>

extern uint32_t imu_test_timer_us;

static inline uint32_t in_w(unsigned address)
{
	(void)address;
	return imu_test_timer_us;
}

#endif
