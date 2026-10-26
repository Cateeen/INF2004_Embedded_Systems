// declarations for low-level hardware interaction

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "imu_types.h"

void imu_driver_init(void);
void imu_driver_read_accel(accel_data_t *out);

#endif