/* 
Declarations for the signal processing state machine logic
Separate math and logic from hardware 
*/

#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "imu_types.h"

void imu_processor_init(imu_state_t *state);
void imu_processor_update(imu_state_t *state, accel_data_t *accel, float current_distance_m);

#endif