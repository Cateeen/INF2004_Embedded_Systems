/* 
Declarations for the signal processing state machine logic
Separate math and logic from hardware 
*/

#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "imu_types.h"

/* Each state is owned by one IMU task. The caller must reject failed I2C
 * samples and supply monotonic, wrapping milliseconds and odometry metres. */
bool imu_processor_init(imu_state_t *state);
bool imu_processor_update(imu_state_t *state, const accel_data_t *accel,
                          float current_distance_m, uint32_t now_ms);

#endif