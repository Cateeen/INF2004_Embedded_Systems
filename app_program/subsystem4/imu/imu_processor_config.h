#ifndef IMU_PROCESSOR_CONFIG_H
#define IMU_PROCESSOR_CONFIG_H

#define GRAVITY_MS2        9.80665f
#define IMU_ACCEL_SENSITIVITY_2G_G_PER_LSB 0.00105f

#define LIFT_ROLL_DEG      15.0f // catch hand pickup; flat track stays ~0 deg
#define LIFT_PITCH_DEG     35.0f
#define OVERTURN_MIN_AZ    6.5f // catches gentle lift deceleration

#define TILT_THRESHOLD_DEG 30.0f

#define PITCH_START_DEG    3.0f
#define PITCH_END_DEG      1.5f
#define MIN_HUMP_HEIGHT_M  0.005f

#endif
