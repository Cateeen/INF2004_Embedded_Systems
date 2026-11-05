#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

// I2C Config
#define IMU_I2C_PORT        i2c1
#define IMU_I2C_SDA_PIN     2
#define IMU_I2C_SCL_PIN     3
#define IMU_I2C_BAUDRATE    400000

// LSM303DLHC Addresses
#define LSM303_ACCEL_ADDR   0x19
#define LSM303_MAG_ADDR     0x1E

//LSM303DLHC Register Addresses
#define LSM303_REG_CTRL1_A      0x20
#define LSM303_REG_CTRL4_A      0x23
#define LSM303_REG_OUT_X_L_A    0x28
#define LSM303_REG_CRA_M        0x00
#define LSM303_REG_CRB_M        0x01
#define LSM303_REG_MR_M         0x02

// Sensor Configuration Values
#define LSM303_CTRL1_100HZ  0x57
#define LSM303_CTRL4_2G     0x00
#define LSM303_CRA_M_15HZ   0x10
#define LSM303_CRB_M_1_3G   0x20
#define LSM303_MR_M_CONT    0x00

// Algorithm Tunables
#define GRAVITY_MS2         9.80665f
#define ACCEL_SENS_2G       0.00105f    // g/LSB
#define SHOCK_THRESHOLD     3.0f        // g-force
#define TILT_THRESHOLD_DEG  30.0f       // safety threshold

#define PITCH_START_DEG     3.0f
#define PITCH_END_DEG       1.5f
#define MIN_HUMP_HEIGHT_M   0.005f

// Buddy 2 needs to verify this number (if timothy u integrating it take note of this)
#define TICKS_PER_METER     1000.0f

#endif // IMU_CONFIG_H