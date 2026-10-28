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
#define LSM303_CTRL4_4G     0x98
#define LSM303_CRA_M_15HZ   0x10
#define LSM303_CRB_M_1_3G   0x20
#define LSM303_MR_M_CONT    0x00

// Algorithm Tunables
#define GRAVITY_MS2         9.80665f
#define ACCEL_SENS_4G       0.001197f   // g/LSB
#define HUMP_THRESHOLD      0.3f        // m/s2
#define SHOCK_THRESHOLD     2.0f        // g-force
#define TILT_THRESHOLD_DEG  30.0f
#define ALPHA_HP            0.85f
#define ALPHA_LP            0.20f

#endif // IMU_CONFIG_H
