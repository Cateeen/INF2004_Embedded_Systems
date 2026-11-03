#include "imu_driver.h"
#include "imu_config.h"
#include "imu_types.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// helper functions
static void lsm_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(IMU_I2C_PORT, dev_addr, buf, 2, false);
}

// reads multiple bytes starting from a specific register
static void lsm_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buffer, size_t len) {
    uint8_t reg_addr = reg | 0x80;
    i2c_write_blocking(IMU_I2C_PORT, dev_addr, &reg_addr, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, dev_addr, buffer, len, false);
}

void imu_driver_init(void) {
    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    sleep_ms(10);

    // config accelerometer
    lsm_write_reg(LSM303_ACCEL_ADDR, LSM303_REG_CTRL1_A, LSM303_CTRL1_100HZ);
    lsm_write_reg(LSM303_ACCEL_ADDR, LSM303_REG_CTRL4_A, LSM303_CTRL4_2G);

    // config magnetometer
    lsm_write_reg(LSM303_MAG_ADDR, LSM303_REG_CRA_M, LSM303_CRA_M_15HZ);
    lsm_write_reg(LSM303_MAG_ADDR, LSM303_REG_CRB_M, LSM303_CRB_M_1_3G);
    lsm_write_reg(LSM303_MAG_ADDR, LSM303_REG_MR_M, LSM303_MR_M_CONT);

    sleep_ms(50);
}

void imu_driver_read_accel(accel_data_t *out) {
    uint8_t buf[6];
    lsm_read_regs(LSM303_ACCEL_ADDR, LSM303_REG_OUT_X_L_A, buf, 6);

    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

    out->raw_x = raw_x >> 4;
    out->raw_y = raw_y >> 4;
    out->raw_z = raw_z >> 4;

    // 3. Apply calibration factor and axis inversions
    out->ax = -(out->raw_x * ACCEL_SENS_2G * GRAVITY_MS2);
    out->ay = (out->raw_y * ACCEL_SENS_2G * GRAVITY_MS2);
    out->az = -(out->raw_z * ACCEL_SENS_2G * GRAVITY_MS2);
}
