#include "imu_driver.h"
#include "imu_config.h"
#include "imu_types.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define IMU_I2C_TIMEOUT_US 2000U /* 2 ms timeout so loose wires never freeze the Pico */

static bool lsm_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_timeout_us(IMU_I2C_PORT, dev_addr, buf, 2, false, IMU_I2C_TIMEOUT_US);
    return (ret == 2);
}

static bool lsm_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buffer, size_t len) {
    uint8_t reg_addr = reg | 0x80U; /* MSB = 1 for auto-increment across XYZ registers */
    if (i2c_write_timeout_us(IMU_I2C_PORT, dev_addr, &reg_addr, 1, true, IMU_I2C_TIMEOUT_US) != 1) {
        return false;
    }
    if (i2c_read_timeout_us(IMU_I2C_PORT, dev_addr, buffer, len, false, IMU_I2C_TIMEOUT_US) != (int)len) {
        return false;
    }
    return true;
}

void imu_driver_init(void) {
    /* GP2 (SDA) and GP3 (SCL) on i2c1 at 400 kHz */
    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    sleep_ms(10);

    /* Configure Accelerometer: 100 Hz, Normal mode, XYZ enabled, +-2g */
    lsm_write_reg(LSM303_ACCEL_ADDR, LSM303_REG_CTRL1_A, LSM303_CTRL1_100HZ);
    lsm_write_reg(LSM303_ACCEL_ADDR, LSM303_REG_CTRL4_A, LSM303_CTRL4_2G);

    /* Configure Magnetometer */
    lsm_write_reg(LSM303_MAG_ADDR, LSM303_REG_CRA_M, LSM303_CRA_M_15HZ);
    lsm_write_reg(LSM303_MAG_ADDR, LSM303_REG_CRB_M, LSM303_CRB_M_1_3G);
    lsm_write_reg(LSM303_MAG_ADDR, LSM303_REG_MR_M, LSM303_MR_M_CONT);

    sleep_ms(50);
}

void imu_driver_read_accel(accel_data_t *out) {
    uint8_t buf[6] = {0};
    if (!lsm_read_regs(LSM303_ACCEL_ADDR, LSM303_REG_OUT_X_L_A, buf, 6)) {
        out->i2c_ok = false;
        return;
    }
    out->i2c_ok = true;

    int16_t raw_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    int16_t raw_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    int16_t raw_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));

    out->raw_x = raw_x >> 4;
    out->raw_y = raw_y >> 4;
    out->raw_z = raw_z >> 4;

    out->ax = -(out->raw_x * ACCEL_SENS_2G * GRAVITY_MS2);
    out->ay =  (out->raw_y * ACCEL_SENS_2G * GRAVITY_MS2);
    out->az = -(out->raw_z * ACCEL_SENS_2G * GRAVITY_MS2);
}