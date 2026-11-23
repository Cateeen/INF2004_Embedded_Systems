#ifndef LSM303DLHC_I2C_H
#define LSM303DLHC_I2C_H

#include "imu_task.h"
#include "lsm303dlhc_reader.h"

typedef enum {
	LSM303DLHC_I2C_PROFILE_DLHC = 0,
	LSM303DLHC_I2C_PROFILE_32_FAMILY
} lsm303dlhc_i2c_profile_t;

typedef struct {
	uint8_t address;
	uint32_t data_ready_timeout_ms;
	lsm303dlhc_axis_map_t mapping;
	lsm303dlhc_i2c_profile_t profile;
} lsm303dlhc_i2c_config_t;

typedef struct {
	lsm303dlhc_reader_t reader;
	ID device;
	uint8_t address;
	uint32_t data_ready_timeout_ms;
	uint32_t next_sample_not_before_ms;
	bool opened;
	bool initialized;
	bool measurement_may_be_active;
	bool faulted;
} lsm303dlhc_i2c_t;

/* Import the original DLHC profile, address, orientation and ±2g behavior.
 * The readiness timeout is caller-selected and must be 1..100 ms. */
ER lsm303dlhc_i2c_default_config(lsm303dlhc_i2c_config_t *config,
				 uint32_t data_ready_timeout_ms);

/* Pass a zero-initialized context. Opens the kernel I2C1 device "iicb"
 * exclusively and configures the selected accelerometer profile for 100 Hz,
 * normal-mode +/-2g operation. The 0x32-family profile is experimental and
 * requires hardware validation; it does not verify identity. The caller owns
 * this context until close or reboot. I2C must be enabled in config_device.h
 * and the bus/device must be
 * exclusively owned. */
ER lsm303dlhc_i2c_init(lsm303dlhc_i2c_t *sensor,
		       const lsm303dlhc_i2c_config_t *config);

/* Convenience startup path. Sensor initialization and reads run through the
 * IMU task using a static-lifetime context. This function is not called by
 * default. */
ER lsm303dlhc_i2c_start_imu_task(const lsm303dlhc_i2c_config_t *sensor_config,
				const imu_task_config_t *task_config);

/* Compatible with imu_task_config_t.read. Waits for fresh XYZ data with a
 * bounded data-ready wait; kernel I2C transaction timeout is separate.
 * Any transport/configuration error latches this reader until restart. */
ER lsm303dlhc_i2c_read(void *context, accel_data_t *sample, uint32_t *timestamp_ms);

/* Call only after the IMU task has stopped using the context. */
ER lsm303dlhc_i2c_close(lsm303dlhc_i2c_t *sensor);

#endif
