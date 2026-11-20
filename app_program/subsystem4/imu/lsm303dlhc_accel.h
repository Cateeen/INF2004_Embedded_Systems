#ifndef LSM303DLHC_ACCEL_H
#define LSM303DLHC_ACCEL_H

#include "imu_types.h"

typedef struct {
	/* For each vehicle axis, select sensor X=0, Y=1, or Z=2 and sign +1/-1. */
	uint8_t axis[3];
	int8_t sign[3];
} lsm303dlhc_axis_map_t;

/* Decode six bytes from OUT_X_L_A through OUT_Z_H_A. The caller must first
 * verify a successful, fresh six-byte transfer and configure the device for
 * little-endian, normal-mode, +/-2g operation. The sensitivity and default
 * orientation match the original vehicle driver. No I2C access occurs here.
 * The explicit mapping describes sensor orientation relative to the vehicle.
 * The decoder leaves i2c_ok false; only the transport owner may set it after
 * verifying the transfer. On failure, sample is unchanged. */
bool lsm303dlhc_accel_decode(const uint8_t bytes[6],
			     const lsm303dlhc_axis_map_t *mapping,
			     accel_data_t *sample);

#endif
