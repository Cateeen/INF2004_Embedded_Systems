#include "lsm303dlhc_accel.h"
#include "imu_processor_config.h"

#include <stddef.h>

bool lsm303dlhc_accel_decode(const uint8_t bytes[6],
			     const lsm303dlhc_axis_map_t *mapping,
			     accel_data_t *sample)
{
	int16_t sensor[3];
	int16_t vehicle[3];
	accel_data_t result = {0};
	unsigned i;

	if (bytes == NULL || mapping == NULL || sample == NULL ||
	    mapping->axis[0] > 2U || mapping->axis[1] > 2U || mapping->axis[2] > 2U ||
	    mapping->axis[0] == mapping->axis[1] ||
	    mapping->axis[0] == mapping->axis[2] ||
	    mapping->axis[1] == mapping->axis[2])
		return false;
	for (i = 0U; i < 3U; i++) {
		uint16_t bits = (uint16_t)bytes[2U * i] |
				((uint16_t)bytes[2U * i + 1U] << 8U);
		int32_t value;
		if ((bits & 0x000FU) != 0U ||
		    (mapping->sign[i] != 1 && mapping->sign[i] != -1))
			return false;
		value = (int32_t)(bits >> 4U);
		if (value >= 2048) value -= 4096;
		sensor[i] = (int16_t)value;
	}
	for (i = 0U; i < 3U; i++)
		vehicle[i] = (int16_t)(sensor[mapping->axis[i]] * mapping->sign[i]);
	result.raw_x = vehicle[0];
	result.raw_y = vehicle[1];
	result.raw_z = vehicle[2];
	result.ax = (float)vehicle[0] *
		    (IMU_ACCEL_SENSITIVITY_2G_G_PER_LSB * GRAVITY_MS2);
	result.ay = (float)vehicle[1] *
		    (IMU_ACCEL_SENSITIVITY_2G_G_PER_LSB * GRAVITY_MS2);
	result.az = (float)vehicle[2] *
		    (IMU_ACCEL_SENSITIVITY_2G_G_PER_LSB * GRAVITY_MS2);
	*sample = result;
	return true;
}
