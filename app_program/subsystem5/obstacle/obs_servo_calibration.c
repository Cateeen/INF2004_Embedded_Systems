#include "obs_servo_calibration.h"

#include <stddef.h>

bool obs_servo_pulse(const obs_servo_calibration_t *calibration,
		     uint8_t angle_deg, uint16_t *pulse_us)
{
	uint8_t start_angle, end_angle;
	uint16_t start_us, end_us;
	int32_t pulse;

	if (calibration == NULL || pulse_us == NULL ||
	    calibration->right_angle_deg >= calibration->ahead_angle_deg ||
	    calibration->ahead_angle_deg >= calibration->left_angle_deg ||
	    calibration->left_angle_deg > 180U ||
	    calibration->right_us == 0U || calibration->ahead_us == 0U ||
	    calibration->left_us == 0U ||
	    calibration->right_us >= 20000U ||
	    calibration->ahead_us >= 20000U ||
	    calibration->left_us >= 20000U ||
	    !((calibration->right_us < calibration->ahead_us &&
	       calibration->ahead_us < calibration->left_us) ||
	      (calibration->right_us > calibration->ahead_us &&
	       calibration->ahead_us > calibration->left_us)) ||
	    angle_deg < calibration->right_angle_deg ||
	    angle_deg > calibration->left_angle_deg)
		return false;

	if (angle_deg <= calibration->ahead_angle_deg) {
		start_angle = calibration->right_angle_deg;
		end_angle = calibration->ahead_angle_deg;
		start_us = calibration->right_us;
		end_us = calibration->ahead_us;
	} else {
		start_angle = calibration->ahead_angle_deg;
		end_angle = calibration->left_angle_deg;
		start_us = calibration->ahead_us;
		end_us = calibration->left_us;
	}
	pulse = (int32_t)start_us +
		((int32_t)end_us - (int32_t)start_us) *
		(angle_deg - start_angle) / (end_angle - start_angle);
	*pulse_us = (uint16_t)pulse;
	return true;
}
