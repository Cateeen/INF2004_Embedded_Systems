#ifndef OBS_SERVO_CALIBRATION_H
#define OBS_SERVO_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	/* Profile coordinates increase from right to left; ahead is not 90 by default. */
	uint8_t right_angle_deg;
	uint8_t ahead_angle_deg;
	uint8_t left_angle_deg;
	uint16_t right_us;
	uint16_t ahead_us;
	uint16_t left_us;
} obs_servo_calibration_t;

/* Interpolate independently on each side; reject angles outside the range. */
bool obs_servo_pulse(const obs_servo_calibration_t *calibration,
		     uint8_t angle_deg, uint16_t *pulse_us);

#endif
