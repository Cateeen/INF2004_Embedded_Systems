#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../../app_program/subsystem5/obstacle/obs_servo_calibration.h"

static int failures;

static void check(bool condition, const char *description)
{
	if (!condition) {
		printf("FAIL: %s\n", description);
		failures++;
	}
}

int main(void)
{
	obs_servo_calibration_t calibration = {80U, 100U, 120U, 1500U, 1700U, 1900U};
	uint16_t pulse = 0U;
	unsigned angle;

	for (angle = 80U; angle <= 120U; angle += 5U) {
		check(obs_servo_pulse(&calibration, (uint8_t)angle, &pulse) &&
		      pulse == 1500U + (angle - 80U) * 10U,
		      "scan position maps to the expected bounded pulse");
	}
	pulse = 123U;
	check(!obs_servo_pulse(&calibration, 79U, &pulse) && pulse == 123U,
	      "reject angle below calibration without changing output");
	check(!obs_servo_pulse(&calibration, 121U, &pulse) && pulse == 123U,
	      "reject angle above calibration without changing output");
	check(!obs_servo_pulse(NULL, 100U, &pulse) &&
	      !obs_servo_pulse(&calibration, 100U, NULL), "reject null arguments");

	calibration.ahead_us = 1650U;
	check(obs_servo_pulse(&calibration, 100U, &pulse) && pulse == 1650U,
	      "preserve an asymmetric center");
	check(obs_servo_pulse(&calibration, 110U, &pulse) && pulse == 1775U,
	      "interpolate from the center to the left endpoint");
	calibration.ahead_us = 1700U;
	calibration.right_us = 1900U;
	calibration.left_us = 1500U;
	check(obs_servo_pulse(&calibration, 85U, &pulse) && pulse == 1850U,
	      "support reversed pulse direction");
	check(obs_servo_pulse(&calibration, 115U, &pulse) && pulse == 1550U,
	      "support reversed pulse direction on the left");
	calibration.ahead_us = 1400U;
	check(!obs_servo_pulse(&calibration, 100U, &pulse),
	      "reject non-monotonic calibration");
	calibration.ahead_us = 1700U;
	calibration.left_angle_deg = 100U;
	check(!obs_servo_pulse(&calibration, 100U, &pulse),
	      "reject zero-width calibration segment");
	calibration.left_angle_deg = 120U;
	calibration.right_us = 20000U;
	check(!obs_servo_pulse(&calibration, 100U, &pulse),
	      "reject pulse outside PWM period");

	if (failures != 0) return 1;
	puts("PASS: bounded servo calibration");
	return 0;
}
