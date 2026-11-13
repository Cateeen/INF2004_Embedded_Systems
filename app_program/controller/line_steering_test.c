#include "../app.h"
#include "line_steering_test.h"

#if APP_TEST
#include "../test_report.h"
#include "../line_barcode/line_observation.h"
#include "line_steering.h"

BOOL line_steering_test(void)
{
	line_steering_t steering = {0};
	line_steering_command_t command;
	BOOL positions_ok;
	BOOL lost_line_ok;
	BOOL latch_ok;
	BOOL invalid_position_ok;
	TEST_INIT();

	line_steering_reset(&steering);
	command = line_steering_update(&steering, POS_JUNCTION, 1000U);
	positions_ok = command.motor_1_pwm == 25 &&
		       command.motor_2_pwm == -25 && !command.stop_latched;
	command = line_steering_update(&steering, POS_LEFT, 1010U);
	positions_ok = command.motor_1_pwm == 15 &&
		       command.motor_2_pwm == -25 && !command.stop_latched &&
		       positions_ok;
	command = line_steering_update(&steering, POS_RIGHT, 1020U);
	positions_ok = command.motor_1_pwm == 25 &&
		       command.motor_2_pwm == -15 && !command.stop_latched &&
		       positions_ok;
	TEST_CHECK("VC-U18", positions_ok,
		   "junction, left and right positions preserve source PWM decisions");

	command = line_steering_update(&steering, POS_LOST, 1030U);
	lost_line_ok = command.motor_1_pwm == 25 &&
		       command.motor_2_pwm == -15 && !command.stop_latched;
	command = line_steering_update(&steering, POS_LOST, 1329U);
	lost_line_ok = !command.stop_latched && lost_line_ok;
	command = line_steering_update(&steering, POS_LOST, 1330U);
	lost_line_ok = command.stop_latched && command.motor_1_pwm == 0 &&
		       command.motor_2_pwm == 0 && lost_line_ok;
	TEST_CHECK("VC-U19", lost_line_ok,
		   "lost-line hold expires at 300 ms with a zero-output latch");

	command = line_steering_update(&steering, POS_JUNCTION, 1400U);
	latch_ok = command.stop_latched && command.motor_1_pwm == 0 &&
		   command.motor_2_pwm == 0;
	line_steering_reset(&steering);
	command = line_steering_update(&steering, POS_JUNCTION, 1410U);
	latch_ok = !command.stop_latched && command.motor_1_pwm == 25 &&
		   command.motor_2_pwm == -25 && latch_ok;
	TEST_CHECK("VC-U20", latch_ok,
		   "stop latch persists until explicit reset");

	line_steering_reset(&steering);
	command = line_steering_update(&steering, UINT8_MAX, 1420U);
	invalid_position_ok = command.stop_latched &&
			      command.motor_1_pwm == 0 &&
			      command.motor_2_pwm == 0;
	command = line_steering_update(&steering, POS_JUNCTION, 1430U);
	invalid_position_ok = command.stop_latched &&
			      command.motor_1_pwm == 0 &&
			      command.motor_2_pwm == 0 && invalid_position_ok;
	TEST_CHECK("VC-U21", invalid_position_ok,
		   "invalid line positions fail closed until reset");

	TEST_SUMMARY("line steering");
	return positions_ok && lost_line_ok && latch_ok && invalid_position_ok;
}
#endif
