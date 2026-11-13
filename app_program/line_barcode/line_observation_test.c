#include "../app.h"
#include "line_observation_test.h"

#if APP_TEST
#include "../test_report.h"
#include "line_observation.h"

BOOL line_observation_test(void)
{
	ir_readings_t readings = {0};
	BOOL positions_ok;
	BOOL independence_ok;
	TEST_INIT();

	positions_ok = line_following_position(&readings) == POS_LOST;
	readings.center.black = true;
	positions_ok = positions_ok &&
		       line_following_position(&readings) == POS_LEFT;
	readings.center.black = false;
	readings.right.black = true;
	positions_ok = positions_ok &&
		       line_following_position(&readings) == POS_RIGHT;
	readings.center.black = true;
	positions_ok = positions_ok &&
		       line_following_position(&readings) == POS_JUNCTION;

	readings.barcode.black = true;
	independence_ok = line_following_position(&readings) == POS_JUNCTION;
	readings.center.black = false;
	independence_ok = independence_ok &&
			  line_following_position(&readings) == POS_RIGHT;
	readings.center.black = true;
	readings.right.black = false;
	independence_ok = independence_ok &&
			  line_following_position(&readings) == POS_LEFT;
	readings.center.black = false;
	independence_ok = independence_ok &&
			  line_following_position(&readings) == POS_LOST &&
			  line_following_position(NULL) == POS_LOST;

	TEST_CHECK("LINE-U01", positions_ok, "line sensor combinations");
	TEST_CHECK("LINE-U02", independence_ok, "barcode input and missing readings");
	TEST_SUMMARY("line observation");
	return positions_ok && independence_ok;
}
#endif
