#include "../../app.h"
#include "line_steering_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "../../subsystem3/line_barcode/line_observation.h"
#include "line_steering.h"

#define SPEED LINE_STEERING_SPEED_CM_S
#define TURN  LINE_STEERING_TURN_CM_S
#define TMIN  LINE_STEERING_TURN_MIN_CM_S
#define RAMP  LINE_STEERING_RAMP_MS

static BOOL is_cmd(line_steering_command_t c, int16_t speed, int16_t turn)
{
	return c.linear_cm_s == speed && c.turn_cm_s == turn && !c.stop_latched;
}

static BOOL is_stop(line_steering_command_t c)
{
	return c.stop_latched && c.linear_cm_s == 0 && c.turn_cm_s == 0;
}

/* The turn builds from TMIN to TURN while one side is held, restarts on a
 * side change, bleeds off (not resets) through CENTRE, and never jumps on a
 * long gap between updates. */
static BOOL ramp_test(void)
{
	line_steering_t steering;
	line_steering_command_t command;
	int16_t previous = 0;
	uint32_t t = 2000U;
	BOOL ok;

	ok = line_steering_ramp_turn(&line_steering_normal, 0U) == TMIN &&
	     line_steering_ramp_turn(&line_steering_normal, (uint32_t)RAMP) == TURN &&
	     line_steering_ramp_turn(&line_steering_normal, (uint32_t)RAMP * 10U) == TURN;

	/* Hold LEFT at the controller's 50 ms: never shrinks, full at RAMP. */
	line_steering_reset(&steering);
	command = line_steering_update(&steering, POS_LEFT, t);
	ok = is_cmd(command, SPEED, TMIN) && ok;
	for (; t < 2000U + (uint32_t)RAMP; ) {
		t += 50U;
		command = line_steering_update(&steering, POS_LEFT, t);
		ok = !command.stop_latched && command.turn_cm_s >= previous &&
		     command.turn_cm_s >= TMIN && command.turn_cm_s <= TURN && ok;
		previous = command.turn_cm_s;
	}
	ok = is_cmd(command, SPEED, TURN) && ok;
	t += 50U;
	ok = is_cmd(line_steering_update(&steering, POS_LEFT, t), SPEED, TURN) && ok;

	/* Changing side restarts at the minimum. */
	t += 50U;
	ok = is_cmd(line_steering_update(&steering, POS_RIGHT, t), SPEED, -TMIN) && ok;

	/* Build RIGHT to full, CENTRE for half the ramp, then RIGHT again:
	 * resumes at the bled-off level, between the minimum and full. */
	for (unsigned i = 0U; i < (unsigned)RAMP / 50U + 1U; i++) {
		t += 50U;
		command = line_steering_update(&steering, POS_RIGHT, t);
	}
	ok = is_cmd(command, SPEED, -TURN) && ok;
	for (unsigned i = 0U; i < (unsigned)RAMP / 100U; i++) {
		t += 50U;
		ok = is_cmd(line_steering_update(&steering, POS_JUNCTION, t), SPEED, 0) && ok;
	}
	t += 50U;
	command = line_steering_update(&steering, POS_RIGHT, t);
	ok = is_cmd(command, SPEED,
		    (int16_t)-line_steering_ramp_turn(&line_steering_normal, steering.ramp_ms)) && ok;
	ok = steering.ramp_ms > 0U && steering.ramp_ms < (uint32_t)RAMP && ok;

	/* A 2 s gap adds at most 100 ms of build-up. */
	line_steering_reset(&steering);
	(void)line_steering_update(&steering, POS_LEFT, 5000U);
	(void)line_steering_update(&steering, POS_LEFT, 7000U);
	ok = steering.ramp_ms == 100U && ok;
	return ok;
}

/* SLOPE profile: its own speed and turn range, selected and cleared at run
 * time; a turn in progress carries its build-up across the switch. */
static BOOL slope_profile_test(void)
{
	const line_steering_profile_t *slope = &line_steering_slope;
	line_steering_t steering;
	uint32_t t = 3000U;
	BOOL ok;

	line_steering_reset(&steering);
	ok = line_steering_profile(&steering) == &line_steering_normal;
	line_steering_set_slope(&steering, true);
	ok = line_steering_profile(&steering) == slope && ok;
	ok = is_cmd(line_steering_update(&steering, POS_JUNCTION, t),
		    LINE_STEERING_SLOPE_SPEED_CM_S, 0) && ok;
	t += 50U;
	ok = is_cmd(line_steering_update(&steering, POS_RIGHT, t),
		    LINE_STEERING_SLOPE_SPEED_CM_S, -LINE_STEERING_SLOPE_TURN_MIN_CM_S) && ok;
	/* Held: builds to the SLOPE full turn, not the NORMAL one. */
	for (unsigned i = 0U; i < (unsigned)LINE_STEERING_SLOPE_RAMP_MS / 50U + 1U; i++) {
		t += 50U;
		(void)line_steering_update(&steering, POS_RIGHT, t);
	}
	ok = steering.last_turn_cm_s == -LINE_STEERING_SLOPE_TURN_CM_S &&
	     steering.last_linear_cm_s == LINE_STEERING_SLOPE_SPEED_CM_S && ok;
	/* Back to NORMAL mid-turn: same side keeps building, at NORMAL values. */
	line_steering_set_slope(&steering, false);
	t += 50U;
	ok = is_cmd(line_steering_update(&steering, POS_RIGHT, t), LINE_STEERING_SPEED_CM_S,
		    (int16_t)-line_steering_ramp_turn(&line_steering_normal,
						      steering.ramp_ms)) && ok;
	ok = steering.ramp_ms > 0U && ok;
	/* Reset returns to NORMAL. */
	line_steering_set_slope(&steering, true);
	line_steering_reset(&steering);
	ok = line_steering_profile(&steering) == &line_steering_normal && ok;
	ok = line_steering_ramp_turn(slope, 0U) == LINE_STEERING_SLOPE_TURN_MIN_CM_S &&
	     line_steering_ramp_turn(slope, 100000U) == LINE_STEERING_SLOPE_TURN_CM_S && ok;
	return ok;
}

BOOL line_steering_test(void)
{
	line_steering_t steering = {0};
	line_steering_command_t command;
	BOOL positions_ok;
	BOOL lost_line_ok;
	BOOL latch_ok;
	BOOL invalid_position_ok;
	BOOL ramp_ok;
	BOOL slope_ok;
	TEST_INIT();

	line_steering_reset(&steering);
	command = line_steering_update(&steering, POS_JUNCTION, 1000U);
	positions_ok = is_cmd(command, SPEED, 0);
	command = line_steering_update(&steering, POS_LEFT, 1010U);
	positions_ok = is_cmd(command, SPEED, TMIN) && positions_ok;
	command = line_steering_update(&steering, POS_RIGHT, 1020U);
	positions_ok = is_cmd(command, SPEED, -TMIN) && positions_ok;
	positions_ok = SPEED > TURN && TURN >= TMIN && TMIN > 0 && positions_ok;
	TEST_CHECK("VC-U18", positions_ok,
		   "junction, left and right positions map to straight, left and right speed commands");

	command = line_steering_update(&steering, POS_LOST, 1030U);
	lost_line_ok = is_cmd(command, SPEED, -TMIN);
	command = line_steering_update(&steering, POS_LOST, 1329U);
	lost_line_ok = !command.stop_latched && lost_line_ok;
	command = line_steering_update(&steering, POS_LOST, 1330U);
	lost_line_ok = is_stop(command) && lost_line_ok;
	TEST_CHECK("VC-U19", lost_line_ok,
		   "lost-line hold expires at 300 ms with a zero-output latch");

	command = line_steering_update(&steering, POS_JUNCTION, 1400U);
	latch_ok = is_stop(command);
	line_steering_reset(&steering);
	command = line_steering_update(&steering, POS_JUNCTION, 1410U);
	latch_ok = is_cmd(command, SPEED, 0) && latch_ok;
	TEST_CHECK("VC-U20", latch_ok,
		   "stop latch persists until explicit reset");

	line_steering_reset(&steering);
	command = line_steering_update(&steering, UINT8_MAX, 1420U);
	invalid_position_ok = is_stop(command);
	command = line_steering_update(&steering, POS_JUNCTION, 1430U);
	invalid_position_ok = is_stop(command) && invalid_position_ok;
	TEST_CHECK("VC-U21", invalid_position_ok,
		   "invalid line positions fail closed until reset");

	ramp_ok = ramp_test();
	TEST_CHECK("VC-U23", ramp_ok,
		   "turn builds from minimum to full on one side, restarts on a side change, bleeds off through centre");

	slope_ok = slope_profile_test();
	TEST_CHECK("VC-U25", slope_ok,
		   "SLOPE profile uses its own speed and turn range, switches at run time, reset returns to NORMAL");

	TEST_SUMMARY("line steering");
	return positions_ok && lost_line_ok && latch_ok && invalid_position_ok && ramp_ok &&
	       slope_ok;
}
#endif
