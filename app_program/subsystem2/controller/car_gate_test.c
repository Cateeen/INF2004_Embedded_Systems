#include "../../app.h"
#include "car_gate_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "car_gate.h"
#include "car_safety.h"
#include "controller_state.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"

static const car_safety_config_t safety_config = {100U, 100U, 100U, 200U, 500U, 300U};

static line_observation_message_t safety_line(uint32_t sequence, uint32_t timestamp,
					      uint8_t detected)
{
	line_observation_message_t message = {0};
	message.header.type = VEHICLE_MESSAGE_LINE_OBSERVATION;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = sequence;
	message.header.timestamp_ms = timestamp;
	message.line_detected = detected;
	message.confidence = detected ? 1000U : 0U;
	return message;
}

static imu_estimate_message_t safety_imu(uint32_t sequence, uint32_t timestamp)
{
	imu_estimate_message_t message = {0};
	message.header.type = VEHICLE_MESSAGE_IMU_ESTIMATE;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = sequence;
	message.header.timestamp_ms = timestamp;
	message.validity = VEHICLE_IMU_VALID_ROLL_PITCH;
	return message;
}

static obstacle_event_message_t safety_obstacle(uint32_t sequence, uint32_t timestamp,
						 uint8_t type, uint16_t distance)
{
	obstacle_event_message_t message = {0};
	message.header.type = VEHICLE_MESSAGE_OBSTACLE_EVENT;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = sequence;
	message.header.timestamp_ms = timestamp;
	message.obstacle_class = type;
	message.distance_mm = distance;
	return message;
}

static BOOL safety_inputs(car_safety_t *safety, uint32_t sequence, uint32_t now)
{
	line_observation_message_t line = safety_line(sequence, now, 1U);
	imu_estimate_message_t imu = safety_imu(sequence, now);
	obstacle_event_message_t obstacle =
		safety_obstacle(sequence, now, VEHICLE_OBSTACLE_AHEAD_READING, OBS_DIST_NONE);
	return car_safety_line(safety, &line, now) && car_safety_imu(safety, &imu, now) &&
	       car_safety_obstacle(safety, &obstacle, now);
}

static BOOL safety_rearm_test(void)
{
	car_safety_t safety = {0};
	line_observation_message_t line;
	imu_estimate_message_t imu;
	obstacle_event_message_t obstacle;
	BOOL ok;

	ok = car_safety_init(&safety, &safety_config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) &&
	     !car_safety_follow(&safety, 100U, true, false, false) &&
	     car_safety_follow(&safety, 100U, true, true, false);
	/* A lost line does not disarm a following robot: the steering holds
	 * 300 ms then latches a stop, and a junction turn leaves the line. */
	line = safety_line(2U, 101U, 0U);
	ok = car_safety_line(&safety, &line, 101U) &&
	     car_safety_follow(&safety, 101U, true, false, false) && ok;
	/* Arming still needs the line: GO off, then a GO edge with no line. */
	ok = !car_safety_follow(&safety, 101U, false, false, false) &&
	     !car_safety_follow(&safety, 101U, true, true, false) && ok;
	line = safety_line(3U, 102U, 1U);
	ok = car_safety_line(&safety, &line, 102U) &&
	     !car_safety_follow(&safety, 102U, true, false, false) &&
	     car_safety_follow(&safety, 102U, true, true, false) && ok;
	obstacle = safety_obstacle(2U, 103U, VEHICLE_OBSTACLE_AHEAD_READING, 300U);
	ok = car_safety_obstacle(&safety, &obstacle, 103U) &&
	     !car_safety_follow(&safety, 103U, true, false, false) && ok;
	obstacle = safety_obstacle(3U, 104U, VEHICLE_OBSTACLE_AHEAD_READING, OBS_DIST_NONE);
	ok = car_safety_obstacle(&safety, &obstacle, 104U) &&
	     !car_safety_follow(&safety, 104U, true, false, false) &&
	     car_safety_follow(&safety, 104U, true, true, false) && ok;
	imu = safety_imu(2U, 105U);
	imu.tilt_detected = 1U;
	ok = car_safety_imu(&safety, &imu, 105U) &&
	     !car_safety_follow(&safety, 105U, true, true, false) && ok;
	imu = safety_imu(3U, 106U);
	ok = car_safety_imu(&safety, &imu, 106U) &&
	     !car_safety_follow(&safety, 106U, true, false, false) &&
	     car_safety_follow(&safety, 106U, true, true, false) && ok;
	return ok;
}

static BOOL safety_scan_test(void)
{
	car_safety_t safety = {0};
	line_observation_message_t line;
	imu_estimate_message_t imu;
	obstacle_event_message_t obstacle;
	BOOL ok;

	ok = car_safety_init(&safety, &safety_config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) &&
	     car_safety_follow(&safety, 100U, true, true, false) &&
	     car_safety_begin_scan(&safety, 110U) &&
	     !car_safety_follow(&safety, 110U, true, true, false);
	line = safety_line(2U, 180U, 1U);
	imu = safety_imu(2U, 180U);
	ok = car_safety_line(&safety, &line, 180U) && car_safety_imu(&safety, &imu, 180U) && ok;
	line = safety_line(3U, 230U, 1U);
	imu = safety_imu(3U, 230U);
	ok = car_safety_line(&safety, &line, 230U) && car_safety_imu(&safety, &imu, 230U) &&
	     !car_safety_follow(&safety, 230U, true, true, false) && safety.faults == 0U && ok;
	obstacle = safety_obstacle(2U, 240U, VEHICLE_OBSTACLE_SCAN_CLEAR, OBS_DIST_NONE);
	ok = car_safety_obstacle(&safety, &obstacle, 240U) &&
	     !car_safety_follow(&safety, 240U, true, true, false) && ok;
	obstacle = safety_obstacle(3U, 245U, VEHICLE_OBSTACLE_AHEAD_READING, OBS_DIST_NONE);
	ok = car_safety_obstacle(&safety, &obstacle, 245U) &&
	     !car_safety_follow(&safety, 245U, true, false, false) &&
	     car_safety_follow(&safety, 245U, true, true, false) && ok;
	return ok;
}

static BOOL safety_fault_test(void)
{
	car_safety_t safety = {0};
	car_safety_config_t config = safety_config;
	line_observation_message_t line;
	obstacle_event_message_t obstacle;
	BOOL ok;

	ok = car_safety_init(&safety, &config, 100U) &&
	     safety_inputs(&safety, 1U, 100U);
	line = safety_line(3U, 101U, 1U);
	ok = !car_safety_line(&safety, &line, 101U) &&
	     (safety.faults & CAR_SAFETY_FAULT_LINE) != 0U &&
	     !car_safety_follow(&safety, 101U, true, true, false) && ok;
	ok = car_safety_init(&safety, &config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) && ok;
	line = safety_line(2U, 201U, 1U);
	ok = !car_safety_line(&safety, &line, 201U) &&
	     (safety.faults & CAR_SAFETY_FAULT_LINE) != 0U && ok;
	ok = car_safety_init(&safety, &config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) && ok;
	obstacle = safety_obstacle(2U, 101U, VEHICLE_OBSTACLE_AHEAD_READING, 100U);
	obstacle.emergency = 1U;
	ok = !car_safety_obstacle(&safety, &obstacle, 101U) &&
	     (safety.faults & CAR_SAFETY_FAULT_EMERGENCY) != 0U &&
	     !car_safety_follow(&safety, 101U, true, true, false) && ok;
	config.line_max_age_ms = 1000U;
	config.imu_max_age_ms = 1000U;
	ok = car_safety_init(&safety, &config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) &&
	     car_safety_begin_scan(&safety, 100U) && ok;
	obstacle = safety_obstacle(2U, 301U, VEHICLE_OBSTACLE_SCAN_CLEAR, OBS_DIST_NONE);
	ok = !car_safety_obstacle(&safety, &obstacle, 301U) &&
	     (safety.faults & CAR_SAFETY_FAULT_OBSTACLE) != 0U && ok;
	ok = car_safety_init(&safety, &config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) && car_safety_begin_scan(&safety, 110U) && ok;
	obstacle = safety_obstacle(2U, 109U, VEHICLE_OBSTACLE_SCAN_CLEAR, OBS_DIST_NONE);
	ok = !car_safety_obstacle(&safety, &obstacle, 110U) &&
	     (safety.faults & CAR_SAFETY_FAULT_OBSTACLE) != 0U && ok;
	ok = car_safety_init(&safety, &config, 100U) &&
	     safety_inputs(&safety, 1U, 100U) && car_safety_begin_scan(&safety, 110U) && ok;
	obstacle = safety_obstacle(2U, 111U, VEHICLE_OBSTACLE_SCAN_CLEAR, OBS_DIST_NONE);
	ok = car_safety_obstacle(&safety, &obstacle, 120U) && ok;
	obstacle = safety_obstacle(3U, 119U, VEHICLE_OBSTACLE_AHEAD_READING, OBS_DIST_NONE);
	ok = !car_safety_obstacle(&safety, &obstacle, 120U) &&
	     (safety.faults & CAR_SAFETY_FAULT_OBSTACLE) != 0U && ok;
	config.ahead_max_age_ms = 0U;
	ok = !car_safety_init(&safety, &config, 400U) &&
	     !car_safety_follow(&safety, 400U, true, true, false) && ok;
	return ok;
}

static BOOL safety_wrap_test(void)
{
	car_safety_t safety = {0};
	line_observation_message_t line;
	BOOL ok;

	ok = car_safety_init(&safety, &safety_config, UINT32_MAX - 10U) &&
	     safety_inputs(&safety, UINT32_MAX, UINT32_MAX - 10U) &&
	     car_safety_follow(&safety, UINT32_MAX - 10U, true, true, false);
	line = safety_line(0U, 5U, 1U);
	ok = car_safety_line(&safety, &line, 5U) &&
	     car_safety_follow(&safety, 5U, true, false, false) && ok;
	ok = !car_safety_follow(&safety, 4U, true, false, false) &&
	     (safety.faults & CAR_SAFETY_FAULT_CLOCK) != 0U && ok;
	return ok;
}

static BOOL controller_state_flow_test(void)
{
	controller_state_t controller = {0};
	controller_state_t startup = {0};
	line_observation_message_t line;
	obstacle_event_message_t obstacle;
	BOOL ok;

	ok = controller_state_init(&controller, &safety_config, 100U) &&
	     !controller_state_init(&controller, &safety_config, 100U) &&
	     safety_inputs(&controller.safety, 1U, 100U) &&
	     controller_state_step(&controller, 100U, true, false) == CONTROLLER_BLOCKED &&
	     controller_state_step(&controller, 100U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&controller, 100U, true, false) == CONTROLLER_FOLLOWING;
	/* Losing the line keeps FOLLOWING; starting again needs it back. */
	line = safety_line(2U, 101U, 0U);
	ok = car_safety_line(&controller.safety, &line, 101U) &&
	     controller_state_step(&controller, 101U, true, false) == CONTROLLER_FOLLOWING &&
	     controller_state_step(&controller, 101U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&controller, 101U, true, false) == CONTROLLER_BLOCKED && ok;
	line = safety_line(3U, 102U, 1U);
	ok = car_safety_line(&controller.safety, &line, 102U) &&
	     controller_state_step(&controller, 102U, true, false) == CONTROLLER_BLOCKED &&
	     controller_state_step(&controller, 102U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&controller, 102U, true, false) == CONTROLLER_FOLLOWING && ok;
	obstacle = safety_obstacle(2U, 103U, VEHICLE_OBSTACLE_AHEAD_READING, 300U);
	ok = car_safety_obstacle(&controller.safety, &obstacle, 103U) &&
	     controller_state_step(&controller, 103U, true, false) == CONTROLLER_BLOCKED && ok;
	obstacle = safety_obstacle(3U, 104U, VEHICLE_OBSTACLE_AHEAD_READING, OBS_DIST_NONE);
	ok = car_safety_obstacle(&controller.safety, &obstacle, 104U) &&
	     controller_state_step(&controller, 104U, true, false) == CONTROLLER_BLOCKED &&
	     controller_state_step(&controller, 104U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&controller, 104U, true, false) == CONTROLLER_FOLLOWING && ok;
	ok = controller_state_init(&startup, &safety_config, 200U) &&
	     controller_state_step(&startup, 200U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&startup, 200U, true, false) == CONTROLLER_BLOCKED &&
	     safety_inputs(&startup.safety, 1U, 201U) &&
	     controller_state_step(&startup, 201U, true, false) == CONTROLLER_BLOCKED &&
	     controller_state_step(&startup, 201U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&startup, 201U, true, false) == CONTROLLER_FOLLOWING && ok;
	return ok;
}

static BOOL controller_state_fault_test(void)
{
	controller_state_t controller = {0};
	controller_state_t stale = {0};
	controller_state_t wrap = {0};
	controller_state_t bad = {0};
	car_safety_config_t invalid = safety_config;
	line_observation_message_t line;
	BOOL ok;

	invalid.line_max_age_ms = 0U;
	ok = controller_state_step(NULL, 0U, false, false) == CONTROLLER_FAULTED &&
	     controller_state_step(&controller, 0U, false, false) == CONTROLLER_FAULTED &&
	     !controller_state_init(&bad, &invalid, 100U) &&
	     controller_state_step(&bad, 100U, false, false) == CONTROLLER_FAULTED &&
	     controller_state_init(&controller, &safety_config, 100U) &&
	     safety_inputs(&controller.safety, 1U, 100U) &&
	     controller_state_step(&controller, 100U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&controller, 100U, true, false) == CONTROLLER_FOLLOWING &&
	     controller_state_step(&controller, 101U, true, true) == CONTROLLER_FAULTED &&
	     controller_state_step(&controller, 102U, false, false) == CONTROLLER_FAULTED &&
	     !controller_state_init(&controller, &safety_config, 102U);
	ok = controller_state_init(&stale, &safety_config, 100U) &&
	     safety_inputs(&stale.safety, 1U, 100U) &&
	     controller_state_step(&stale, 201U, false, false) == CONTROLLER_FAULTED && ok;
	line = safety_line(3U, 101U, 1U);
	ok = controller_state_init(&bad, &safety_config, 100U) &&
	     safety_inputs(&bad.safety, 1U, 100U) &&
	     !car_safety_line(&bad.safety, &line, 101U) &&
	     controller_state_step(&bad, 101U, true, false) == CONTROLLER_FAULTED && ok;
	ok = controller_state_init(&wrap, &safety_config, UINT32_MAX - 5U) &&
	     safety_inputs(&wrap.safety, UINT32_MAX, UINT32_MAX - 5U) &&
	     controller_state_step(&wrap, UINT32_MAX - 5U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&wrap, 3U, true, false) == CONTROLLER_FOLLOWING &&
	     controller_state_step(&wrap, 2U, true, false) == CONTROLLER_FAULTED && ok;
	return ok;
}

static BOOL controller_state_scan_test(void)
{
	controller_state_t controller = {0};
	obstacle_event_message_t obstacle;
	BOOL ok;

	ok = controller_state_init(&controller, &safety_config, 100U) &&
	     safety_inputs(&controller.safety, 1U, 100U) &&
	     controller_state_step(&controller, 100U, false, false) == CONTROLLER_WAITING &&
	     car_safety_begin_scan(&controller.safety, 101U) &&
	     controller_state_step(&controller, 101U, true, false) == CONTROLLER_SCANNING;
	obstacle = safety_obstacle(2U, 102U, VEHICLE_OBSTACLE_SCAN_CLEAR, OBS_DIST_NONE);
	ok = car_safety_obstacle(&controller.safety, &obstacle, 102U) &&
	     controller_state_step(&controller, 102U, true, false) == CONTROLLER_BLOCKED && ok;
	obstacle = safety_obstacle(3U, 103U, VEHICLE_OBSTACLE_AHEAD_READING, OBS_DIST_NONE);
	ok = car_safety_obstacle(&controller.safety, &obstacle, 103U) &&
	     controller_state_step(&controller, 103U, true, false) == CONTROLLER_BLOCKED &&
	     controller_state_step(&controller, 103U, false, false) == CONTROLLER_WAITING &&
	     controller_state_step(&controller, 103U, true, false) == CONTROLLER_FOLLOWING && ok;
	return ok;
}

BOOL car_gate_test(void)
{
	BOOL motion_ok;
	BOOL follow_ok;
	BOOL rearm_ok;
	BOOL safety_ok;
	BOOL scan_ok;
	BOOL fault_ok;
	BOOL wrap_ok;
	BOOL state_flow_ok;
	BOOL state_fault_ok;
	BOOL state_scan_ok;
	TEST_INIT();

	motion_ok = car_gate_motion(true, false, 0U) &&
		    !car_gate_motion(false, false, 0U) &&
		    !car_gate_motion(true, true, 0U) &&
		    !car_gate_motion(true, false, 1U) &&
		    !car_gate_motion(true, false, 1UL << 31);

	follow_ok = car_gate_follow(true, false, 0U, false) &&
		    !car_gate_follow(true, false, 0U, true) &&
		    !car_gate_follow(false, false, 0U, false) &&
		    !car_gate_follow(true, true, 0U, false) &&
		    !car_gate_follow(true, false, 1U, false) &&
		    car_gate_motion(true, false, 0U);

	rearm_ok = car_gate_rearm(true, false, 0U, false) &&
		   !car_gate_rearm(false, false, 0U, false) &&
		   !car_gate_rearm(true, true, 0U, false) &&
		   !car_gate_rearm(true, false, 1U, false) &&
		   !car_gate_rearm(true, false, 0U, true);
	safety_ok = safety_rearm_test();
	scan_ok = safety_scan_test();
	fault_ok = safety_fault_test();
	wrap_ok = safety_wrap_test();
	state_flow_ok = controller_state_flow_test();
	state_fault_ok = controller_state_fault_test();
	state_scan_ok = controller_state_scan_test();

	TEST_CHECK("VC-U01", motion_ok, "stop, emergency and fault veto motion");
	TEST_CHECK("VC-U02", follow_ok, "obstacle vetoes line following");
	TEST_CHECK("VC-U03", rearm_ok, "line following requires a fresh go edge");
	TEST_CHECK("VC-U04", safety_ok, "sensor vetoes require a fresh go edge");
	TEST_CHECK("VC-U05", scan_ok, "scan requires fresh ahead data before rearming");
	TEST_CHECK("VC-U06", fault_ok, "invalid, late and emergency inputs latch faults");
	TEST_CHECK("VC-U07", wrap_ok, "sensor and clock wrap with backward clock rejection");
	TEST_CHECK("VC-U10", state_flow_ok, "state transitions and fresh go edges");
	TEST_CHECK("VC-U11", state_fault_ok, "state faults, emergency latch and clock wrap");
	TEST_CHECK("VC-U12", state_scan_ok, "scan state and ahead reacquisition");
	TEST_SUMMARY("controller gate");
	return motion_ok && follow_ok && rearm_ok && safety_ok && scan_ok && fault_ok &&
	       wrap_ok && state_flow_ok && state_fault_ok && state_scan_ok;
}
#endif
