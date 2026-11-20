#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "vehicle_controller_task.h"

#define VEHICLE_CONTROLLER_PRIORITY 10

typedef struct {
	controller_state_t controller;
	uint32_t motion_sequence;
	uint32_t status_wait_ms;
	uint32_t status_sequence;
	uint32_t status_timestamp_ms;
	motion_status_message_t motion_status;
	bool arm_requested;
	bool emergency_sent;
	bool have_motion_status;
} vehicle_controller_context_t;

static vehicle_controller_task_config_t s_config;
static vehicle_controller_context_t s_context;
static bool s_started;

static bool publish_motion_command(vehicle_controller_context_t *context,
				   uint8_t mode, uint8_t stop_reason,
				   uint32_t now)
{
	motion_command_message_t message = {0};
	INT result;

	message.header.type = VEHICLE_MESSAGE_MOTION_COMMAND;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = context->motion_sequence + 1U;
	message.header.timestamp_ms = now;
	message.mode = mode;
	message.stop_reason = stop_reason;
	result = tk_snd_mbf(g_mbf_motion_commands, &message,
			    (INT)sizeof(message), TMO_POL);
	if (result != E_OK) {
		tm_printf((UB*)"vehicle controller command delivery failed: %d\n",
			  (int)result);
		return false;
	}
	context->motion_sequence = message.header.sequence;
	return true;
}

static bool receive_line(vehicle_controller_context_t *context, uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_LINE_CAPACITY; i++) {
		line_observation_message_t message;
		INT received = tk_rcv_mbf(g_mbf_line_observations, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"vehicle controller line receive failed: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		if (!car_safety_line(&context->controller.safety, &message, now))
			return false;
	}
	return true;
}

static bool receive_imu(vehicle_controller_context_t *context, uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_IMU_CAPACITY; i++) {
		imu_estimate_message_t message;
		INT received = tk_rcv_mbf(g_mbf_imu_estimates, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"vehicle controller IMU receive failed: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		if (!car_safety_imu(&context->controller.safety, &message, now))
			return false;
	}
	return true;
}

static bool receive_obstacle(vehicle_controller_context_t *context, uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_OBSTACLE_CAPACITY; i++) {
		obstacle_event_message_t message;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"vehicle controller obstacle receive failed: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		if (!car_safety_obstacle(&context->controller.safety, &message, now))
			return false;
	}
	return true;
}

static bool receive_faults(vehicle_controller_context_t *context)
{
	for (;;) {
		fault_message_t message;
		INT received = tk_rcv_mbf(g_mbf_faults, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message) ||
		    message.header.type != VEHICLE_MESSAGE_FAULT ||
		    message.header.version != VEHICLE_MESSAGE_VERSION ||
		    message.source < VEHICLE_FAULT_SENSOR ||
		    message.source > VEHICLE_FAULT_SYSTEM ||
		    message.recoverable > 1U || message.code == 0U) {
			tm_printf((UB*)"vehicle controller invalid fault message: %d\n",
				  (int)received);
		} else {
			tm_printf((UB*)"vehicle controller subsystem fault: source=%u code=%u\n",
				  (unsigned)message.source, (unsigned)message.code);
		}
		car_safety_latch_fault(&context->controller.safety);
		return false;
	}
}

static bool receive_motion_status(vehicle_controller_context_t *context,
				  uint32_t now)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_STATUS_CAPACITY; i++) {
		motion_status_message_t message;
		uint32_t age;
		uint32_t future;
		INT received = tk_rcv_mbf(g_mbf_motion_status, &message, TMO_POL);

		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message) ||
		    message.header.type != VEHICLE_MESSAGE_MOTION_STATUS ||
		    message.header.version != VEHICLE_MESSAGE_VERSION ||
		    message.active > 1U || message.faulted > 1U ||
		    (message.faulted != 0U && message.fault_code == 0U) ||
		    (message.faulted == 0U && message.fault_code != 0U)) {
			tm_printf((UB*)"vehicle controller invalid motion status: %d\n",
				  (int)received);
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		age = now - message.header.timestamp_ms;
		future = message.header.timestamp_ms - now;
		if (age > s_config.motion_status_max_age_ms ||
		    (future != 0U && future <= INT32_MAX) ||
		    (context->have_motion_status &&
		     (message.header.sequence - context->status_sequence != 1U ||
		      message.header.timestamp_ms - context->status_timestamp_ms >
			      INT32_MAX))) {
			tm_printf((UB*)"vehicle controller stale or out-of-order motion status\n");
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
		context->motion_status = message;
		context->status_sequence = message.header.sequence;
		context->status_timestamp_ms = message.header.timestamp_ms;
		context->have_motion_status = true;
		if (message.faulted != 0U || message.active != 0U ||
		    message.motor_1_pwm != 0 || message.motor_2_pwm != 0) {
			tm_printf((UB*)"vehicle controller motion status is not safely idle\n");
			car_safety_latch_fault(&context->controller.safety);
			return false;
		}
	}
	return true;
}

static void latch_controller_fault(vehicle_controller_context_t *context,
				   uint32_t now)
{
	car_safety_latch_fault(&context->controller.safety);
	(void)controller_state_step(&context->controller, now, false, false);
}

static bool controller_cycle(vehicle_controller_context_t *context,
			     uint32_t now,
			     const vehicle_controller_inputs_t *inputs)
{
	controller_state_value_t state;
	bool messages_valid = true;

	if (!receive_faults(context)) messages_valid = false;
	if (!receive_motion_status(context, now)) messages_valid = false;
	if (!receive_line(context, now)) messages_valid = false;
	if (!receive_imu(context, now)) messages_valid = false;
	if (!receive_obstacle(context, now)) messages_valid = false;
	if ((!context->have_motion_status &&
	     now - context->status_wait_ms > s_config.motion_status_max_age_ms) ||
	    (context->have_motion_status &&
	     now - context->status_timestamp_ms > s_config.motion_status_max_age_ms)) {
		tm_printf((UB*)"vehicle controller motion status deadline expired\n");
		messages_valid = false;
	}
	if (!messages_valid) car_safety_latch_fault(&context->controller.safety);

	state = controller_state_step(&context->controller, now,
				      inputs->go, inputs->emergency_stopped);
	if (state == CONTROLLER_FAULTED) {
		if (!context->emergency_sent &&
		    publish_motion_command(context, VEHICLE_MOTION_EMERGENCY_STOP,
					   (uint8_t)context->controller.safety.faults,
					   now))
			context->emergency_sent = true;
		return false;
	}

	if (state == CONTROLLER_FOLLOWING) {
		if (!context->have_motion_status) return true;
		if (!context->arm_requested) {
			if (publish_motion_command(context, VEHICLE_MOTION_ARM, 0U, now)) {
				context->arm_requested = true;
			} else {
				latch_controller_fault(context, now);
				if (publish_motion_command(context,
							   VEHICLE_MOTION_EMERGENCY_STOP,
							   (uint8_t)context->controller.safety.faults,
							   now))
					context->emergency_sent = true;
				return false;
			}
		}
		return true;
	}

	if (context->arm_requested) {
		if (publish_motion_command(context, VEHICLE_MOTION_STOP, 0U, now)) {
			context->arm_requested = false;
		} else {
			latch_controller_fault(context, now);
			if (publish_motion_command(context, VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)context->controller.safety.faults,
						   now))
				context->emergency_sent = true;
			return false;
		}
	}
	return true;
}

static void vehicle_controller_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	for (;;) {
		vehicle_controller_inputs_t inputs = {0};
		SYSTIM now;
		ER er = tk_get_otm(&now);

		if (er != E_OK) {
			tm_printf((UB*)"vehicle controller clock failed: %d\n", (int)er);
			latch_controller_fault(&s_context,
					       s_context.controller.safety.last_clock_ms);
			if (!s_context.emergency_sent &&
			    publish_motion_command(&s_context,
						   VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)s_context.controller.safety.faults,
						   s_context.controller.safety.last_clock_ms))
				s_context.emergency_sent = true;
		} else {
			bool inputs_valid = s_config.read_inputs(s_config.input_context, &inputs);
			if (!inputs_valid) {
				tm_printf((UB*)"vehicle controller local input read failed\n");
				latch_controller_fault(&s_context, now.lo);
			}
			(void)controller_cycle(&s_context, now.lo, &inputs);
		}

		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) {
			tm_printf((UB*)"vehicle controller schedule failed: %d\n", (int)er);
			latch_controller_fault(&s_context,
					       s_context.controller.safety.last_clock_ms);
			if (!s_context.emergency_sent &&
			    publish_motion_command(&s_context,
						   VEHICLE_MOTION_EMERGENCY_STOP,
						   (uint8_t)s_context.controller.safety.faults,
						   s_context.controller.safety.last_clock_ms))
				s_context.emergency_sent = true;
			for (;;) {
				er = tk_slp_tsk(TMO_FEVR);
				if (er != E_OK)
					tm_printf((UB*)"vehicle controller sleep failed: %d\n",
						  (int)er);
			}
		}
	}
}

ER vehicle_controller_task_start(const vehicle_controller_task_config_t *config)
{
	controller_state_t validation = {0};
	controller_state_t initial = {0};
	T_CTSK task = {0};
	SYSTIM now;
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->read_inputs == NULL ||
	    config->period_ms == 0U ||
	    config->period_ms > VEHICLE_MOTION_STATUS_PERIOD_MS / 2U ||
	    config->motion_status_max_age_ms < 100U ||
	    config->motion_status_max_age_ms > INT32_MAX ||
	    config->period_ms > config->motion_status_max_age_ms ||
	    config->period_ms > config->safety.line_max_age_ms ||
	    config->period_ms > config->safety.imu_max_age_ms ||
	    config->period_ms > config->safety.ahead_max_age_ms ||
	    !controller_state_init(&validation, &config->safety, 0U))
		return E_PAR;
	if (g_mbf_line_observations <= 0 || g_mbf_imu_estimates <= 0 ||
	    g_mbf_obstacle_events <= 0 || g_mbf_faults <= 0 ||
	    g_mbf_motion_commands <= 0 || g_mbf_motion_status <= 0)
		return E_OBJ;
	er = tk_get_otm(&now);
	if (er != E_OK) return er < E_OK ? er : E_SYS;
	if (!controller_state_init(&initial, &config->safety, now.lo))
		return E_PAR;

	s_config = *config;
	s_context = (vehicle_controller_context_t){0};
	s_context.controller = initial;
	s_context.status_wait_ms = now.lo;
	task.task = (FP)vehicle_controller_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = VEHICLE_CONTROLLER_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK)
			tm_printf((UB*)"vehicle controller cleanup failed: %d\n", (int)cleanup);
		else
			s_started = false;
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

#if APP_TEST
#include "../../test_report.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"

static BOOL drain_controller_test_buffer(ID id)
{
	union {
		line_observation_message_t line;
		imu_estimate_message_t imu;
		obstacle_event_message_t obstacle;
		fault_message_t fault;
		motion_status_message_t status;
		motion_command_message_t command;
	} message;
	INT received;

	do {
		received = tk_rcv_mbf(id, &message, TMO_POL);
	} while (received > 0);
	return received == E_TMOUT;
}

static BOOL drain_controller_test_buffers(void)
{
	BOOL ok = drain_controller_test_buffer(g_mbf_line_observations);
	ok = drain_controller_test_buffer(g_mbf_imu_estimates) && ok;
	ok = drain_controller_test_buffer(g_mbf_obstacle_events) && ok;
	ok = drain_controller_test_buffer(g_mbf_faults) && ok;
	ok = drain_controller_test_buffer(g_mbf_motion_status) && ok;
	ok = drain_controller_test_buffer(g_mbf_motion_commands) && ok;
	return ok;
}

static BOOL queue_motion_status(uint32_t now, uint32_t sequence,
				uint8_t active, int16_t motor_1, int16_t motor_2)
{
	motion_status_message_t status = {0};

	status.header.type = VEHICLE_MESSAGE_MOTION_STATUS;
	status.header.version = VEHICLE_MESSAGE_VERSION;
	status.header.sequence = sequence;
	status.header.timestamp_ms = now;
	status.active = active;
	status.motor_1_pwm = motor_1;
	status.motor_2_pwm = motor_2;
	return tk_snd_mbf(g_mbf_motion_status, &status, (INT)sizeof(status),
			  TMO_POL) == E_OK;
}

static BOOL queue_safe_sensor_messages(uint32_t now, uint32_t sequence)
{
	line_observation_message_t line = {0};
	imu_estimate_message_t imu = {0};
	obstacle_event_message_t obstacle = {0};
	BOOL ok;

	line.header = (vehicle_message_header_t){VEHICLE_MESSAGE_LINE_OBSERVATION,
						  VEHICLE_MESSAGE_VERSION, sequence, now};
	line.line_detected = 1U;
	line.confidence = 1000U;
	imu.header = (vehicle_message_header_t){VEHICLE_MESSAGE_IMU_ESTIMATE,
						 VEHICLE_MESSAGE_VERSION, sequence, now};
	imu.validity = VEHICLE_IMU_VALID_ROLL_PITCH;
	obstacle.header = (vehicle_message_header_t){VEHICLE_MESSAGE_OBSTACLE_EVENT,
						      VEHICLE_MESSAGE_VERSION, sequence, now};
	obstacle.obstacle_class = VEHICLE_OBSTACLE_AHEAD_READING;
	obstacle.distance_mm = OBS_DIST_NONE;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) == E_OK;
	ok = tk_snd_mbf(g_mbf_imu_estimates, &imu, (INT)sizeof(imu), TMO_POL) == E_OK && ok;
	ok = tk_snd_mbf(g_mbf_obstacle_events, &obstacle, (INT)sizeof(obstacle),
			TMO_POL) == E_OK && ok;
	return ok;
}

static BOOL receive_test_command(uint8_t expected_mode, uint32_t expected_sequence)
{
	motion_command_message_t command;
	INT received = tk_rcv_mbf(g_mbf_motion_commands, &command, TMO_POL);

	return received == (INT)sizeof(command) &&
	       command.header.type == VEHICLE_MESSAGE_MOTION_COMMAND &&
	       command.header.version == VEHICLE_MESSAGE_VERSION &&
	       command.header.sequence == expected_sequence &&
	       command.mode == expected_mode &&
	       command.linear_velocity == 0 && command.angular_velocity == 0 &&
	       command.duration_ms == 0U;
}

static const vehicle_controller_task_config_t test_config = {
	{200U, 200U, 200U, 200U, 500U, 300U},
	10U, 100U, NULL, NULL
};

static BOOL controller_arm_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	motion_command_message_t unexpected;
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = controller_cycle(&context, 100U, &inputs) &&
	     context.controller.state == CONTROLLER_WAITING &&
	     tk_rcv_mbf(g_mbf_motion_commands, &unexpected, TMO_POL) == E_TMOUT && ok;
	inputs.go = true;
	ok = controller_cycle(&context, 101U, &inputs) &&
	     context.controller.state == CONTROLLER_FOLLOWING &&
	     !context.arm_requested &&
	     tk_rcv_mbf(g_mbf_motion_commands, &unexpected, TMO_POL) == E_TMOUT && ok;
	ok = queue_motion_status(101U, 1U, 0U, 0, 0) && ok;
	ok = controller_cycle(&context, 102U, &inputs) &&
	     context.controller.state == CONTROLLER_FOLLOWING &&
	     context.arm_requested &&
	     receive_test_command(VEHICLE_MOTION_ARM, 1U) && ok;
	inputs.go = false;
	ok = controller_cycle(&context, 103U, &inputs) &&
	     context.controller.state == CONTROLLER_WAITING &&
	     !context.arm_requested &&
	     receive_test_command(VEHICLE_MOTION_STOP, 2U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_fault_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_motion_status(301U, 1U, 0U, 0, 0) && ok;
	ok = !controller_cycle(&context, 301U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_local_emergency_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {true, true};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = queue_motion_status(100U, 1U, 0U, 0, 0) && ok;
	ok = !controller_cycle(&context, 100U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_nonidle_status_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = queue_motion_status(100U, 1U, 0U, 20, -20) && ok;
	ok = !controller_cycle(&context, 100U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

static BOOL controller_stale_status_test(void)
{
	vehicle_controller_context_t context = {0};
	vehicle_controller_inputs_t inputs = {false, false};
	BOOL ok = controller_state_init(&context.controller, &test_config.safety, 100U);

	context.status_wait_ms = 100U;
	ok = queue_safe_sensor_messages(100U, 1U) && ok;
	ok = queue_motion_status(100U, 1U, 0U, 0, 0) && ok;
	ok = controller_cycle(&context, 100U, &inputs) && ok;
	ok = queue_safe_sensor_messages(201U, 2U) && ok;
	ok = !controller_cycle(&context, 201U, &inputs) &&
	     context.controller.state == CONTROLLER_FAULTED &&
	     context.emergency_sent &&
	     receive_test_command(VEHICLE_MOTION_EMERGENCY_STOP, 1U) && ok;
	return drain_controller_test_buffers() && ok;
}

BOOL vehicle_controller_task_test(void)
{
	BOOL arm_ok;
	BOOL fault_ok;
	BOOL emergency_ok;
	BOOL status_ok;
	BOOL stale_status_ok;
	TEST_INIT();

	/* controller_cycle() reads the module config (s_config) for the motion
	   status deadline.  The task is not started during the boot-time test
	   run, so s_config would otherwise still be all zeros (max age 0 ms)
	   and every advancing-clock cycle would fault.  Install the test
	   config; vehicle_controller_task_start() overwrites it later. */
	s_config = test_config;

	arm_ok = controller_arm_test();
	fault_ok = controller_fault_test();
	emergency_ok = controller_local_emergency_test();
	status_ok = controller_nonidle_status_test();
	stale_status_ok = controller_stale_status_test();
	TEST_CHECK("VC-U13", arm_ok, "fresh safety inputs and local GO gate ARM/STOP commands");
	TEST_CHECK("VC-U14", emergency_ok, "local emergency input sends a latched emergency stop");
	TEST_CHECK("VC-U15", fault_ok, "missing sensor deadlines latch fault and request emergency stop");
	TEST_CHECK("VC-U16", status_ok, "nonzero commanded output status blocks arming and requests emergency stop");
	TEST_CHECK("VC-U17", stale_status_ok, "stale motion status faults despite fresh sensor observations");
	TEST_SUMMARY("vehicle controller task");
	return arm_ok && fault_ok && emergency_ok && status_ok && stale_status_ok;
}
#endif
