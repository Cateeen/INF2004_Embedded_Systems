#include "../app.h"
#include "../vehicle_buffers.h"
#include "controller_monitor.h"

#define CONTROLLER_MONITOR_PRIORITY 10
#define CONTROLLER_MONITOR_FAULT_DELAY_MS 1000U

static controller_monitor_config_t s_config;
static controller_state_t s_controller;
static bool s_started;

static bool read_clock(car_safety_t *safety, uint32_t *now)
{
	SYSTIM time;
	ER er = tk_get_otm(&time);
	if (er != E_OK) {
		tm_printf((UB*)"controller monitor clock failed: %d\n", (int)er);
		safety->faults |= CAR_SAFETY_FAULT_CLOCK;
		safety->armed = false;
		return false;
	}
	*now = time.lo;
	return true;
}

static bool receive_faults(car_safety_t *safety)
{
	fault_message_t fault;
	INT received = tk_rcv_mbf(g_mbf_faults, &fault, TMO_POL);
	if (received == E_TMOUT) return true;
	if (received != (INT)sizeof(fault) ||
	    fault.header.type != VEHICLE_MESSAGE_FAULT ||
	    fault.header.version != VEHICLE_MESSAGE_VERSION ||
	    fault.source < VEHICLE_FAULT_SENSOR || fault.source > VEHICLE_FAULT_SYSTEM ||
	    fault.recoverable > 1U || fault.code == 0U) {
		tm_printf((UB*)"controller monitor invalid fault message: %d\n", (int)received);
	} else {
		tm_printf((UB*)"controller monitor subsystem fault: source=%u code=%u\n",
			  (unsigned)fault.source, (unsigned)fault.code);
	}
	car_safety_latch_fault(safety);
	return false;
}

static bool receive_line(car_safety_t *safety)
{
	unsigned i;
	for (i = 0U; i < VEHICLE_MB_LINE_CAPACITY; i++) {
		line_observation_message_t message;
		uint32_t now;
		INT received = tk_rcv_mbf(g_mbf_line_observations, &message, TMO_POL);
		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"controller monitor line receive failed: %d\n", (int)received);
			car_safety_latch_fault(safety);
			return false;
		}
		if (!read_clock(safety, &now) || !car_safety_line(safety, &message, now)) return false;
	}
	return true;
}

static bool receive_imu(car_safety_t *safety)
{
	unsigned i;
	for (i = 0U; i < VEHICLE_MB_IMU_CAPACITY; i++) {
		imu_estimate_message_t message;
		uint32_t now;
		INT received = tk_rcv_mbf(g_mbf_imu_estimates, &message, TMO_POL);
		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"controller monitor IMU receive failed: %d\n", (int)received);
			car_safety_latch_fault(safety);
			return false;
		}
		if (!read_clock(safety, &now) || !car_safety_imu(safety, &message, now)) return false;
	}
	return true;
}

static bool receive_obstacle(car_safety_t *safety)
{
	unsigned i;
	for (i = 0U; i < VEHICLE_MB_OBSTACLE_CAPACITY; i++) {
		obstacle_event_message_t message;
		uint32_t now;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &message, TMO_POL);
		if (received == E_TMOUT) return true;
		if (received != (INT)sizeof(message)) {
			tm_printf((UB*)"controller monitor obstacle receive failed: %d\n", (int)received);
			car_safety_latch_fault(safety);
			return false;
		}
		if (!read_clock(safety, &now) || !car_safety_obstacle(safety, &message, now))
			return false;
	}
	return true;
}

static bool monitor_cycle(controller_state_t *controller)
{
	car_safety_t *safety = &controller->safety;
	uint32_t now = safety->last_clock_ms;

	if (!receive_faults(safety) || !receive_line(safety) || !receive_imu(safety) ||
	    !receive_obstacle(safety) || !read_clock(safety, &now)) {
		(void)controller_state_step(controller, safety->last_clock_ms, false, false);
		return false;
	}
	return controller_state_step(controller, now, false, false) != CONTROLLER_FAULTED;
}

static void monitor_stopped(void)
{
	tm_printf((UB*)"controller monitor stopped; safety faults=0x%x\n",
		  (unsigned)s_controller.safety.faults);
	for (;;) {
		ER er = tk_dly_tsk(CONTROLLER_MONITOR_FAULT_DELAY_MS);
		if (er != E_OK) {
			tm_printf((UB*)"controller monitor delay failed: %d\n", (int)er);
			er = tk_slp_tsk(TMO_FEVR);
			tm_printf((UB*)"controller monitor sleep returned: %d\n", (int)er);
		}
	}
}

static void controller_monitor_task(INT stacd, void *exinf)
{
	ER er;

	(void)stacd;
	(void)exinf;
	for (;;) {
		if (!monitor_cycle(&s_controller)) monitor_stopped();
		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) {
			tm_printf((UB*)"controller monitor schedule failed: %d\n", (int)er);
			car_safety_latch_fault(&s_controller.safety);
			(void)controller_state_step(&s_controller, s_controller.safety.last_clock_ms,
						    false, false);
			monitor_stopped();
		}
	}
}

ER controller_monitor_start(const controller_monitor_config_t *config)
{
	controller_state_t probe = {0};
	controller_state_t validation = {0};
	T_CTSK task = {0};
	SYSTIM now;
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->period_ms == 0U ||
	    config->period_ms > config->safety.line_max_age_ms ||
	    config->period_ms > config->safety.imu_max_age_ms ||
	    config->period_ms > config->safety.ahead_max_age_ms ||
	    !controller_state_init(&probe, &config->safety, 0U))
		return E_PAR;
	if (g_mbf_line_observations <= 0 || g_mbf_imu_estimates <= 0 ||
	    g_mbf_obstacle_events <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	er = tk_get_otm(&now);
	if (er != E_OK) return er < E_OK ? er : E_SYS;
	if (!controller_state_init(&validation, &config->safety, now.lo)) return E_PAR;
	s_config = *config;
	s_controller = validation;
	task.task = (FP)controller_monitor_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = CONTROLLER_MONITOR_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK)
			tm_printf((UB*)"controller monitor task cleanup failed: %d\n", (int)cleanup);
		else s_started = false;
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

#if APP_TEST
#include "../test_report.h"
#include "../obstacle/obs_profile_config.h"

static BOOL drain_test_buffer(ID id)
{
	union {
		line_observation_message_t line;
		imu_estimate_message_t imu;
		obstacle_event_message_t obstacle;
		fault_message_t fault;
	} message;
	INT received;
	do {
		received = tk_rcv_mbf(id, &message, TMO_POL);
	} while (received > 0);
	return received == E_TMOUT;
}

static BOOL drain_test_buffers(void)
{
	BOOL ok = drain_test_buffer(g_mbf_line_observations);
	ok = drain_test_buffer(g_mbf_imu_estimates) && ok;
	ok = drain_test_buffer(g_mbf_obstacle_events) && ok;
	ok = drain_test_buffer(g_mbf_faults) && ok;
	return ok;
}

static const car_safety_config_t test_config = {10000U, 10000U, 10000U, 10000U, 500U, 300U};

static BOOL monitor_flow_test(void)
{
	controller_state_t controller = {0};
	car_safety_t *safety = &controller.safety;
	line_observation_message_t line = {0};
	imu_estimate_message_t imu = {0};
	obstacle_event_message_t obstacle = {0};
	uint32_t now;
	BOOL ok;

	if (!read_clock(safety, &now) || !controller_state_init(&controller, &test_config, now))
		return FALSE;
	line.header = (vehicle_message_header_t){VEHICLE_MESSAGE_LINE_OBSERVATION,
						  VEHICLE_MESSAGE_VERSION, 1U, now};
	line.line_detected = 1U;
	line.confidence = 1000U;
	imu.header = (vehicle_message_header_t){VEHICLE_MESSAGE_IMU_ESTIMATE,
						 VEHICLE_MESSAGE_VERSION, 1U, now};
	imu.validity = VEHICLE_IMU_VALID_ROLL_PITCH;
	obstacle.header = (vehicle_message_header_t){VEHICLE_MESSAGE_OBSTACLE_EVENT,
						      VEHICLE_MESSAGE_VERSION, 1U, now};
	obstacle.obstacle_class = VEHICLE_OBSTACLE_AHEAD_READING;
	obstacle.distance_mm = OBS_DIST_NONE;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) == E_OK;
	ok = tk_snd_mbf(g_mbf_imu_estimates, &imu, (INT)sizeof(imu), TMO_POL) == E_OK && ok;
	ok = tk_snd_mbf(g_mbf_obstacle_events, &obstacle, (INT)sizeof(obstacle), TMO_POL) ==
	     E_OK && ok;
	ok = monitor_cycle(&controller) && safety->line.have_message && safety->imu.have_message &&
	     safety->have_ahead && safety->faults == 0U && !safety->armed &&
	     controller.state == CONTROLLER_WAITING && ok;
	ok = monitor_cycle(&controller) && !safety->armed &&
	     controller.state == CONTROLLER_WAITING && ok;
	return drain_test_buffers() && ok;
}

static BOOL monitor_fault_test(void)
{
	controller_state_t controller = {0};
	car_safety_t *safety = &controller.safety;
	car_safety_config_t config = test_config;
	line_observation_message_t line = {0};
	fault_message_t fault = {0};
	uint32_t now;
	BOOL ok;

	if (!read_clock(safety, &now) || !controller_state_init(&controller, &config, now))
		return FALSE;
	line.header = (vehicle_message_header_t){VEHICLE_MESSAGE_LINE_OBSERVATION,
						  VEHICLE_MESSAGE_VERSION, 1U, now};
	line.line_detected = 1U;
	line.confidence = 1000U;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line) - 1, TMO_POL) == E_OK;
	ok = !monitor_cycle(&controller) && (safety->faults & CAR_SAFETY_FAULT_EXTERNAL) != 0U &&
	     controller.state == CONTROLLER_FAULTED && ok;
	controller = (controller_state_t){0};
	ok = controller_state_init(&controller, &config, now) && ok;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) == E_OK && ok;
	line.header.sequence = 3U;
	ok = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) == E_OK && ok;
	ok = !monitor_cycle(&controller) && (safety->faults & CAR_SAFETY_FAULT_LINE) != 0U &&
	     controller.state == CONTROLLER_FAULTED && ok;
	controller = (controller_state_t){0};
	ok = controller_state_init(&controller, &config, now) && ok;
	fault.header.type = VEHICLE_MESSAGE_FAULT;
	fault.header.version = VEHICLE_MESSAGE_VERSION;
	fault.source = VEHICLE_FAULT_SENSOR;
	fault.code = VEHICLE_FAULT_CODE_IMU_SENSOR;
	ok = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL) == E_OK && ok;
	ok = !monitor_cycle(&controller) && (safety->faults & CAR_SAFETY_FAULT_EXTERNAL) != 0U &&
	     controller.state == CONTROLLER_FAULTED && ok;
	config.line_max_age_ms = 10U;
	config.imu_max_age_ms = 10U;
	config.ahead_max_age_ms = 10U;
	controller = (controller_state_t){0};
	ok = controller_state_init(&controller, &config, now - 11U) && ok;
	ok = !monitor_cycle(&controller) && (safety->faults & CAR_SAFETY_FAULT_LINE) != 0U &&
	     controller.state == CONTROLLER_FAULTED && ok;
	return drain_test_buffers() && ok;
}

BOOL controller_monitor_test(void)
{
	BOOL flow_ok;
	BOOL fault_ok;
	TEST_INIT();

	flow_ok = monitor_flow_test();
	fault_ok = monitor_fault_test();
	TEST_CHECK("VC-U08", flow_ok, "controller consumes copied sensor messages while waiting");
	TEST_CHECK("VC-U09", fault_ok, "invalid, missing and fault messages enter faulted state");
	TEST_SUMMARY("controller monitor");
	return flow_ok && fault_ok;
}
#endif
