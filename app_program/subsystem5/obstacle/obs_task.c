#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "obs_producer.h"
#include "obs_task.h"

#define OBS_TASK_PRIORITY 10
#define OBS_FAULT_RETRY_MS 1000U
/* Deadline for the very first ahead reading after the task starts, when
 * longer than ahead_max_age_ms. The servo's position is unknown at boot, so
 * the port waits a full 180-degree sweep (~380 ms) before the first ping, and
 * the radio is busy joining Wi-Fi on the same core: 500 ms was too tight and
 * faulted at random. Inside the controller's 3 s first-message window. */
#define OBS_FIRST_READING_MS 2500U

typedef struct {
	obs_producer_t producer;
	obs_task_mode_t mode;
	uint32_t mode_start_ms;
	uint32_t last_ahead_ms;
	uint32_t last_command_ms;
	uint32_t last_command_sequence;
	uint32_t last_clock_ms;
	ER delivery_error;
	bool have_ahead_in_mode;
	bool have_command;
	/* Set by the task at start: no ahead reading yet since boot, so the
	 * first one gets OBS_FIRST_READING_MS, and sensor errors before it are
	 * retried (a reset Pico finds the still-powered HC-SR04 mid-ping, echo
	 * high). Cleared by the first reading. */
	bool awaiting_first;
	ER startup_error;       /* last sensor error while awaiting_first */
} obs_task_state_t;

static obs_task_config_t s_config;
static obs_task_state_t s_state;
static bool s_started;

static bool interval_valid(uint32_t interval)
{
	return interval > 0U && interval <= INT32_MAX;
}

static uint16_t receive_commands(obs_task_state_t *state,
				 const obs_task_config_t *config, uint32_t now)
{
	unsigned i;
	for (i = 0U; i < VEHICLE_MB_OBSTACLE_COMMAND_CAPACITY; i++) {
		obstacle_command_message_t command;
		INT received = tk_rcv_mbf(g_mbf_obstacle_commands, &command, TMO_POL);
		if (received == E_TMOUT) break;
		if (received != (INT)sizeof(command) ||
		    command.header.type != VEHICLE_MESSAGE_OBSTACLE_COMMAND ||
		    command.header.version != VEHICLE_MESSAGE_VERSION ||
		    now - command.header.timestamp_ms > config->command_max_age_ms ||
		    (state->have_command &&
		     (command.header.sequence - state->last_command_sequence != 1U ||
		      command.header.timestamp_ms - state->last_command_ms > INT32_MAX)))
			return VEHICLE_FAULT_CODE_OBSTACLE_COMMAND;
		switch (command.command) {
		case VEHICLE_OBSTACLE_SCAN_REQUEST:
			if (state->mode == OBS_TASK_SCAN || !state->have_ahead_in_mode ||
			    now - state->last_ahead_ms > config->ahead_max_age_ms)
				return VEHICLE_FAULT_CODE_OBSTACLE_COMMAND;
			state->mode = OBS_TASK_SCAN;
			state->mode_start_ms = now;
			break;
		case VEHICLE_OBSTACLE_ABORT:
		case VEHICLE_OBSTACLE_RESET:
			if (state->mode == OBS_TASK_SCAN) {
				state->mode = OBS_TASK_AHEAD;
				state->mode_start_ms = now;
				state->have_ahead_in_mode = false;
			}
			break;
		case VEHICLE_OBSTACLE_BYPASS_COMPLETE:
			if (state->mode != OBS_TASK_AHEAD)
				return VEHICLE_FAULT_CODE_OBSTACLE_COMMAND;
			break;
		default:
			return VEHICLE_FAULT_CODE_OBSTACLE_COMMAND;
		}
		state->last_command_ms = command.header.timestamp_ms;
		state->last_command_sequence = command.header.sequence;
		state->have_command = true;
	}
	return 0U;
}

static uint16_t check_timeout(const obs_task_state_t *state,
			      const obs_task_config_t *config, uint32_t now)
{
	uint32_t since = state->mode == OBS_TASK_AHEAD && state->have_ahead_in_mode ?
			 state->last_ahead_ms : state->mode_start_ms;
	uint32_t limit = config->ahead_max_age_ms;

	if (state->mode == OBS_TASK_SCAN)
		return now - since > config->scan_timeout_ms ?
		       VEHICLE_FAULT_CODE_OBSTACLE_SCAN_TIMEOUT : 0U;
	if (state->awaiting_first && limit < OBS_FIRST_READING_MS)
		limit = OBS_FIRST_READING_MS;
	if (now - since > limit) {
		if (state->awaiting_first)
			tm_printf((UB*)"obstacle: no first ultrasonic reading within %u ms, last "
				  "sensor error %d (sensor unpowered, wiring, or echo stuck: "
				  "switch the robot off and on)\n", (unsigned)limit,
				  (int)state->startup_error);
		else
			tm_printf((UB*)"obstacle: no ultrasonic reading for %u ms (limit %u)\n",
				  (unsigned)(now - since), (unsigned)limit);
		return VEHICLE_FAULT_CODE_OBSTACLE_SENSOR;
	}
	return 0U;
}

static uint16_t process_poll(obs_task_state_t *state, const obs_task_config_t *config,
			     const obs_task_sample_t *sample, uint32_t now)
{
	obstacle_event_message_t event;
	uint32_t age_limit = state->mode == OBS_TASK_SCAN ?
			     config->scan_max_age_ms : config->ahead_max_age_ms;
	uint16_t fault = check_timeout(state, config, now);
	ER er;

	if (fault != 0U) return fault;
	if (!sample->ready) return 0U;
	if (now - sample->timestamp_ms > age_limit ||
	    sample->timestamp_ms - state->mode_start_ms > INT32_MAX)
		return VEHICLE_FAULT_CODE_OBSTACLE_SENSOR;
	if (state->mode == OBS_TASK_AHEAD) {
		if (!obs_producer_ahead(&state->producer, sample->distance_mm,
					sample->timestamp_ms, &event))
			return VEHICLE_FAULT_CODE_OBSTACLE_SENSOR;
	} else {
		if (!obs_producer_scan(&state->producer, sample->distances, sample->count,
				       sample->angle0_deg, sample->step_deg,
				       sample->timestamp_ms, &event))
			return VEHICLE_FAULT_CODE_OBSTACLE_SENSOR;
	}
	er = tk_snd_mbf(g_mbf_obstacle_events, &event, (INT)sizeof(event), TMO_POL);
	if (er != E_OK) {
		state->delivery_error = er;
		return VEHICLE_FAULT_CODE_OBSTACLE_DELIVERY;
	}
	if (state->mode == OBS_TASK_AHEAD) {
		state->last_ahead_ms = sample->timestamp_ms;
		state->have_ahead_in_mode = true;
		state->awaiting_first = false;
	} else {
		state->mode = OBS_TASK_AHEAD;
		state->mode_start_ms = now;
		state->have_ahead_in_mode = false;
	}
	return 0U;
}

static void report_obstacle_fault(uint16_t code, uint32_t timestamp_ms)
{
	fault_message_t fault = {0};
	bool sent = false;
	ER stop_error = s_config.stop(s_config.context);

	if (stop_error != E_OK)
		tm_printf((UB*)"obstacle sensor stop failed: %d\n", (int)stop_error);
	fault.header.type = VEHICLE_MESSAGE_FAULT;
	fault.header.version = VEHICLE_MESSAGE_VERSION;
	fault.header.sequence = 1U;
	fault.header.timestamp_ms = timestamp_ms;
	fault.source = code == VEHICLE_FAULT_CODE_OBSTACLE_COMMAND ||
		       code == VEHICLE_FAULT_CODE_OBSTACLE_DELIVERY ?
		       VEHICLE_FAULT_COMMUNICATION :
		       (code == VEHICLE_FAULT_CODE_OBSTACLE_CLOCK ||
			code == VEHICLE_FAULT_CODE_OBSTACLE_SCHEDULE ?
			VEHICLE_FAULT_SYSTEM : VEHICLE_FAULT_SENSOR);
	fault.code = code;
	fault.recoverable = 0U;
	tm_printf((UB*)"obstacle task stopped; fault code %u\n", (unsigned)code);
	for (;;) {
		ER er;
		if (!sent) {
			er = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
			if (er == E_OK) sent = true;
			else tm_printf((UB*)"obstacle fault delivery failed: %d\n", (int)er);
		}
		er = tk_dly_tsk(OBS_FAULT_RETRY_MS);
		if (er < E_OK) {
			tm_printf((UB*)"obstacle fault retry delay failed: %d\n", (int)er);
			er = tk_slp_tsk(TMO_FEVR);
			tm_printf((UB*)"obstacle fault sleep returned: %d\n", (int)er);
		}
	}
}

static void obs_task(INT stacd, void *exinf)
{
	SYSTIM now;
	uint32_t timestamp_ms = 0U;
	ER er;

	(void)stacd;
	(void)exinf;
	er = tk_get_otm(&now);
	if (er != E_OK) report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_CLOCK, timestamp_ms);
	timestamp_ms = now.lo;
	if (s_config.initialize != NULL) {
		er = s_config.initialize(s_config.context);
		if (er != E_OK) {
			tm_printf((UB*)"obstacle sensor initialization failed: %d\n", (int)er);
			report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_SENSOR, timestamp_ms);
		}
	}
	er = tk_get_otm(&now);
	if (er != E_OK) report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_CLOCK, timestamp_ms);
	timestamp_ms = now.lo;
	s_state.mode_start_ms = timestamp_ms;
	s_state.last_clock_ms = timestamp_ms;
	s_state.awaiting_first = true;
	for (;;) {
		obs_task_sample_t sample = {0};
		uint16_t fault;

		er = tk_get_otm(&now);
		if (er != E_OK || now.lo - s_state.last_clock_ms > INT32_MAX)
			report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_CLOCK, timestamp_ms);
		timestamp_ms = now.lo;
		s_state.last_clock_ms = timestamp_ms;
		fault = check_timeout(&s_state, &s_config, timestamp_ms);
		if (fault != 0U) report_obstacle_fault(fault, timestamp_ms);
		fault = receive_commands(&s_state, &s_config, timestamp_ms);
		if (fault != 0U) report_obstacle_fault(fault, timestamp_ms);
		er = s_config.poll(s_config.context, s_state.mode, &sample);
		if (er != E_OK && s_state.awaiting_first) {
			/* Before the first reading: retry until OBS_FIRST_READING_MS
			 * (check_timeout). The robot cannot arm without that reading. */
			if (s_state.startup_error == E_OK)
				tm_printf((UB*)"obstacle: ultrasonic not ready yet (%d), retrying\n",
					  (int)er);
			s_state.startup_error = er;
			sample.ready = false;
		} else if (er != E_OK) {
			tm_printf((UB*)"obstacle sensor poll failed: %d\n", (int)er);
			report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_SENSOR, timestamp_ms);
		}
		er = tk_get_otm(&now);
		if (er != E_OK || now.lo - s_state.last_clock_ms > INT32_MAX)
			report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_CLOCK, timestamp_ms);
		timestamp_ms = now.lo;
		s_state.last_clock_ms = timestamp_ms;
		fault = process_poll(&s_state, &s_config, &sample, timestamp_ms);
		if (fault != 0U) {
			if (fault == VEHICLE_FAULT_CODE_OBSTACLE_DELIVERY)
				tm_printf((UB*)"obstacle delivery failed: %d\n", (int)s_state.delivery_error);
			report_obstacle_fault(fault, timestamp_ms);
		}
		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) report_obstacle_fault(VEHICLE_FAULT_CODE_OBSTACLE_SCHEDULE, timestamp_ms);
	}
}

ER obs_task_start(const obs_task_config_t *config)
{
	obs_task_state_t initial = {0};
	T_CTSK task = {0};
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->poll == NULL || config->stop == NULL ||
	    !interval_valid(config->period_ms) || !interval_valid(config->ahead_max_age_ms) ||
	    !interval_valid(config->scan_max_age_ms) || !interval_valid(config->scan_timeout_ms) ||
	    !interval_valid(config->command_max_age_ms) ||
	    !obs_producer_init(&initial.producer, config->emergency_distance_mm))
		return E_PAR;
	if (g_mbf_obstacle_commands <= 0 || g_mbf_obstacle_events <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	s_config = *config;
	s_state = initial;
	task.task = (FP)obs_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = OBS_TASK_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
	/* The shared Echo GPIO dispatcher and its edge state are core-0 owned. */
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC1;
#endif
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK) tm_printf((UB*)"obstacle task cleanup failed: %d\n", (int)cleanup);
		else s_started = false;
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

#if APP_TEST
#include "../../test_report.h"

static BOOL drain_test_buffer(ID buffer)
{
	union {
		obstacle_command_message_t command;
		obstacle_event_message_t event;
	} message;
	INT result;

	do {
		result = tk_rcv_mbf(buffer, &message, TMO_POL);
	} while (result > 0);
	return result == E_TMOUT;
}

static BOOL send_test_command(uint8_t kind, uint32_t sequence, uint32_t timestamp)
{
	obstacle_command_message_t command = {0};
	command.header.type = VEHICLE_MESSAGE_OBSTACLE_COMMAND;
	command.header.version = VEHICLE_MESSAGE_VERSION;
	command.header.sequence = sequence;
	command.header.timestamp_ms = timestamp;
	command.command = kind;
	return tk_snd_mbf(g_mbf_obstacle_commands, &command,
			  (INT)sizeof(command), TMO_POL) == E_OK;
}

static BOOL task_flow_test(void)
{
	obs_task_state_t state = {0};
	obs_task_config_t config = {10U, 20U, 30U, 50U, 20U, 300U, NULL, NULL, NULL};
	obs_task_sample_t sample = {0};
	obstacle_event_message_t event;
	BOOL ok;
	unsigned i;

	ok = obs_producer_init(&state.producer, config.emergency_distance_mm);
	state.mode_start_ms = 100U;
	sample.ready = true;
	sample.timestamp_ms = 100U;
	sample.distance_mm = 250U;
	ok = process_poll(&state, &config, &sample, 100U) == 0U &&
	     tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) == (INT)sizeof(event) &&
	     event.obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING && event.emergency && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_SCAN_REQUEST, UINT32_MAX, 101U) &&
	     receive_commands(&state, &config, 101U) == 0U &&
	     state.mode == OBS_TASK_SCAN && ok;
	sample.ready = false;
	ok = process_poll(&state, &config, &sample, 105U) == 0U && ok;
	sample.ready = true;
	sample.timestamp_ms = 102U;
	sample.count = 9U;
	sample.angle0_deg = 80U;
	sample.step_deg = 5U;
	for (i = 0U; i < 9U; i++) sample.distances[i] = OBS_DIST_NONE;
	sample.distances[4] = 300U;
	ok = process_poll(&state, &config, &sample, 110U) == 0U &&
	     tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) == (INT)sizeof(event) &&
	     event.obstacle_class == VEHICLE_OBSTACLE_SCAN_FOUND &&
	     event.header.timestamp_ms == 102U && state.mode == OBS_TASK_AHEAD &&
	     !state.have_ahead_in_mode && ok;
	sample.timestamp_ms = 109U;
	sample.distance_mm = OBS_DIST_NONE;
	ok = process_poll(&state, &config, &sample, 111U) == VEHICLE_FAULT_CODE_OBSTACLE_SENSOR && ok;
	sample.timestamp_ms = 111U;
	ok = process_poll(&state, &config, &sample, 111U) == 0U &&
	     tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) == (INT)sizeof(event) &&
	     !event.emergency && state.have_ahead_in_mode && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_SCAN_REQUEST, 0U, 112U) &&
	     receive_commands(&state, &config, 112U) == 0U &&
	     state.mode == OBS_TASK_SCAN && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_ABORT, 1U, 113U) &&
	     receive_commands(&state, &config, 113U) == 0U &&
	     state.mode == OBS_TASK_AHEAD && !state.have_ahead_in_mode && ok;
	ok = drain_test_buffer(g_mbf_obstacle_commands) && ok;
	ok = drain_test_buffer(g_mbf_obstacle_events) && ok;
	state = (obs_task_state_t){0};
	ok = obs_producer_init(&state.producer, config.emergency_distance_mm) && ok;
	state.mode_start_ms = UINT32_MAX - 3U;
	sample.timestamp_ms = UINT32_MAX - 2U;
	sample.distance_mm = 500U;
	ok = process_poll(&state, &config, &sample, sample.timestamp_ms) == 0U &&
	     tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) == (INT)sizeof(event) && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_SCAN_REQUEST, UINT32_MAX, UINT32_MAX - 1U) &&
	     receive_commands(&state, &config, UINT32_MAX - 1U) == 0U && ok;
	sample.timestamp_ms = UINT32_MAX;
	ok = process_poll(&state, &config, &sample, 1U) == 0U &&
	     tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) == (INT)sizeof(event) &&
	     event.obstacle_class == VEHICLE_OBSTACLE_SCAN_FOUND && ok;
	sample.timestamp_ms = 2U;
	ok = process_poll(&state, &config, &sample, 2U) == 0U &&
	     tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_POL) == (INT)sizeof(event) && ok;
	ok = drain_test_buffer(g_mbf_obstacle_commands) && ok;
	ok = drain_test_buffer(g_mbf_obstacle_events) && ok;
	return ok;
}

static BOOL task_fault_test(void)
{
	obs_task_state_t state = {0};
	obs_task_config_t config = {10U, 20U, 30U, 50U, 20U, 300U, NULL, NULL, NULL};
	obs_task_sample_t sample = {0};
	BOOL ok;
	unsigned i;

	ok = obs_producer_init(&state.producer, config.emergency_distance_mm);
	state.mode_start_ms = 100U;
	ok = send_test_command(VEHICLE_OBSTACLE_SCAN_REQUEST, 1U, 100U) &&
	     receive_commands(&state, &config, 100U) == VEHICLE_FAULT_CODE_OBSTACLE_COMMAND && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_RESET, 1U, 100U) &&
	     receive_commands(&state, &config, 110U) == 0U &&
	     state.mode_start_ms == 100U && ok;
	ok = process_poll(&state, &config, &sample, 121U) == VEHICLE_FAULT_CODE_OBSTACLE_SENSOR && ok;
	state = (obs_task_state_t){0};
	ok = obs_producer_init(&state.producer, config.emergency_distance_mm) && ok;
	state.mode_start_ms = 100U;
	ok = send_test_command(VEHICLE_OBSTACLE_SCAN_REQUEST, 1U, 79U) &&
	     receive_commands(&state, &config, 100U) == VEHICLE_FAULT_CODE_OBSTACLE_COMMAND && ok;
	sample.ready = true;
	sample.timestamp_ms = 100U;
	sample.distance_mm = 0U;
	ok = process_poll(&state, &config, &sample, 100U) == VEHICLE_FAULT_CODE_OBSTACLE_SENSOR && ok;
	sample.distance_mm = 500U;
	ok = process_poll(&state, &config, &sample, 100U) == 0U && ok;
	ok = drain_test_buffer(g_mbf_obstacle_events) && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_SCAN_REQUEST, 1U, 100U) &&
	     receive_commands(&state, &config, 100U) == 0U && ok;
	sample.ready = false;
	ok = process_poll(&state, &config, &sample, 151U) ==
	     VEHICLE_FAULT_CODE_OBSTACLE_SCAN_TIMEOUT && ok;
	ok = send_test_command(VEHICLE_OBSTACLE_ABORT, 3U, 151U) &&
	     receive_commands(&state, &config, 151U) == VEHICLE_FAULT_CODE_OBSTACLE_COMMAND && ok;
	ok = drain_test_buffer(g_mbf_obstacle_commands) && ok;

	state = (obs_task_state_t){0};
	ok = obs_producer_init(&state.producer, config.emergency_distance_mm) && ok;
	state.mode_start_ms = 200U;
	sample.ready = true;
	sample.distance_mm = OBS_DIST_NONE;
	for (i = 0U; i < VEHICLE_MB_OBSTACLE_CAPACITY; i++) {
		sample.timestamp_ms = 200U + i;
		ok = process_poll(&state, &config, &sample, sample.timestamp_ms) == 0U && ok;
	}
	sample.timestamp_ms = 204U;
	ok = process_poll(&state, &config, &sample, 204U) == VEHICLE_FAULT_CODE_OBSTACLE_DELIVERY &&
	     state.delivery_error == E_TMOUT && ok;
	ok = drain_test_buffer(g_mbf_obstacle_events) && ok;
	return ok;
}

BOOL obs_task_test(void)
{
	BOOL flow_ok;
	BOOL fault_ok;
	TEST_INIT();

	flow_ok = task_flow_test();
	fault_ok = task_fault_test();
	TEST_CHECK("OBS-U05", flow_ok, "ahead monitoring and requested scan");
	TEST_CHECK("OBS-U06", fault_ok, "sensor, command, scan and delivery faults");
	TEST_SUMMARY("obstacle task");
	return flow_ok && fault_ok;
}
#endif
