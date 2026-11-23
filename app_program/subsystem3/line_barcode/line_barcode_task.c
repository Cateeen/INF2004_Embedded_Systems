#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "line_barcode_task.h"

#define LINE_BARCODE_PRIORITY 10
#define LINE_FAULT_RETRY_MS 1000U

typedef struct {
	barcode_scanner_t scanner;
	barcode_scanner_config_t barcode_config;
	wheel_odometry_message_t pending;
	uint32_t start_ms;
	uint32_t max_age_ms;
	uint32_t last_odom_ms;
	uint32_t last_odom_sequence;
	uint32_t last_left;
	uint32_t last_right;
	uint32_t mean_count;
	int32_t half_count;
	uint32_t line_sequence;
	uint32_t barcode_sequence;
	uint32_t last_sample_ms;
	ER delivery_error;
	bool have_pending;
	bool have_odom;
	bool scanner_ready;
	bool have_sample;
} line_state_t;

static line_barcode_task_config_t s_config;
static line_state_t s_state;
static bool s_started;

static int32_t signed_bits(uint32_t bits)
{
	return bits <= INT32_MAX ? (int32_t)bits :
	       INT32_MIN + (int32_t)(bits - 0x80000000U);
}

static bool add_odometry(line_state_t *state, const wheel_odometry_message_t *message)
{
	uint32_t left = (uint32_t)message->left_count;
	uint32_t right = (uint32_t)message->right_count;

	if (message->header.type != VEHICLE_MESSAGE_WHEEL_ODOMETRY ||
	    message->header.version != VEHICLE_MESSAGE_VERSION || message->valid != 1U)
		return false;
	if (state->have_odom) {
		uint32_t ld = left - state->last_left;
		uint32_t rd = right - state->last_right;
		uint32_t elapsed = message->header.timestamp_ms - state->last_odom_ms;
		int64_t sum;
		int64_t whole;
		int32_t remainder;
		if (message->header.sequence - state->last_odom_sequence != 1U ||
		    elapsed == 0U || elapsed > INT32_MAX ||
		    ld == 0x80000000U || rd == 0x80000000U)
			return false;
		sum = (int64_t)signed_bits(ld) + signed_bits(rd) + state->half_count;
		whole = sum / 2;
		remainder = (int32_t)(sum % 2);
		if (remainder < 0) {
			whole--;
			remainder += 2;
		}
		state->mean_count += (uint32_t)whole;
		state->half_count = remainder;
	}
	state->last_left = left;
	state->last_right = right;
	state->last_odom_ms = message->header.timestamp_ms;
	state->last_odom_sequence = message->header.sequence;
	state->have_odom = true;
	return true;
}

/* A future-dated message is retained until a later sensor sample. */
static uint16_t receive_odometry(line_state_t *state, uint32_t timestamp_ms,
				 bool *updated)
{
	wheel_odometry_message_t message;
	INT received;

	*updated = false;
	for (;;) {
		uint32_t ahead;
		if (state->have_pending) {
			message = state->pending;
		} else {
			received = tk_rcv_mbf(g_mbf_line_odometry, &message, TMO_POL);
			if (received == E_TMOUT) break;
			if (received != (INT)sizeof(message))
				return VEHICLE_FAULT_CODE_LINE_ODOMETRY;
		}
		if (message.header.type != VEHICLE_MESSAGE_WHEEL_ODOMETRY ||
		    message.header.version != VEHICLE_MESSAGE_VERSION || message.valid != 1U)
			return VEHICLE_FAULT_CODE_LINE_ODOMETRY;
		ahead = message.header.timestamp_ms - timestamp_ms;
		if (ahead != 0U && ahead <= INT32_MAX) {
			state->pending = message;
			state->have_pending = true;
			break;
		}
		state->have_pending = false;
		if (timestamp_ms - message.header.timestamp_ms > state->max_age_ms ||
		    !add_odometry(state, &message))
			return VEHICLE_FAULT_CODE_LINE_ODOMETRY;
		*updated = true;
	}
	if (state->have_odom) {
		if (timestamp_ms - state->last_odom_ms > state->max_age_ms)
			return VEHICLE_FAULT_CODE_LINE_ODOMETRY;
	} else if (timestamp_ms - state->start_ms > state->max_age_ms) {
		return VEHICLE_FAULT_CODE_LINE_ODOMETRY;
	}
	return 0U;
}

static uint16_t process_readings(line_state_t *state, const ir_readings_t *readings,
				 uint32_t timestamp_ms)
{
	line_observation_message_t line = {0};
	barcode_event_message_t barcode = {0};
	uint8_t position;
	bool odom_updated;
	uint16_t fault;
	ER er;

	if (state->have_sample) {
		uint32_t elapsed = timestamp_ms - state->last_sample_ms;
		if (elapsed == 0U || elapsed > INT32_MAX)
			return VEHICLE_FAULT_CODE_LINE_CLOCK;
	}
	fault = receive_odometry(state, timestamp_ms, &odom_updated);
	if (fault != 0U) return fault;
	if (state->have_odom) {
		int32_t count = signed_bits(state->mean_count);
		if (!state->scanner_ready) {
			if (!barcode_scanner_init(&state->scanner, &state->barcode_config,
						  readings->barcode.black, count))
				return VEHICLE_FAULT_CODE_LINE_BARCODE;
			state->scanner_ready = true;
		} else {
			if (readings->barcode.black != state->scanner.last_black) {
				uint32_t since_sample = state->last_odom_ms - state->last_sample_ms;
				if (!odom_updated || since_sample == 0U || since_sample > INT32_MAX)
					return VEHICLE_FAULT_CODE_LINE_ODOMETRY;
			}
			if (!barcode_scanner_update(&state->scanner, readings->barcode.black, count))
				return VEHICLE_FAULT_CODE_LINE_BARCODE;
		}
	}
	position = line_following_position(readings);
	line.header.type = VEHICLE_MESSAGE_LINE_OBSERVATION;
	line.header.version = VEHICLE_MESSAGE_VERSION;
	line.header.sequence = ++state->line_sequence;
	line.header.timestamp_ms = timestamp_ms;
	line.lateral_error = position == POS_LEFT ? -1 : position == POS_RIGHT ? 1 : 0;
	line.confidence = position == POS_LOST ? 0U : 1000U;
	line.line_detected = position != POS_LOST;
	line.intersection_detected = position == POS_JUNCTION;
	er = tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL);
	if (er != E_OK) {
		state->delivery_error = er;
		return VEHICLE_FAULT_CODE_LINE_DELIVERY;
	}
	if (state->scanner_ready && state->scanner.result_ready) {
		uint8_t payload[BARCODE_MAX_PAYLOAD];
		uint8_t length;
		unsigned i;
		if (!barcode_scanner_take_result(&state->scanner, payload, sizeof(payload), &length) ||
		    length > sizeof(barcode.payload))
			return VEHICLE_FAULT_CODE_LINE_BARCODE;
		barcode.header.type = VEHICLE_MESSAGE_BARCODE_EVENT;
		barcode.header.version = VEHICLE_MESSAGE_VERSION;
		barcode.header.sequence = ++state->barcode_sequence;
		barcode.header.timestamp_ms = timestamp_ms;
		for (i = 0U; i < length; i++) barcode.payload[i] = (char)payload[i];
		barcode.length = length;
		barcode.valid = 1U;
		er = tk_snd_mbf(g_mbf_barcode_events, &barcode, (INT)sizeof(barcode), TMO_POL);
		if (er != E_OK) {
			state->delivery_error = er;
			return VEHICLE_FAULT_CODE_LINE_DELIVERY;
		}
	}
	state->last_sample_ms = timestamp_ms;
	state->have_sample = true;
	return 0U;
}

static void report_line_fault(uint16_t code, uint32_t timestamp_ms)
{
	fault_message_t fault = {0};
	bool sent = false;

	fault.header.type = VEHICLE_MESSAGE_FAULT;
	fault.header.version = VEHICLE_MESSAGE_VERSION;
	fault.header.sequence = 1U;
	fault.header.timestamp_ms = timestamp_ms;
	fault.source = code == VEHICLE_FAULT_CODE_LINE_DELIVERY ||
		       code == VEHICLE_FAULT_CODE_LINE_ODOMETRY ?
		       VEHICLE_FAULT_COMMUNICATION :
		       (code == VEHICLE_FAULT_CODE_LINE_CLOCK ||
			code == VEHICLE_FAULT_CODE_LINE_SCHEDULE ?
			VEHICLE_FAULT_SYSTEM : VEHICLE_FAULT_SENSOR);
	fault.code = code;
	fault.recoverable = 0U;
	tm_printf((UB*)"line/barcode stopped; fault code %u\n", (unsigned)code);
	for (;;) {
		ER er;
		if (!sent) {
			er = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
			if (er == E_OK) sent = true;
			else tm_printf((UB*)"line/barcode fault delivery failed: %d\n", (int)er);
		}
		er = tk_dly_tsk(LINE_FAULT_RETRY_MS);
		if (er < E_OK) {
			tm_printf((UB*)"line fault retry delay failed: %d\n", (int)er);
			er = tk_slp_tsk(TMO_FEVR);
			tm_printf((UB*)"line fault sleep returned: %d\n", (int)er);
		}
	}
}

static void line_barcode_task(INT stacd, void *exinf)
{
	SYSTIM now;
	uint32_t timestamp_ms = 0U;
	ER er;

	(void)stacd;
	(void)exinf;
	er = tk_get_otm(&now);
	if (er < E_OK) report_line_fault(VEHICLE_FAULT_CODE_LINE_CLOCK, timestamp_ms);
	timestamp_ms = now.lo;
	s_state.start_ms = timestamp_ms;
	if (s_config.initialize != NULL) {
		er = s_config.initialize(s_config.read_context);
		if (er != E_OK) {
			tm_printf((UB*)"line sensor initialization failed: %d\n", (int)er);
			report_line_fault(VEHICLE_FAULT_CODE_LINE_SENSOR, timestamp_ms);
		}
	}
	for (;;) {
		ir_readings_t readings;
		uint32_t sample_ms;
		uint16_t fault;
		er = s_config.read(s_config.read_context, &readings, &sample_ms);
		if (er != E_OK) {
			tm_printf((UB*)"line sensor read failed: %d\n", (int)er);
			report_line_fault(VEHICLE_FAULT_CODE_LINE_SENSOR, timestamp_ms);
		}
		er = tk_get_otm(&now);
		if (er < E_OK) report_line_fault(VEHICLE_FAULT_CODE_LINE_CLOCK, timestamp_ms);
		timestamp_ms = now.lo;
		if (timestamp_ms - sample_ms > s_config.odometry_max_age_ms)
			report_line_fault(VEHICLE_FAULT_CODE_LINE_SENSOR, timestamp_ms);
		fault = process_readings(&s_state, &readings, sample_ms);
		if (fault != 0U) {
			if (fault == VEHICLE_FAULT_CODE_LINE_DELIVERY)
				tm_printf((UB*)"line/barcode delivery failed: %d\n",
					  (int)s_state.delivery_error);
			report_line_fault(fault, timestamp_ms);
		}
		er = tk_dly_tsk(s_config.period_ms);
		if (er < E_OK) report_line_fault(VEHICLE_FAULT_CODE_LINE_SCHEDULE, timestamp_ms);
	}
}

ER line_barcode_task_start(const line_barcode_task_config_t *config)
{
	T_CTSK task = {0};
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->read == NULL || config->period_ms == 0U ||
	    config->period_ms > INT32_MAX || config->odometry_max_age_ms == 0U ||
	    config->odometry_max_age_ms > INT32_MAX ||
	    !barcode_scanner_init(&s_state.scanner, &config->barcode, false, 0))
		return E_PAR;
	if (g_mbf_line_odometry <= 0 || g_mbf_line_observations <= 0 ||
	    g_mbf_barcode_events <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	s_config = *config;
	s_state.barcode_config = config->barcode;
	s_state.max_age_ms = config->odometry_max_age_ms;
	task.task = (FP)line_barcode_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = LINE_BARCODE_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er < E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup < E_OK) tm_printf((UB*)"line task cleanup failed: %d\n", (int)cleanup);
		else s_started = false;
		return er;
	}
	return E_OK;
}

#if APP_TEST
#include "../../test_report.h"

static line_state_t s_test_state;

static BOOL test_drain(ID buffer)
{
	union {
		wheel_odometry_message_t odometry;
		line_observation_message_t line;
		barcode_event_message_t barcode;
	} message;
	INT result;

	do {
		result = tk_rcv_mbf(buffer, &message, TMO_POL);
	} while (result > 0);
	return result == E_TMOUT;
}

static BOOL test_sample(line_state_t *state, ir_readings_t *readings,
			uint32_t count, uint32_t sequence, uint32_t timestamp)
{
	wheel_odometry_message_t odometry = {0};
	line_observation_message_t line;

	odometry.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	odometry.header.version = VEHICLE_MESSAGE_VERSION;
	odometry.header.sequence = sequence;
	odometry.header.timestamp_ms = timestamp;
	odometry.left_count = signed_bits(count);
	odometry.right_count = signed_bits(count);
	odometry.valid = 1U;
	if (tk_snd_mbf(g_mbf_line_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) != E_OK ||
	    process_readings(state, readings, timestamp) != 0U ||
	    tk_rcv_mbf(g_mbf_line_observations, &line, TMO_POL) != (INT)sizeof(line))
		return FALSE;
	return line.header.type == VEHICLE_MESSAGE_LINE_OBSERVATION &&
	       line.line_detected == 1U && line.lateral_error == -1;
}

static BOOL test_character(line_state_t *state, ir_readings_t *readings,
			   const char *pattern, uint32_t *count,
			   uint32_t *sequence, uint32_t *timestamp, int direction)
{
	unsigned i;
	*count += (uint32_t)(direction * 10);
	*timestamp += 10U;
	readings->barcode.black = true;
	if (!test_sample(state, readings, *count, ++*sequence, *timestamp)) return FALSE;
	for (i = 0U; i < BARCODE_ELEMENTS; i++) {
		unsigned index = direction > 0 ? i : BARCODE_ELEMENTS - 1U - i;
		int width = pattern[index] == 'w' ? 20 : 10;
		*count += (uint32_t)(direction * width);
		*timestamp += 10U;
		readings->barcode.black = !readings->barcode.black;
		if (!test_sample(state, readings, *count, ++*sequence, *timestamp)) return FALSE;
	}
	return TRUE;
}

static BOOL task_frame_test(int direction)
{
	ir_readings_t readings = {0};
	barcode_event_message_t result;
	uint32_t count = direction > 0 ? (uint32_t)INT32_MAX - 40U :
			 (uint32_t)INT32_MIN + 40U;
	uint32_t sequence = UINT32_MAX - 10U;
	uint32_t timestamp = 100U;

	s_test_state = (line_state_t){0};
	s_test_state.barcode_config = (barcode_scanner_config_t){10U, 0U, 0U, 0U, 0U};
	s_test_state.max_age_ms = 50U;
	readings.center.black = true;
	if (!test_sample(&s_test_state, &readings, count, sequence, timestamp) ||
	    !test_character(&s_test_state, &readings, "nwnnwnwnn", &count, &sequence, &timestamp, direction) ||
	    !test_character(&s_test_state, &readings, "wnnnnwnnw", &count, &sequence, &timestamp, direction) ||
	    !test_character(&s_test_state, &readings, "nwnnwnwnn", &count, &sequence, &timestamp, direction))
		return FALSE;
	return tk_rcv_mbf(g_mbf_barcode_events, &result, TMO_POL) == (INT)sizeof(result) &&
	       result.header.type == VEHICLE_MESSAGE_BARCODE_EVENT && result.valid == 1U &&
	       result.length == 1U && result.payload[0] == 'A';
}

static BOOL task_mean_test(void)
{
	wheel_odometry_message_t message = {0};

	s_test_state = (line_state_t){0};
	message.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = UINT32_MAX;
	message.header.timestamp_ms = UINT32_MAX - 5U;
	message.left_count = INT32_MAX - 1;
	message.right_count = INT32_MIN + 1;
	message.valid = 1U;
	if (!add_odometry(&s_test_state, &message)) return FALSE;
	message.header.sequence = 0U;
	message.header.timestamp_ms = 4U;
	message.left_count = INT32_MIN;
	message.right_count = INT32_MIN + 4;
	if (!add_odometry(&s_test_state, &message) ||
	    s_test_state.mean_count != 2U || s_test_state.half_count != 1)
		return FALSE;
	message.header.sequence = 1U;
	message.header.timestamp_ms = 5U;
	message.left_count = INT32_MIN + 1;
	if (!add_odometry(&s_test_state, &message) ||
	    s_test_state.mean_count != 3U || s_test_state.half_count != 0)
		return FALSE;
	message.header.sequence = 3U;
	message.header.timestamp_ms = 6U;
	if (add_odometry(&s_test_state, &message) || s_test_state.mean_count != 3U)
		return FALSE;
	message.header.sequence = 2U;
	message.left_count = INT32_MIN;
	if (!add_odometry(&s_test_state, &message) ||
	    s_test_state.mean_count != 2U || s_test_state.half_count != 1)
		return FALSE;
	message.header.sequence = 3U;
	message.header.timestamp_ms = 7U;
	message.left_count = INT32_MIN + 1;
	return add_odometry(&s_test_state, &message) &&
	       s_test_state.mean_count == 3U && s_test_state.half_count == 0;
}

static BOOL task_fault_test(void)
{
	ir_readings_t readings = {0};
	wheel_odometry_message_t odometry = {0};
	line_observation_message_t line;
	BOOL ok;
	unsigned i;

	s_test_state = (line_state_t){0};
	s_test_state.barcode_config = (barcode_scanner_config_t){10U, 0U, 0U, 0U, 0U};
	s_test_state.max_age_ms = 50U;
	s_test_state.start_ms = 100U;
	readings.center.black = true;
	ok = process_readings(&s_test_state, &readings, 100U) == 0U;
	ok = tk_rcv_mbf(g_mbf_line_observations, &line, TMO_POL) == (INT)sizeof(line) && ok;
	odometry.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	odometry.header.version = VEHICLE_MESSAGE_VERSION;
	odometry.header.sequence = 1U;
	odometry.header.timestamp_ms = 110U;
	odometry.valid = 1U;
	ok = tk_snd_mbf(g_mbf_line_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_readings(&s_test_state, &readings, 105U) == 0U &&
	     s_test_state.have_pending && !s_test_state.have_odom && ok;
	ok = tk_rcv_mbf(g_mbf_line_observations, &line, TMO_POL) == (INT)sizeof(line) && ok;
	ok = process_readings(&s_test_state, &readings, 110U) == 0U &&
	     s_test_state.scanner_ready && !s_test_state.have_pending && ok;
	ok = tk_rcv_mbf(g_mbf_line_observations, &line, TMO_POL) == (INT)sizeof(line) && ok;
	readings.barcode.black = true;
	ok = process_readings(&s_test_state, &readings, 111U) ==
	     VEHICLE_FAULT_CODE_LINE_ODOMETRY && ok;

	s_test_state = (line_state_t){0};
	s_test_state.barcode_config = (barcode_scanner_config_t){10U, 0U, 0U, 0U, 0U};
	s_test_state.max_age_ms = 50U;
	readings.barcode.black = false;
	if (!test_sample(&s_test_state, &readings, 0U, 1U, 100U)) ok = FALSE;
	for (i = 0U; i < VEHICLE_MB_LINE_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_line_observations, &line, (INT)sizeof(line), TMO_POL) != E_OK)
			ok = FALSE;
	}
	if (process_readings(&s_test_state, &readings, 110U) != VEHICLE_FAULT_CODE_LINE_DELIVERY ||
	    s_test_state.delivery_error != E_TMOUT)
		ok = FALSE;
	ok = test_drain(g_mbf_line_odometry) && ok;
	ok = test_drain(g_mbf_line_observations) && ok;
	ok = test_drain(g_mbf_barcode_events) && ok;

	s_test_state = (line_state_t){0};
	s_test_state.barcode_config = (barcode_scanner_config_t){10U, 0U, 0U, 0U, 0U};
	s_test_state.max_age_ms = 50U;
	if (!test_sample(&s_test_state, &readings, 0U, 1U, 100U)) ok = FALSE;
	odometry.header.sequence = 2U;
	odometry.header.timestamp_ms = 101U;
	if (tk_snd_mbf(g_mbf_line_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) != E_OK)
		ok = FALSE;
	if (process_readings(&s_test_state, &readings, 200U) != VEHICLE_FAULT_CODE_LINE_ODOMETRY)
		ok = FALSE;
	ok = test_drain(g_mbf_line_odometry) && ok;
	ok = test_drain(g_mbf_line_observations) && ok;
	return ok;
}

BOOL line_barcode_task_test(void)
{
	BOOL frame_ok;
	BOOL mean_ok;
	BOOL fault_ok;
	TEST_INIT();

	frame_ok = task_frame_test(1);
	frame_ok = task_frame_test(-1) && frame_ok;
	mean_ok = task_mean_test();
	fault_ok = task_fault_test();
	frame_ok = test_drain(g_mbf_line_odometry) && frame_ok;
	frame_ok = test_drain(g_mbf_line_observations) && frame_ok;
	frame_ok = test_drain(g_mbf_barcode_events) && frame_ok;
	TEST_CHECK("LB-U01", frame_ok, "simulated line and barcode messages");
	TEST_CHECK("LB-U02", mean_ok, "wheel-count and sequence wrap");
	TEST_CHECK("LB-U03", fault_ok, "odometry synchronization and full queues");
	TEST_SUMMARY("line/barcode task");
	return frame_ok && mean_ok && fault_ok;
}
#endif
