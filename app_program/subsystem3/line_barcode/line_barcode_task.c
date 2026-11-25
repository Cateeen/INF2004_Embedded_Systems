#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "line_barcode_task.h"
#include "straddle_junction.h"

#define LINE_BARCODE_PRIORITY 9
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
	float count_rate;       /* mean counts per ms over the last odometry period */
	float mm_per_count;     /* > 0 with junction.initialized: detector on */
	straddle_junction_t junction;
	uint32_t line_sequence;
	uint32_t barcode_sequence;
	uint32_t last_sample_ms;
	ER delivery_error;
	bool have_pending;
	bool have_odom;
	bool scanner_ready;
	bool have_sample;
	bool was_in_frame;      /* scanner was inside a barcode frame last sample */
	bool bc_black;          /* barcode colour after hysteresis */
	bool bc_deb_black;      /* ... after the debounce (what the scanner sees) */
	bool bc_deb_pending;
	int32_t bc_deb_start;   /* count where the pending change began */
	bool line_centre_black; /* line sensor colours after hysteresis */
	bool line_right_black;
	bool all_black;         /* all three sensors black ... */
	uint32_t all_black_since_ms;    /* ... since this sample */
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
		state->count_rate = (float)((int64_t)signed_bits(ld) + signed_bits(rd)) *
				    0.5f / (float)elapsed;
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

/*
 * Wheel count at the sample time: the last odometry count plus the travel
 * since it at the last measured rate. Odometry arrives every 50 ms but the
 * sensors are sampled every 20 ms, so without this a barcode or junction
 * edge between odometry updates would be placed up to 50 ms of travel late
 * (it used to be a fault instead). The extrapolation is capped at the
 * odometry age limit, which is enforced separately.
 */
static int32_t sample_count(const line_state_t *state, uint32_t timestamp_ms)
{
	uint32_t since = timestamp_ms - state->last_odom_ms;

	if (!state->have_odom) return 0;
	if (since > INT32_MAX) since = 0U;      /* odometry newer than the sample */
	if (since > state->max_age_ms) since = state->max_age_ms;
	return signed_bits(state->mean_count +
			   (uint32_t)(int32_t)__builtin_lroundf(state->count_rate * (float)since));
}

/* Barcode colour: the reader's threshold, or hysteresis on the raw value. */
static bool barcode_colour(const line_barcode_task_config_t *config, bool previous,
			   const ir_readings_t *readings)
{
	if (config->barcode_black_adc == 0U) return readings->barcode.black;
	if (readings->barcode.raw >= config->barcode_black_adc) return true;
	if (readings->barcode.raw <= config->barcode_white_adc) return false;
	return previous;
}

/* Accept a colour change once the wheels have moved debounce_counts with it
 * held; *edge is where it began (as the bench test), else count. */
static bool barcode_debounce(line_state_t *state, uint32_t debounce_counts, bool black,
			     int32_t count, int32_t *edge)
{
	int32_t moved;
	uint32_t distance;

	*edge = count;
	if (debounce_counts == 0U) {
		state->bc_deb_black = black;
		return black;
	}
	if (black == state->bc_deb_black) {
		state->bc_deb_pending = false;
		return state->bc_deb_black;
	}
	if (!state->bc_deb_pending) {
		state->bc_deb_pending = true;
		state->bc_deb_start = count;
	}
	moved = (int32_t)((uint32_t)count - (uint32_t)state->bc_deb_start);
	distance = moved < 0 ? 0U - (uint32_t)moved : (uint32_t)moved;
	if (distance < debounce_counts) return state->bc_deb_black;
	state->bc_deb_black = black;
	state->bc_deb_pending = false;
	*edge = state->bc_deb_start;
	return black;
}

/* Line sensor colours: the reader's threshold, or hysteresis on the raw
 * values (config line_black_adc/line_white_adc). */
static void line_colours(const line_barcode_task_config_t *config, line_state_t *state,
			 ir_readings_t *readings)
{
	if (config->line_black_adc == 0U) return;
	if (readings->center.raw >= config->line_black_adc) state->line_centre_black = true;
	else if (readings->center.raw <= config->line_white_adc) state->line_centre_black = false;
	if (readings->right.raw >= config->line_black_adc) state->line_right_black = true;
	else if (readings->right.raw <= config->line_white_adc) state->line_right_black = false;
	readings->center.black = state->line_centre_black;
	readings->right.black = state->line_right_black;
}

static uint16_t process_readings(line_state_t *state, const ir_readings_t *raw_readings,
				 uint32_t timestamp_ms)
{
	ir_readings_t adjusted = *raw_readings;
	const ir_readings_t *readings = &adjusted;
	line_observation_message_t line = {0};
	barcode_event_message_t barcode = {0};
	uint8_t position;
	bool odom_updated;
	bool junction;
	uint16_t fault;
	ER er;

	if (state->have_sample) {
		uint32_t elapsed = timestamp_ms - state->last_sample_ms;
		if (elapsed == 0U || elapsed > INT32_MAX)
			return VEHICLE_FAULT_CODE_LINE_CLOCK;
	}
	line_colours(&s_config, state, &adjusted);
	fault = receive_odometry(state, timestamp_ms, &odom_updated);
	if (fault != 0U) return fault;
	(void)odom_updated;
	junction = false;
	if (state->have_odom) {
		int32_t count = sample_count(state, timestamp_ms);
		bool black;

		if (!state->scanner_ready) {
			black = s_config.barcode_black_adc == 0U ? readings->barcode.black :
				readings->barcode.raw >= s_config.barcode_black_adc;
			if (!barcode_scanner_init(&state->scanner, &state->barcode_config,
						  black, count))
				return VEHICLE_FAULT_CODE_LINE_BARCODE;
			state->bc_black = state->bc_deb_black = black;
			state->bc_deb_pending = false;
			state->scanner_ready = true;
		} else {
			int32_t edge;

			black = barcode_colour(&s_config, state->bc_black, readings);
			state->bc_black = black;
			black = barcode_debounce(state, s_config.barcode_debounce_counts, black,
						 count, &edge);
			if (!barcode_scanner_update(&state->scanner, black, edge))
				return VEHICLE_FAULT_CODE_LINE_BARCODE;
		}
		if (state->scanner.in_frame && !state->was_in_frame &&
		    s_config.barcode_started != NULL)
			s_config.barcode_started(s_config.barcode_started_context);
		state->was_in_frame = state->scanner.in_frame;
		if (state->junction.initialized)
			junction = straddle_junction_update(
				&state->junction, readings->center.black,
				readings->right.black,
				(float)count * state->mm_per_count) ==
				STRADDLE_JUNCTION_DETECTED;
	}
	position = line_following_position(readings);
	line.header.type = VEHICLE_MESSAGE_LINE_OBSERVATION;
	line.header.version = VEHICLE_MESSAGE_VERSION;
	line.header.sequence = ++state->line_sequence;
	line.header.timestamp_ms = timestamp_ms;
	line.lateral_error = position == POS_LEFT ? -1 : position == POS_RIGHT ? 1 : 0;
	line.confidence = position == POS_LOST ? 0U : 1000U;
	line.line_detected = position != POS_LOST;
	/* A confirmed junction (both line sensors black for a few mm; the
	 * sensors straddle the tape, so only a crossing tape covers both). */
	line.intersection_detected = junction ? 1U : 0U;
	if (!(readings->barcode.black && readings->center.black && readings->right.black)) {
		state->all_black = false;
	} else if (!state->all_black) {
		state->all_black = true;
		state->all_black_since_ms = timestamp_ms;
	}
	line.picked_up = state->all_black &&
			 timestamp_ms - state->all_black_since_ms >= LINE_PICKUP_MS ? 1U : 0U;
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
	    !__builtin_isfinite(config->junction_mm_per_count) ||
	    config->junction_mm_per_count < 0.0f ||
	    (config->barcode_black_adc != 0U &&
	     (config->barcode_white_adc >= config->barcode_black_adc ||
	      config->barcode_black_adc > 4095U)) ||
	    (config->line_black_adc != 0U &&
	     (config->line_white_adc >= config->line_black_adc ||
	      config->line_black_adc > 4095U)) ||
	    !barcode_scanner_init(&s_state.scanner, &config->barcode, false, 0))
		return E_PAR;
	if (g_mbf_line_odometry <= 0 || g_mbf_line_observations <= 0 ||
	    g_mbf_barcode_events <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	s_config = *config;
	s_state.barcode_config = config->barcode;
	s_state.max_age_ms = config->odometry_max_age_ms;
	if (config->junction_mm_per_count > 0.0f) {
		static const straddle_junction_config_t junction_config =
			STRADDLE_JUNCTION_DEFAULT_CONFIG;

		if (!straddle_junction_init(&s_state.junction, &junction_config)) return E_PAR;
		s_state.mm_per_count = config->junction_mm_per_count;
	}
	task.task = (FP)line_barcode_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = LINE_BARCODE_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
	/* Core 1 (TP_PRC2): core 0 runs the radio (priority 4) and USB console
	 * (3), which pre-empt any task there for milliseconds at a time. */
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC2;
#endif
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
	/* A barcode edge between odometry updates is placed at the estimated
	 * count (it used to fault). */
	readings.barcode.black = true;
	ok = process_readings(&s_test_state, &readings, 111U) == 0U &&
	     s_test_state.scanner.last_black && ok;
	ok = tk_rcv_mbf(g_mbf_line_observations, &line, TMO_POL) == (INT)sizeof(line) && ok;

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

/* Count at a sample between odometry updates: extrapolated at the last rate,
 * capped at the odometry age limit, never before the last odometry. */
static BOOL task_estimate_test(void)
{
	s_test_state = (line_state_t){0};
	s_test_state.have_odom = true;
	s_test_state.mean_count = 1000U;
	s_test_state.count_rate = 2.0f;
	s_test_state.last_odom_ms = 100U;
	s_test_state.max_age_ms = 50U;
	return sample_count(&s_test_state, 100U) == 1000 &&
	       sample_count(&s_test_state, 120U) == 1040 &&
	       sample_count(&s_test_state, 200U) == 1100 &&
	       sample_count(&s_test_state, 90U) == 1000;
}

static BOOL test_sample_line(line_state_t *state, const ir_readings_t *readings,
			     uint32_t count, uint32_t sequence, uint32_t timestamp,
			     line_observation_message_t *line)
{
	wheel_odometry_message_t odometry = {0};

	odometry.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	odometry.header.version = VEHICLE_MESSAGE_VERSION;
	odometry.header.sequence = sequence;
	odometry.header.timestamp_ms = timestamp;
	odometry.left_count = signed_bits(count);
	odometry.right_count = signed_bits(count);
	odometry.valid = 1U;
	return tk_snd_mbf(g_mbf_line_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK &&
	       process_readings(state, readings, timestamp) == 0U &&
	       tk_rcv_mbf(g_mbf_line_observations, line, TMO_POL) == (INT)sizeof(*line);
}

/* With the detector on (1 mm per count), a tape across the path -- centre
 * and right black together 28-38 mm -- sets intersection_detected in exactly
 * one observation; one line sensor at a time (straddle corrections) never
 * does, and those observations carry the side. */
static BOOL task_junction_test(void)
{
	static const straddle_junction_config_t config = STRADDLE_JUNCTION_DEFAULT_CONFIG;
	ir_readings_t readings = {0};
	line_observation_message_t line;
	unsigned junctions = 0U, centred = 0U, d;
	uint32_t sequence = 1U, timestamp = 100U;
	BOOL ok = TRUE;

	s_test_state = (line_state_t){0};
	s_test_state.barcode_config = (barcode_scanner_config_t){10U, 0U, 0U, 0U, 0U};
	s_test_state.max_age_ms = 50U;
	s_test_state.mm_per_count = 1.0f;
	if (!straddle_junction_init(&s_test_state.junction, &config)) return FALSE;
	for (d = 0U; d <= 80U; d += 2U) {
		readings.center.black = d >= 28U && d < 39U;
		readings.right.black = d >= 28U && d < 39U;
		if (!test_sample_line(&s_test_state, &readings, d, sequence++, timestamp, &line))
			ok = FALSE;
		timestamp += 20U;
		if (line.intersection_detected != 0U) junctions++;
	}
	ok = junctions == 1U && ok;
	/* Straddling: one line sensor at a time, then the tape in the gap. */
	for (; d <= 160U; d += 2U) {
		readings.center.black = (d / 10U) % 3U == 0U;
		readings.right.black = (d / 10U) % 3U == 1U;
		if (!test_sample_line(&s_test_state, &readings, d, sequence++, timestamp, &line))
			ok = FALSE;
		timestamp += 20U;
		if (line.intersection_detected != 0U) junctions++;
		if (line.lateral_error != 0 && line.line_detected) centred++;
	}
	ok = junctions == 1U && centred > 0U && ok;
	ok = test_drain(g_mbf_line_odometry) && ok;
	ok = test_drain(g_mbf_line_observations) && ok;
	return test_drain(g_mbf_barcode_events) && ok;
}

/* All three sensors black for LINE_PICKUP_MS reports picked_up; one sample
 * with floor under any sensor clears it and restarts the wait. */
static BOOL task_pickup_test(void)
{
	ir_readings_t readings = {0};
	line_observation_message_t line;
	uint32_t timestamp = 1000U;
	uint32_t sequence = 1U;
	uint32_t t;
	BOOL ok = TRUE;

	s_test_state = (line_state_t){0};
	s_test_state.barcode_config = (barcode_scanner_config_t){10U, 0U, 0U, 0U, 0U};
	s_test_state.max_age_ms = 50U;
	readings.barcode.black = readings.center.black = readings.right.black = true;
	for (t = 0U; t <= LINE_PICKUP_MS + 40U; t += 20U) {
		if (!test_sample_line(&s_test_state, &readings, 0U, sequence++, timestamp + t, &line))
			ok = FALSE;
		if (line.picked_up != (t >= LINE_PICKUP_MS ? 1U : 0U)) ok = FALSE;
	}
	timestamp += t;
	readings.right.black = false;
	ok = test_sample_line(&s_test_state, &readings, 0U, sequence++, timestamp, &line) &&
	     line.picked_up == 0U && ok;
	readings.right.black = true;
	ok = test_sample_line(&s_test_state, &readings, 0U, sequence++, timestamp + 20U, &line) &&
	     line.picked_up == 0U && ok;
	ok = test_drain(g_mbf_line_odometry) && ok;
	ok = test_drain(g_mbf_line_observations) && ok;
	return test_drain(g_mbf_barcode_events) && ok;
}

/* Hysteresis holds the colour between the thresholds; the debounce accepts a
 * change only after debounce_counts of travel and places the edge where it
 * began; a blip shorter than that is ignored. */
static BOOL task_barcode_filter_test(void)
{
	line_barcode_task_config_t cfg = {0};
	ir_readings_t r = {0};
	line_state_t st = {0};
	int32_t edge;
	BOOL ok;

	cfg.barcode_black_adc = 1000U;
	cfg.barcode_white_adc = 600U;
	r.barcode.raw = 1100U;
	ok = barcode_colour(&cfg, false, &r);
	r.barcode.raw = 800U;
	ok = barcode_colour(&cfg, true, &r) && !barcode_colour(&cfg, false, &r) && ok;
	r.barcode.raw = 500U;
	ok = !barcode_colour(&cfg, true, &r) && ok;
	cfg.barcode_black_adc = 0U;     /* off: the reader's colour */
	r.barcode.black = true;
	ok = barcode_colour(&cfg, false, &r) && ok;

	st.bc_deb_black = false;
	ok = !barcode_debounce(&st, 4U, true, 102, &edge) && edge == 102 && ok;
	ok = !barcode_debounce(&st, 4U, true, 105, &edge) && ok;        /* 3 counts */
	ok = barcode_debounce(&st, 4U, true, 106, &edge) && edge == 102 && ok;
	ok = barcode_debounce(&st, 4U, false, 110, &edge) && ok;        /* blip starts */
	ok = barcode_debounce(&st, 4U, true, 112, &edge) && !st.bc_deb_pending && ok;
	ok = barcode_debounce(&st, 4U, false, 120, &edge) && ok;
	ok = !barcode_debounce(&st, 4U, false, 124, &edge) && edge == 120 && ok;
	ok = barcode_debounce(&st, 0U, true, 130, &edge) && edge == 130 && ok;  /* off */

	/* Line sensors: black at >= 1500, white again only at <= 900. */
	cfg.line_black_adc = 1500U;
	cfg.line_white_adc = 900U;
	st = (line_state_t){0};
	{
		static const uint16_t raw[] = { 200U, 1250U, 1600U, 1250U, 950U, 880U, 1250U };
		static const bool want[] = { false, false, true, true, true, false, false };

		for (unsigned i = 0U; i < sizeof(raw) / sizeof(raw[0]); i++) {
			r = (ir_readings_t){0};
			r.center.raw = raw[i];
			r.right.raw = 200U;
			r.right.black = true;           /* the reader's flag is replaced */
			line_colours(&cfg, &st, &r);
			ok = r.center.black == want[i] && !r.right.black && ok;
		}
	}
	cfg.line_black_adc = 0U;        /* off: the reader's colours */
	r.right.black = true;
	line_colours(&cfg, &st, &r);
	ok = r.right.black && ok;
	return ok;
}

static unsigned s_test_started;

static void test_barcode_started(void *context)
{
	(void)context;
	s_test_started++;
}

/* The barcode_started hook runs once per frame (at the start '*'). */
static BOOL task_started_test(void)
{
	line_barcode_task_config_t saved = s_config;
	BOOL ok;

	s_test_started = 0U;
	s_config.barcode_started = test_barcode_started;
	s_config.barcode_started_context = NULL;
	ok = task_frame_test(1);
	s_config = saved;
	return ok && s_test_started == 1U;
}

BOOL line_barcode_task_test(void)
{
	BOOL pickup_ok;
	BOOL started_ok;
	BOOL filter_ok;
	BOOL frame_ok;
	BOOL mean_ok;
	BOOL fault_ok;
	BOOL estimate_ok;
	BOOL junction_ok;
	TEST_INIT();

	frame_ok = task_frame_test(1);
	frame_ok = task_frame_test(-1) && frame_ok;
	mean_ok = task_mean_test();
	fault_ok = task_fault_test();
	estimate_ok = task_estimate_test();
	junction_ok = task_junction_test();
	frame_ok = test_drain(g_mbf_line_odometry) && frame_ok;
	frame_ok = test_drain(g_mbf_line_observations) && frame_ok;
	frame_ok = test_drain(g_mbf_barcode_events) && frame_ok;
	TEST_CHECK("LB-U01", frame_ok, "simulated line and barcode messages");
	TEST_CHECK("LB-U02", mean_ok, "wheel-count and sequence wrap");
	TEST_CHECK("LB-U03", fault_ok, "odometry synchronization and full queues");
	TEST_CHECK("LB-U04", estimate_ok, "wheel count estimated between odometry updates");
	TEST_CHECK("LB-U05", junction_ok, "a tape across both line sensors flags one junction; one sensor at a time does not");
	pickup_ok = task_pickup_test();
	started_ok = task_started_test();
	TEST_CHECK("LB-U06", pickup_ok, "all three sensors black for LINE_PICKUP_MS reports picked_up; floor clears it");
	TEST_CHECK("LB-U07", started_ok, "barcode_started hook runs once per barcode frame");
	filter_ok = task_barcode_filter_test();
	TEST_CHECK("LB-U08", filter_ok, "barcode hysteresis and debounce as the bench barcode test; line sensor hysteresis");
	TEST_SUMMARY("line/barcode task");
	return frame_ok && mean_ok && fault_ok && estimate_ok && junction_ok &&
	       pickup_ok && started_ok && filter_ok;
}
#endif
