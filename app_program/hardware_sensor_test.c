#include "app.h"
#include "hardware_sensor_test.h"

#include "subsystem6/gpio/gpio_dispatcher.h"
#include "subsystem6/odometry/encoder_source.h"
#include "subsystem4/imu/lsm303dlhc_i2c.h"
#include "subsystem3/line_barcode/line_adc_reader.h"
#include "subsystem5/obstacle/obs_pico_port.h"
#include "subsystem5/obstacle/obs_task.h"
#include "subsystem5/obstacle/obs_width_hwtest.h"
#include "subsystem2/motion/spin_heading_hwtest.h"
#include "subsystem2/motion/motor_moves_hwtest.h"
#include "subsystem2/motion/pid_tune_hwtest.h"
#include "subsystem2/motion/spin_width_hwtest.h"
#include "subsystem2/motion/diag_width_hwtest.h"
#include "subsystem2/controller/line_follow_hwtest.h"
#include "subsystem2/controller/junction_hwtest.h"
#include "subsystem1/wifi/wifi_link_hwtest.h"
#include "vehicle_buffers.h"
#include "vehicle_messages.h"

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 7
#include "subsystem6/odometry/odometry_task.h"
#elif APP_SENSOR_TEST == 10
#include "subsystem6/odometry/odometry_producer.h"
#include "subsystem4/imu/imu_processor_config.h"
#endif

#if APP_SENSOR_TEST == 2 || APP_SENSOR_TEST == 6 || APP_SENSOR_TEST == 10
#include "subsystem4/imu/imu_processor.h"
#endif

#if APP_SENSOR_TEST == 2
#include "../device/include/dev_i2c.h"
#include "../device/include/i2c_probe_trace.h"
#include <sys/sysdef.h>
#include <tk/syslib.h>
#endif

#if APP_SENSOR_TEST == 4
#include <sys/sysdef.h>
#include <tk/syslib.h>

#ifndef SERVO_TEST_US
#define SERVO_TEST_US 0
#endif

_Static_assert(SERVO_TEST_US == 0 ||
	       (SERVO_TEST_US >= 1000 && SERVO_TEST_US <= 2000),
	       "SERVO_TEST_US must be 0 or a pulse width from 1000 to 2000 us");
#endif

#if APP_SENSOR_TEST && APP_SENSOR_TEST != 5

#define SENSOR_TEST_PRIORITY 10
#define SENSOR_TEST_INTERVAL_MS 500U
#define SENSOR_TEST_ERROR_DELAY_MS 5000U

static void report_error_forever(const char *sensor, const char *operation, ER error)
{
	for (;;) {
		tm_printf((UB*)"[HWTEST][%s] %s failed: %d\n",
			  sensor, operation, (int)error);
		tk_dly_tsk(SENSOR_TEST_ERROR_DELAY_MS);
	}
}

#if APP_SENSOR_TEST == 8 || APP_SENSOR_TEST == 9
static void init_bench_reader(line_adc_reader_t *reader, const char *name)
{
	line_adc_config_t config;
	ER error = line_adc_reader_default_config(&config, 100U);

	if (error == E_OK) error = line_adc_reader_init(reader, &config);
	if (error != E_OK) report_error_forever(name, "ADC init", error);
}
#endif

#if APP_SENSOR_TEST == 1
static void sensor_test_task(INT stacd, void *exinf)
{
	line_adc_reader_t reader = {0};
	line_adc_config_t config;
	ER error;

	(void)stacd;
	(void)exinf;
	error = line_adc_reader_default_config(&config, 100U);
	if (error != E_OK) report_error_forever("IR", "default config", error);
	error = line_adc_reader_init(&reader, &config);
	if (error != E_OK) report_error_forever("IR", "reader init", error);

	tm_printf((UB*)"[HWTEST][IR] live ADC readings; GP26-28\n");
	for (;;) {
		ir_readings_t readings;
		uint32_t timestamp_ms;

		error = line_adc_reader_read(&reader, &readings, &timestamp_ms);
		if (error != E_OK) report_error_forever("IR", "read", error);
		tm_printf((UB*)"[HWTEST][IR] t=%u ms barcode=%u black=%u "
			  "center=%u black=%u right=%u black=%u\n",
			  (unsigned)timestamp_ms,
			  (unsigned)readings.barcode.raw, readings.barcode.black ? 1U : 0U,
			  (unsigned)readings.center.raw, readings.center.black ? 1U : 0U,
			  (unsigned)readings.right.raw, readings.right.black ? 1U : 0U);
		error = tk_dly_tsk(SENSOR_TEST_INTERVAL_MS);
		if (error != E_OK) report_error_forever("IR", "delay", error);
	}
}
#elif APP_SENSOR_TEST == 7
#define LINE_BARCODE_TEST_ODOMETRY_PERIOD_MS 10U
#define LINE_BARCODE_TEST_PERIOD_MS 20U
#define LINE_BARCODE_TEST_MAX_SAMPLE_SPAN_MS 10U
#define LINE_BARCODE_TEST_REPORT_INTERVAL_MS 250U
#define LINE_BARCODE_TEST_COUNTS_PER_MODULE 250U

static const odometry_producer_config_t s_line_barcode_odometry_config = {
	{2500U, 20.4f}, {2500U, 20.4f}, -1, 1
};

static const line_barcode_task_config_t s_line_barcode_task_config = {
	LINE_BARCODE_TEST_PERIOD_MS,
	200U,
	{LINE_BARCODE_TEST_COUNTS_PER_MODULE, 25U, 0U, 0U, 0U},
	NULL, NULL, NULL,
	/* Junction detection on, as in the vehicle: push the robot over a tape
	 * laid across the line and a JUNCTION observation is reported. */
	204.0f / 2500.0f
};

static const char *line_position_name(const line_observation_message_t *line)
{
	if (!line->line_detected) return "LOST";
	if (line->intersection_detected) return "JUNCTION";
	if (line->lateral_error < 0) return "LEFT";
	if (line->lateral_error > 0) return "RIGHT";
	return "CENTER";
}

static void drain_unused_imu_odometry(void)
{
	wheel_odometry_message_t message;
	INT received;

	do {
		received = tk_rcv_mbf(g_mbf_imu_odometry, &message, TMO_POL);
	} while (received == (INT)sizeof(message));
	if (received != E_TMOUT)
		report_error_forever("LINE-BARCODE", "unused IMU odometry receive",
				     received < E_OK ? (ER)received : E_IO);
}

static void report_line_observations(bool *have_last, const char **last_position,
				     uint32_t *last_report_ms)
{
	line_observation_message_t line;
	INT received;

	for (;;) {
		const char *position;
		received = tk_rcv_mbf(g_mbf_line_observations, &line, TMO_POL);
		if (received == E_TMOUT) return;
		if (received != (INT)sizeof(line))
			report_error_forever("LINE-BARCODE", "line observation receive",
					     received < E_OK ? (ER)received : E_IO);
		if (line.header.type != VEHICLE_MESSAGE_LINE_OBSERVATION ||
		    line.header.version != VEHICLE_MESSAGE_VERSION)
			report_error_forever("LINE-BARCODE", "invalid line observation", E_SYS);

		position = line_position_name(&line);
		if (!*have_last || position != *last_position ||
		    (uint32_t)(line.header.timestamp_ms - *last_report_ms) >=
		    LINE_BARCODE_TEST_REPORT_INTERVAL_MS) {
			tm_printf((UB*)"[HWTEST][LINE-H01] t=%u ms line=%s "
				  "lateral=%d intersection=%u\n",
				  (unsigned)line.header.timestamp_ms, position,
				  (int)line.lateral_error,
				  (unsigned)line.intersection_detected);
			*last_position = position;
			*last_report_ms = line.header.timestamp_ms;
			*have_last = true;
		}
	}
}

static void report_barcode_events(void)
{
	barcode_event_message_t event;
	INT received;

	for (;;) {
		char payload[VEHICLE_BARCODE_PAYLOAD_SIZE + 1U];
		unsigned i;

		received = tk_rcv_mbf(g_mbf_barcode_events, &event, TMO_POL);
		if (received == E_TMOUT) return;
		if (received != (INT)sizeof(event))
			report_error_forever("LINE-BARCODE", "barcode event receive",
					     received < E_OK ? (ER)received : E_IO);
		if (event.header.type != VEHICLE_MESSAGE_BARCODE_EVENT ||
		    event.header.version != VEHICLE_MESSAGE_VERSION ||
		    event.valid != 1U || event.length > VEHICLE_BARCODE_PAYLOAD_SIZE)
			report_error_forever("LINE-BARCODE", "invalid barcode event", E_SYS);

		for (i = 0U; i < event.length; i++) payload[i] = event.payload[i];
		payload[event.length] = '\0';
		tm_printf((UB*)"[HWTEST][BAR-H01] t=%u ms decoded=%s length=%u\n",
			  (unsigned)event.header.timestamp_ms, payload,
			  (unsigned)event.length);
	}
}

static void report_sensor_faults(void)
{
	fault_message_t fault;
	INT received;

	for (;;) {
		received = tk_rcv_mbf(g_mbf_faults, &fault, TMO_POL);
		if (received == E_TMOUT) return;
		if (received != (INT)sizeof(fault))
			report_error_forever("LINE-BARCODE", "fault receive",
					     received < E_OK ? (ER)received : E_IO);
		tm_printf((UB*)"[HWTEST][LINE-BARCODE] fault source=%u code=%u\n",
			  (unsigned)fault.source, (unsigned)fault.code);
	}
}

static void sensor_test_task(INT stacd, void *exinf)
{
	bool have_last_line = false;
	const char *last_position = NULL;
	uint32_t last_report_ms = 0U;
	ER error;

	(void)stacd;
	(void)exinf;
	error = encoder_source_register();
	if (error != E_OK)
		report_error_forever("LINE-BARCODE", "encoder registration", error);
	error = gpio_dispatcher_start();
	if (error != E_OK)
		report_error_forever("LINE-BARCODE", "GPIO dispatcher start", error);
	error = odometry_task_start(&s_line_barcode_odometry_config,
				    LINE_BARCODE_TEST_ODOMETRY_PERIOD_MS);
	if (error != E_OK)
		report_error_forever("LINE-BARCODE", "odometry task start", error);
	error = line_adc_reader_start_default_task(LINE_BARCODE_TEST_MAX_SAMPLE_SPAN_MS,
						   &s_line_barcode_task_config);
	if (error != E_OK)
		report_error_forever("LINE-BARCODE", "line/barcode task start", error);

	tm_printf((UB*)"[HWTEST][LINE-BARCODE] monitor started; motors and servo "
		  "are not started; keep wheels still until the first line sample\n");
	tm_printf((UB*)"[HWTEST][LINE-BARCODE] wheel_counts_per_rev=2500 "
		  "circumference_cm=20.4 counts_per_module=%u\n",
		  (unsigned)LINE_BARCODE_TEST_COUNTS_PER_MODULE);
	tm_printf((UB*)"[HWTEST][LINE-H01] move the sensors across line, junction, "
		  "and blank track; observe LOST/LEFT/RIGHT/JUNCTION\n");
	tm_printf((UB*)"[HWTEST][BAR-H01] roll over a known Code 39 label in both "
		  "directions; expected payload is printed below\n");

	for (;;) {
		report_line_observations(&have_last_line, &last_position, &last_report_ms);
		report_barcode_events();
		report_sensor_faults();
		drain_unused_imu_odometry();
		error = tk_dly_tsk(10U);
		if (error != E_OK)
			report_error_forever("LINE-BARCODE", "diagnostic delay", error);
	}
}
#elif APP_SENSOR_TEST == 8
static void sensor_test_task(INT stacd, void *exinf)
{
	static const char *const names[] = {
		[POS_LOST] = "LOST", [POS_LEFT] = "LEFT",
		[POS_RIGHT] = "RIGHT", [POS_JUNCTION] = "JUNCTION"
	};
	line_adc_reader_t reader = {0};
	ER error;

	(void)stacd;
	(void)exinf;
	init_bench_reader(&reader, "LINE-H02");
	tm_printf((UB*)"[HWTEST][LINE-H02] GP27 center, GP26 right; no encoders or motors\n");
	for (;;) {
		ir_readings_t readings;
		uint32_t timestamp;

		error = line_adc_reader_read_line(&reader, &readings, &timestamp);
		if (error != E_OK) report_error_forever("LINE-H02", "ADC read", error);
		tm_printf((UB*)"[HWTEST][LINE-H02] t=%u line=%s center=%u right=%u\n",
			  (unsigned)timestamp, names[line_following_position(&readings)],
			  (unsigned)readings.center.raw, (unsigned)readings.right.raw);
		error = tk_dly_tsk(250U);
		if (error != E_OK) report_error_forever("LINE-H02", "delay", error);
	}
}
#elif APP_SENSOR_TEST == 9
_Static_assert(BARCODE_TEST_MODULE_COUNTS >= 0 &&
	       BARCODE_TEST_MODULE_COUNTS <= UINT32_MAX,
	       "BARCODE_TEST_MODULE_COUNTS must be 0 or a positive 32-bit count");
_Static_assert(BARCODE_TEST_WHITE_ADC >= 0 &&
	       BARCODE_TEST_WHITE_ADC < BARCODE_TEST_BLACK_ADC &&
	       BARCODE_TEST_BLACK_ADC <= 4095,
	       "barcode ADC thresholds need 0 <= WHITE < BLACK <= 4095");
_Static_assert(BARCODE_TEST_DEBOUNCE_COUNTS >= 0,
	       "BARCODE_TEST_DEBOUNCE_COUNTS must not be negative");

/* Hysteresis: the colour only changes once the reading clearly passes the
 * opposite threshold, so noise on a slow bar edge cannot add extra runs. */
static bool bench_barcode_black(uint16_t raw, bool previous)
{
	if (raw >= BARCODE_TEST_BLACK_ADC) return true;
	if (raw <= BARCODE_TEST_WHITE_ADC) return false;
	return previous;
}

static uint16_t s_bench_adc_min = UINT16_MAX;
static uint16_t s_bench_adc_max;

/* A colour change is accepted only after the encoders have moved
 * BARCODE_TEST_DEBOUNCE_COUNTS with the new colour held. The edge is placed
 * where the change began, so accepted widths are not shifted. */
static bool s_debounce_black;
static bool s_debounce_pending;
static int32_t s_debounce_start;

static bool bench_debounce(bool black, int32_t count, int32_t *edge_count)
{
	*edge_count = count;
	if (black == s_debounce_black) {
		s_debounce_pending = false;
		return s_debounce_black;
	}
	if (!s_debounce_pending) {
		s_debounce_pending = true;
		s_debounce_start = count;
	}
	{
		int32_t moved = (int32_t)((uint32_t)count - (uint32_t)s_debounce_start);
		uint32_t distance = (moved < 0) ? 0U - (uint32_t)moved : (uint32_t)moved;

		if (distance < (uint32_t)BARCODE_TEST_DEBOUNCE_COUNTS) return s_debounce_black;
	}
	s_debounce_black = black;
	s_debounce_pending = false;
	*edge_count = s_debounce_start;
	return s_debounce_black;
}

/* Calibration mode allows short runs before the module size is known. */
static const barcode_scanner_config_t s_bench_barcode_config = {
	BARCODE_TEST_MODULE_COUNTS ? BARCODE_TEST_MODULE_COUNTS : 1U,
	25U, 0U, BARCODE_TEST_MODULE_COUNTS ? 0U : 1000U, 0U
};
static barcode_scanner_t s_bench_scanner;

static void report_bench_barcode(void)
{
	barcode_char_info_t info;
	uint8_t payload[BARCODE_MAX_PAYLOAD + 1U];
	uint8_t length;

	if (barcode_scanner_take_event(&s_bench_scanner, &info)) {
		if (info.reject == BARCODE_REJECT_NONE && info.symbol == '*' && info.in_frame)
			barcode_scanner_reset_stats(&s_bench_scanner);
		tm_printf((UB*)"[HWTEST][BAR-H02] char=%c %s dir=%d label=%s narrow_max=%u "
			  "wide_min=%u gap=%u offending=%u\n",
			  info.symbol ? info.symbol : '?', barcode_reject_name(info.reject),
			  (int)info.direction, info.label_reversed ? "reversed" : "normal",
			  (unsigned)info.narrow_max, (unsigned)info.wide_min,
			  (unsigned)info.gap_counts, (unsigned)info.offending_counts);
		if (info.run_count == BARCODE_ELEMENTS)
			tm_printf((UB*)"[HWTEST][BAR-H02]   pattern=%s widths=%u %u %u %u %u %u %u %u %u "
				  "adc_min=%u adc_max=%u\n",
				  info.raw_pattern[0] ? info.raw_pattern : "?",
				  (unsigned)info.run_counts[0], (unsigned)info.run_counts[1],
				  (unsigned)info.run_counts[2], (unsigned)info.run_counts[3],
				  (unsigned)info.run_counts[4], (unsigned)info.run_counts[5],
				  (unsigned)info.run_counts[6], (unsigned)info.run_counts[7],
				  (unsigned)info.run_counts[8],
				  (unsigned)s_bench_adc_min, (unsigned)s_bench_adc_max);
		s_bench_adc_min = UINT16_MAX;
		s_bench_adc_max = 0U;
	}
	if (barcode_scanner_take_result(&s_bench_scanner, payload, BARCODE_MAX_PAYLOAD, &length)) {
		barcode_scanner_config_t suggested;

		payload[length] = '\0';
		tm_printf((UB*)"[HWTEST][BAR-H02] decoded=%s length=%u\n", payload, (unsigned)length);
		if (barcode_scanner_suggest_config(&s_bench_scanner, &suggested))
			tm_printf((UB*)"[HWTEST][BAR-H02] if payload matches, try "
				  "BARCODE_TEST_MODULE_COUNTS=%u\n",
				  (unsigned)suggested.counts_per_module);
	}
}

/*
 * Whole-pass recorder: every bar and space of one pass over a barcode, then
 * a report of why it did or did not decode. A pass starts at the first bar
 * and ends after PASS_END_MM of white, or on white once the wheels have not
 * moved for PASS_IDLE_MS (a slow push over a wide space must not end it).
 *
 * For each run it keeps the width and the extreme ADC value (a bar's darkest,
 * a space's lightest), plus "dips": inside a bar, the reading fell by
 * >= PASS_DIP_ADC and rose again without reaching white -- a narrow space
 * the thresholds did not see, which merges two bars into one (and the same
 * for a narrow bar hidden in a space). BARCODE_TEST_EXPECT="Y" (the payload
 * without the '*'s) compares the runs with the pattern it should give.
 */
#ifndef BARCODE_TEST_EXPECT
#define BARCODE_TEST_EXPECT ""
#endif
/* Longest frame: '*' + 32 payload characters + '*', 9 runs each plus gaps. */
#define PASS_MAX_RUNS     ((BARCODE_MAX_PAYLOAD + 2U) * 10U)
#define PASS_END_MM       40.0f
#define PASS_IDLE_MS      3000U
#define PASS_MOVE_COUNTS  12U     /* ~1 mm: the wheels are still moving */
#define PASS_DIP_ADC      250U
#define PASS_MM_PER_COUNT (204.0f / 2500.0f)    /* = app_main.c calibration */
#define PASS_MIN_RATIO    1.3f                  /* = the scanner's wide/narrow split */

typedef struct {
	uint32_t counts;
	uint16_t extreme;       /* bar: max ADC; space: min ADC */
	uint16_t dip;           /* deepest hidden dip (bar: lowest; space: highest) */
	uint8_t dips;
	bool black;
} pass_run_t;

static pass_run_t s_runs[PASS_MAX_RUNS];
static unsigned s_run_n;
static bool s_pass_active, s_pass_overflow, s_pass_colour;
static int32_t s_pass_start_count, s_run_start;
static uint32_t s_last_move_ms;
static int32_t s_last_move_count;
static unsigned s_pass_no;
static uint32_t s_pass_start_ms, s_pass_bar_end_ms;     /* first bar edge, last bar end */
/* While driving, the report waits until the robot has stopped (printing
 * takes long enough to upset the speed control). */
static bool s_report_pending;
static int32_t s_report_count;
/* The current run, in "darkness" units: raw for a bar, 4095 - raw for a
 * space, so one dip detector serves both. */
static uint16_t s_run_ext, s_run_peak, s_run_trough, s_run_dip;
static uint8_t s_run_dips;

static uint32_t abs_delta(int32_t from, int32_t to)
{
	int32_t d = (int32_t)((uint32_t)to - (uint32_t)from);
	return d < 0 ? 0U - (uint32_t)d : (uint32_t)d;
}

/* "12.3" for a width in counts (tm_printf has no %f). */
static void print_mm(char *buf, float mm)
{
	unsigned t = (unsigned)(mm * 10.0f + 0.5f);
	buf[0] = '\0';
	(void)tm_sprintf((UB*)buf, (UB*)"%u.%u", t / 10U, t % 10U);
}

static void run_begin(bool black, int32_t at, uint16_t raw)
{
	uint16_t v = black ? raw : (uint16_t)(4095U - raw);

	s_pass_colour = black;
	s_run_start = at;
	s_run_ext = s_run_peak = s_run_trough = v;
	s_run_dip = 4095U;
	s_run_dips = 0U;
}

static void run_end(int32_t at)
{
	pass_run_t *r;

	if (s_run_n >= PASS_MAX_RUNS) {
		s_pass_overflow = true;
		return;
	}
	r = &s_runs[s_run_n++];
	r->black = s_pass_colour;
	r->counts = abs_delta(s_run_start, at);
	r->extreme = s_pass_colour ? s_run_ext : (uint16_t)(4095U - s_run_ext);
	r->dips = s_run_dips;
	r->dip = s_run_dips == 0U ? 0U :
		 (s_pass_colour ? s_run_dip : (uint16_t)(4095U - s_run_dip));
}

static void run_sample(uint16_t raw)
{
	uint16_t v = s_pass_colour ? raw : (uint16_t)(4095U - raw);

	if (v > s_run_ext) s_run_ext = v;
	if (v > s_run_peak) {
		s_run_peak = s_run_trough = v;
	} else if (v < s_run_trough) {
		s_run_trough = v;
	} else if ((unsigned)(s_run_peak - s_run_trough) >= PASS_DIP_ADC &&
		   (unsigned)v >= s_run_trough + PASS_DIP_ADC) {
		if (s_run_trough < s_run_dip) s_run_dip = s_run_trough;
		if (s_run_dips < 255U) s_run_dips++;
		s_run_peak = s_run_trough = v;
	}
}

/* Two-group split of one colour's widths (iterated midpoint). */
static uint32_t split_threshold(bool black)
{
	uint32_t lo = UINT32_MAX, hi = 0U, t;
	unsigned i, k;

	for (i = 0U; i < s_run_n; i++) {
		if (s_runs[i].black != black) continue;
		if (s_runs[i].counts < lo) lo = s_runs[i].counts;
		if (s_runs[i].counts > hi) hi = s_runs[i].counts;
	}
	if (hi == 0U) return 0U;
	t = (lo + hi) / 2U;
	for (k = 0U; k < 8U; k++) {
		uint64_t sn = 0U, sw = 0U;
		unsigned nn = 0U, nw = 0U;
		for (i = 0U; i < s_run_n; i++) {
			if (s_runs[i].black != black) continue;
			if (s_runs[i].counts < t) { sn += s_runs[i].counts; nn++; }
			else { sw += s_runs[i].counts; nw++; }
		}
		if (nn == 0U || nw == 0U) break;
		t = (uint32_t)((sn / nn + sw / nw) / 2U);
	}
	return t;
}

typedef struct {
	float mean;
	uint32_t min, max;
	unsigned n;
} group_t;

static group_t width_group(bool black, bool wide, uint32_t t)
{
	group_t g = {0.0f, UINT32_MAX, 0U, 0U};
	uint64_t sum = 0U;
	unsigned i;

	for (i = 0U; i < s_run_n; i++) {
		const pass_run_t *r = &s_runs[i];
		if (r->black != black || (r->counts >= t) != wide) continue;
		sum += r->counts;
		if (r->counts < g.min) g.min = r->counts;
		if (r->counts > g.max) g.max = r->counts;
		g.n++;
	}
	if (g.n) g.mean = (float)sum / (float)g.n;
	else g.min = 0U;
	return g;
}

/* Expected n/w per run for "*EXPECT*": 9 per character plus a narrow gap
 * between characters. Returns the run count, 0 if a symbol is unknown. */
static unsigned expected_runs(char *out, unsigned capacity)
{
	const char *payload = BARCODE_TEST_EXPECT;
	unsigned chars = 0U, n = 0U, i, k;

	while (payload[chars] != '\0') chars++;
	if (chars == 0U) return 0U;
	for (i = 0U; i < chars + 2U; i++) {
		char c = (i == 0U || i == chars + 1U) ? '*' : payload[i - 1U];
		const char *p;

		if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
		p = barcode_code39_pattern(c);
		if (p == NULL || n + 10U > capacity) return 0U;
		if (i > 0U) out[n++] = 'n';
		for (k = 0U; k < BARCODE_ELEMENTS; k++) out[n++] = p[k];
	}
	return n;
}

static void pass_report(int32_t end_count)
{
	static char expect[PASS_MAX_RUNS + 1U];
	const uint32_t tb = split_threshold(true), ts = split_threshold(false);
	const unsigned n_expect = expected_runs(expect, PASS_MAX_RUNS);
	group_t bn = width_group(true, false, tb), bw = width_group(true, true, tb);
	group_t sn = width_group(false, false, ts), sw = width_group(false, true, ts);
	uint16_t weakest_bar = 4095U, palest_space = 0U, hidden_space = 0U, hidden_bar = 4095U;
	unsigned hidden_spaces = 0U, hidden_bars = 0U, mismatches = 0U, i;
	bool reversed = false, compare = n_expect != 0U && n_expect == s_run_n;
	uint32_t total = 0U, pass_ms = s_pass_bar_end_ms - s_pass_start_ms;
	char b1[16], b2[40], b3[16], b4[16];

	for (i = 0U; i < s_run_n; i++) total += s_runs[i].counts;
	/* Pick the orientation that matches better (label or push reversed). */
	if (compare) {
		unsigned fwd = 0U, rev = 0U;
		for (i = 0U; i < s_run_n; i++) {
			char c = s_runs[i].counts >= (s_runs[i].black ? tb : ts) ? 'w' : 'n';
			if (c != expect[i]) fwd++;
			if (c != expect[s_run_n - 1U - i]) rev++;
		}
		reversed = rev < fwd;
		mismatches = reversed ? rev : fwd;
	}

	s_pass_no++;
	print_mm(b1, (float)total * PASS_MM_PER_COUNT);
	/* Speed over the label: its length / the time from first to last bar. */
	print_mm(b3, pass_ms ? (float)total * PASS_MM_PER_COUNT / (float)pass_ms * 100.0f : 0.0f);
	tm_printf((UB*)"[HWTEST][BAR-H02] ==== PASS %u: %u runs over %s mm at %s cm/s, moving %s%s "
		  "====\n", s_pass_no, s_run_n, b1, b3,
		  (int32_t)((uint32_t)end_count - (uint32_t)s_pass_start_count) >= 0 ?
		  "forward" : "BACKWARD", s_pass_overflow ? " (TOO MANY RUNS, rest dropped)" : "");
	if (n_expect != 0U)
		tm_printf((UB*)"[HWTEST][BAR-H02]   expected *%s* = %u runs: %s%s\n",
			  BARCODE_TEST_EXPECT, n_expect,
			  n_expect == s_run_n ? "count matches" :
			  (s_run_n < n_expect ? "MISSING runs" : "EXTRA runs"),
			  compare && reversed ? " (read as reversed)" : "");
	tm_printf((UB*)"[HWTEST][BAR-H02]    #  kind  counts    mm   adc  class%s  hidden\n",
		  compare ? " expect" : "");
	for (i = 0U; i < s_run_n; i++) {
		const pass_run_t *r = &s_runs[i];
		char c = r->counts >= (r->black ? tb : ts) ? 'w' : 'n';
		char e = compare ? expect[reversed ? s_run_n - 1U - i : i] : ' ';

		print_mm(b1, (float)r->counts * PASS_MM_PER_COUNT);
		b2[0] = '\0';
		if (r->dips)
			(void)tm_sprintf((UB*)b2, (UB*)"%s x%u adc %u", r->black ? "space?" : "bar?",
					 (unsigned)r->dips, (unsigned)r->dip);
		tm_printf((UB*)"[HWTEST][BAR-H02]   %2u  %s %6u %5s  %4u    %c   %c%s  %s\n",
			  i + 1U, r->black ? "BAR " : "spc ", (unsigned)r->counts, b1,
			  (unsigned)r->extreme, c, compare ? e : ' ',
			  compare && c != e ? " <--" : (compare ? "    " : ""), b2);
		if (r->black) {
			if (r->extreme < weakest_bar) weakest_bar = r->extreme;
			if (r->dips) {
				hidden_spaces += r->dips;
				if (r->dip > hidden_space) hidden_space = r->dip;
			}
		} else {
			if (r->extreme > palest_space) palest_space = r->extreme;
			if (r->dips) {
				hidden_bars += r->dips;
				if (r->dip < hidden_bar) hidden_bar = r->dip;
			}
		}
	}

	/* Widths: the scanner needs every wide >= 1.3 x every narrow. */
	print_mm(b1, bn.mean * PASS_MM_PER_COUNT);
	print_mm(b2, bw.mean * PASS_MM_PER_COUNT);
	print_mm(b3, sn.mean * PASS_MM_PER_COUNT);
	print_mm(b4, sw.mean * PASS_MM_PER_COUNT);
	tm_printf((UB*)"[HWTEST][BAR-H02]   widths: bars narrow %s mm (%u..%u counts) wide %s mm "
		  "(%u..%u); spaces narrow %s mm (%u..%u) wide %s mm (%u..%u)\n",
		  b1, (unsigned)bn.min, (unsigned)bn.max, b2, (unsigned)bw.min, (unsigned)bw.max,
		  b3, (unsigned)sn.min, (unsigned)sn.max, b4, (unsigned)sw.min, (unsigned)sw.max);
	tm_printf((UB*)"[HWTEST][BAR-H02]   separation (worst wide / worst narrow, need >= 1.3): "
		  "bars %u%%, spaces %u%%\n",
		  bn.max ? (unsigned)(100U * bw.min / bn.max) : 0U,
		  sn.max ? (unsigned)(100U * sw.min / sn.max) : 0U);
	if (bn.n && sn.n) {
		float bias = (sn.mean - bn.mean) * 0.5f * PASS_MM_PER_COUNT;
		print_mm(b1, bias < 0.0f ? -bias : bias);
		tm_printf((UB*)"[HWTEST][BAR-H02]   the sensor makes bars %s by ~%s mm each edge "
			  "(narrow bar vs narrow space)\n", bias >= 0.0f ? "THINNER" : "THICKER", b1);
	}
	tm_printf((UB*)"[HWTEST][BAR-H02]   levels: weakest bar peaked at %u, palest space reached "
		  "%u (thresholds black >= %u, white <= %u)\n", (unsigned)weakest_bar,
		  (unsigned)palest_space, (unsigned)BARCODE_TEST_BLACK_ADC,
		  (unsigned)BARCODE_TEST_WHITE_ADC);

	/* Diagnosis. */
	if (hidden_spaces)
		tm_printf((UB*)"[HWTEST][BAR-H02]   PROBLEM: %u hidden space(s): inside a bar the reading "
			  "dipped to as low as %u but not to white (<= %u), so two bars merged into "
			  "one wide 'bar' (2 runs lost each).\n", hidden_spaces,
			  (unsigned)hidden_space, (unsigned)BARCODE_TEST_WHITE_ADC);
	if (hidden_bars)
		tm_printf((UB*)"[HWTEST][BAR-H02]   PROBLEM: %u hidden bar(s): inside a space the reading "
			  "rose to %u but not to black (>= %u), so two spaces merged.\n", hidden_bars,
			  (unsigned)hidden_bar, (unsigned)BARCODE_TEST_BLACK_ADC);
	if (hidden_spaces || hidden_bars) {
		/* Thresholds between the shallowest dip and the weakest bar peak. */
		uint16_t space_top = hidden_space > palest_space ? hidden_space : palest_space;
		uint16_t bar_bottom = hidden_bars && hidden_bar < weakest_bar ? hidden_bar : weakest_bar;

		if (space_top + 2U * PASS_DIP_ADC / 5U < bar_bottom) {
			unsigned gap = bar_bottom - space_top;
			tm_printf((UB*)"[HWTEST][BAR-H02]   FIX: rebuild with BARCODE_TEST_WHITE_ADC=%u "
				  "BARCODE_TEST_BLACK_ADC=%u, and use the same in app_main.c line_config\n",
				  (unsigned)(space_top + gap / 3U),
				  (unsigned)(space_top + 2U * gap / 3U));
		} else {
			tm_printf((UB*)"[HWTEST][BAR-H02]   FIX: no threshold pair separates them: the IR "
				  "spot is wider than a narrow element. Mount the sensor lower/closer, "
				  "or print the barcode larger.\n");
		}
	}
	if (n_expect != 0U && s_run_n != n_expect)
		tm_printf((UB*)"[HWTEST][BAR-H02]   PROBLEM: %u runs, expected %u. %s\n", s_run_n,
			  n_expect, s_run_n < n_expect ?
			  "Fewer = elements merged (hidden spaces/bars above) or the pass started "
			  "or ended mid-barcode (start before and finish after the label)." :
			  "More = elements split by noise or a smudge: look for very short runs, "
			  "or raise BARCODE_TEST_DEBOUNCE_COUNTS.");
	else if (compare && mismatches)
		tm_printf((UB*)"[HWTEST][BAR-H02]   PROBLEM: run count right but %u element(s) "
			  "classified wrong (marked <--): widths overlap; see separation above.\n",
			  mismatches);
	else if (compare)
		tm_printf((UB*)"[HWTEST][BAR-H02]   every element matches *%s*: the barcode reads "
			  "cleanly at this push.\n", BARCODE_TEST_EXPECT);
	if ((bn.max && 100U * bw.min < 130U * bn.max) || (sn.max && 100U * sw.min < 130U * sn.max))
		tm_printf((UB*)"[HWTEST][BAR-H02]   PROBLEM: wide and narrow overlap (under %u%%): the "
			  "decoder cannot tell them apart%s.\n", 130U,
			  hidden_spaces || hidden_bars ? " -- merged elements look extra wide" : "");
}

/* Every sample: the debounced colour, the edge position the debounce gave
 * and the live count. */
static void pass_sample(uint16_t raw, bool black, int32_t edge_count, int32_t count,
			uint32_t now)
{
	if (!s_pass_active) {
		if (!black) return;
		s_pass_active = true;
		s_pass_overflow = false;
		s_run_n = 0U;
		s_pass_start_count = edge_count;
		s_pass_start_ms = s_pass_bar_end_ms = now;
		s_last_move_ms = now;
		s_last_move_count = count;
		run_begin(true, edge_count, raw);
		return;
	}
	if (black != s_pass_colour) {
		run_end(edge_count);
		run_begin(black, edge_count, raw);
		if (!black) s_pass_bar_end_ms = now;
	} else {
		run_sample(raw);
	}
	if (abs_delta(s_last_move_count, count) >= PASS_MOVE_COUNTS) {
		s_last_move_count = count;
		s_last_move_ms = now;
	}
	if (!s_pass_colour &&
	    ((float)abs_delta(s_run_start, count) * PASS_MM_PER_COUNT >= PASS_END_MM ||
	     now - s_last_move_ms >= PASS_IDLE_MS)) {
		s_pass_active = false;          /* the trailing white is the quiet zone */
		s_report_pending = true;        /* printed once the robot is not driving */
		s_report_count = count;
	}
}

/*
 * BARCODE_TEST_DRIVE=<cm/s>: instead of a hand push, GO (GP20) drives the
 * robot dead straight at that speed for BARCODE_TEST_DRIVE_MM -- the
 * vehicle's wheel PID (motion_config in app_main.c), no line steering -- so
 * runs at different speeds show whether the bar widths change with speed
 * (sensor response) or only go wrong while line following (path/steering).
 * GO again stops; so do a stall or a wheel turning the wrong way.
 */
/* BARCODE_TEST_DRIVE=vehicle (make) passes VEHICLE_LINE_SPEED_CM_S: the
 * main robot's line-following speed, whatever it is set to. */
#include "subsystem2/controller/vehicle_controller_task.h"
#ifndef BARCODE_TEST_DRIVE
#define BARCODE_TEST_DRIVE 0            /* cm/s; 0 = push by hand */
#endif
#ifndef BARCODE_TEST_DRIVE_MM
#define BARCODE_TEST_DRIVE_MM 300U
#endif
_Static_assert(BARCODE_TEST_DRIVE >= 0 && BARCODE_TEST_DRIVE <= 20,
	       "BARCODE_TEST_DRIVE is the speed in cm/s, 1..20 (0 = push by hand)");
_Static_assert(BARCODE_TEST_DRIVE_MM >= 50U && BARCODE_TEST_DRIVE_MM <= 2000U,
	       "need 50 <= BARCODE_TEST_DRIVE_MM <= 2000");

#include <sys/sysdef.h>
#include <tk/syslib.h>
#include "subsystem2/motion/motor_driver.h"
#include "subsystem2/motion/wheel_speed_control.h"

/* The kernel tick is 10 ms (CNF_TIMER_PERIOD), so tk_dly_tsk(2) really
 * waits 10-20 ms: busy-wait for 2 ms samples while it matters. */
#define BENCH_SAMPLE_US    2000U
#define BENCH_READ_US      200U         /* one ADC read + encoder snapshot, roughly */
#define DRIVE_GO_PIN       20U
#define DRIVE_START_MS     2000U
#define DRIVE_PID_MS       50U          /* = odometry period, as the motion task */
#define DRIVE_STALL_MS     1000U
#define DRIVE_STALL_COUNTS 25U          /* ~2 mm */
#define DRIVE_WRONG_COUNTS 245          /* ~20 mm backwards */
#define DRIVE_GO_STABLE    10U          /* samples (~20 ms) of a steady button */
#define DRIVE_LEFT_FWD     1            /* motion_task.c MOTION_LEFT_POLARITY */
#define DRIVE_RIGHT_FWD    (-1)         /* motion_task.c MOTION_RIGHT_POLARITY */

typedef enum { DRIVE_OFF = 0, DRIVE_WAIT_GO, DRIVE_DELAY, DRIVE_RUN } drive_state_t;

static struct {
	drive_state_t state;
	wheel_speed_control_t pid;
	wheel_speed_output_t out;
	bool go_pressed, go_raw_last;
	unsigned go_stable;
	uint32_t t0, next_pid, last_pid, progress_ms;
	int32_t start_count, progress_count;
	float pid_l, pid_r;     /* wheel mm at the last PID update */
} s_drive;

static void drive_wheels_mm(const encoder_snapshot_t *s, float *l, float *r)
{
	*l = (float)(-s->left_count) * PASS_MM_PER_COUNT;      /* left counts down forward */
	*r = (float)s->right_count * PASS_MM_PER_COUNT;
}

static void drive_pwm(uint8_t left, uint8_t right)
{
	/* E_BUSY = reversal brake interval; the next update retries. */
	(void)motor_set_speed(MOTOR_1, (int16_t)((int)left * DRIVE_LEFT_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)((int)right * DRIVE_RIGHT_FWD));
}

/* GO pressed (active low, debounced over DRIVE_GO_STABLE samples): edge. */
static bool drive_go_edge(void)
{
	bool raw = gpio_get_val(DRIVE_GO_PIN) == 0U;
	bool edge = false;

	if (raw != s_drive.go_raw_last) {
		s_drive.go_raw_last = raw;
		s_drive.go_stable = 0U;
	} else if (s_drive.go_stable < DRIVE_GO_STABLE) {
		if (++s_drive.go_stable == DRIVE_GO_STABLE && raw != s_drive.go_pressed) {
			s_drive.go_pressed = raw;
			edge = raw;
		}
	}
	return edge;
}

static void drive_init(void)
{
	const wheel_speed_config_t config = {
		2.0f, 4.0f, 0.0f, (float)BARCODE_TEST_DRIVE, 22U, 60U, 1.4f, 15.0f
	};
	ER error;

	error = gpio_set_pin(DRIVE_GO_PIN, GPIO_MODE_IN);
	if (error != E_OK) report_error_forever("BAR-H02", "GO pin", error);
	clr_w(GPIO(DRIVE_GO_PIN), GPIO_PDE);
	set_w(GPIO(DRIVE_GO_PIN), GPIO_PUE);
	error = motor_driver_init();
	if (error != E_OK) report_error_forever("BAR-H02", "motor driver init", error);
	if (!wheel_speed_control_init(&s_drive.pid, &config))
		report_error_forever("BAR-H02", "wheel PID config", E_PAR);
	s_drive.go_pressed = s_drive.go_raw_last = gpio_get_val(DRIVE_GO_PIN) == 0U;
	s_drive.state = DRIVE_WAIT_GO;
}

static void drive_stop(const char *why)
{
	(void)motor_brake_all();
	(void)motor_emergency_stop();
	s_drive.state = DRIVE_WAIT_GO;
	tm_printf((UB*)"[HWTEST][BAR-H02] drive ended: %s -- place the robot again and press GO\n",
		  why);
}

/* One step per sample: GO handling, safety checks and the wheel PID. */
static void drive_step(uint32_t now, const encoder_snapshot_t *snapshot, int32_t count)
{
	bool go = drive_go_edge();
	float l, r;

	switch (s_drive.state) {
	case DRIVE_WAIT_GO:
		if (go) {
			tm_printf((UB*)"[HWTEST][BAR-H02] driving %u cm/s for %u mm in %u s, hands clear\n",
				  (unsigned)BARCODE_TEST_DRIVE, (unsigned)BARCODE_TEST_DRIVE_MM,
				  (unsigned)(DRIVE_START_MS / 1000U));
			s_drive.state = DRIVE_DELAY;
			s_drive.t0 = now;
		}
		break;
	case DRIVE_DELAY:
		if (go) {
			s_drive.state = DRIVE_WAIT_GO;
			tm_printf((UB*)"[HWTEST][BAR-H02] start cancelled\n");
		} else if (now - s_drive.t0 >= DRIVE_START_MS) {
			ER error = motor_clear_emergency_stop();

			if (error != E_OK) report_error_forever("BAR-H02", "motor arm", error);
			if (!wheel_speed_control_reset(&s_drive.pid) ||
			    !wheel_speed_control_set_targets(&s_drive.pid, (float)BARCODE_TEST_DRIVE,
							     (float)BARCODE_TEST_DRIVE)) {
				drive_stop("wheel PID refused the target");
				break;
			}
			s_drive.out.left_pwm_percent = s_drive.out.right_pwm_percent = 22U;
			drive_wheels_mm(snapshot, &s_drive.pid_l, &s_drive.pid_r);
			s_drive.start_count = s_drive.progress_count = count;
			s_drive.last_pid = s_drive.progress_ms = now;
			s_drive.next_pid = now + DRIVE_PID_MS;
			s_drive.state = DRIVE_RUN;
			drive_pwm(s_drive.out.left_pwm_percent, s_drive.out.right_pwm_percent);
		}
		break;
	case DRIVE_RUN:
		if (go) { drive_stop("stopped by GO"); break; }
		if ((float)abs_delta(s_drive.start_count, count) * PASS_MM_PER_COUNT >=
		    (float)BARCODE_TEST_DRIVE_MM) {
			drive_stop("distance reached");
			break;
		}
		if ((int32_t)((uint32_t)count - (uint32_t)s_drive.start_count) < -DRIVE_WRONG_COUNTS) {
			drive_stop("moving BACKWARDS: motor or encoder sign");
			break;
		}
		if (abs_delta(s_drive.progress_count, count) >= DRIVE_STALL_COUNTS) {
			s_drive.progress_count = count;
			s_drive.progress_ms = now;
		} else if (now - s_drive.progress_ms > DRIVE_STALL_MS) {
			drive_stop("STALLED (no movement for 1 s): motor power on?");
			break;
		}
		if ((int32_t)(now - s_drive.next_pid) >= 0) {
			float dt = (float)(now - s_drive.last_pid) / 1000.0f;
			float sl, sr;

			drive_wheels_mm(snapshot, &l, &r);
			sl = (l - s_drive.pid_l) / 10.0f / dt;
			sr = (r - s_drive.pid_r) / 10.0f / dt;
			s_drive.pid_l = l;
			s_drive.pid_r = r;
			s_drive.last_pid = now;
			s_drive.next_pid += DRIVE_PID_MS;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (!wheel_speed_control_update(&s_drive.pid, sl, sr, dt, &s_drive.out)) {
				drive_stop("wheel PID refused its inputs");
				break;
			}
			drive_pwm(s_drive.out.left_pwm_percent, s_drive.out.right_pwm_percent);
		}
		break;
	default:
		break;
	}
}

static void sensor_test_task(INT stacd, void *exinf)
{
	line_adc_reader_t reader = {0};
	bool initialized = false;
	bool black = false;
	uint32_t last_report = 0U;
	ER error;

	(void)stacd;
	(void)exinf;
	init_bench_reader(&reader, "BAR-H02");
	error = encoder_source_register();
	if (error == E_OK) error = gpio_dispatcher_start();
	if (error != E_OK) report_error_forever("BAR-H02", "encoder startup", error);
	tm_printf((UB*)"[HWTEST][BAR-H02] GP28 barcode; %s; "
		  "module_counts=%u (0 = learn from a scan); encoder signs: left=-1 right=+1; "
		  "white<=%u black>=%u\n",
		  BARCODE_TEST_DRIVE > 0 ? "DRIVE MODE" : "keep motor power off",
		  (unsigned)BARCODE_TEST_MODULE_COUNTS, (unsigned)BARCODE_TEST_WHITE_ADC,
		  (unsigned)BARCODE_TEST_BLACK_ADC);
	if (BARCODE_TEST_DRIVE > 0) {
		drive_init();
		tm_printf((UB*)"[HWTEST][BAR-H02] DRIVE MODE (motor power ON): put the barcode sensor "
			  ">= 3 cm before the label, square to it, with %u mm clear ahead; GO = "
			  "drive straight at %u cm/s (no steering), GO again = stop\n",
			  (unsigned)BARCODE_TEST_DRIVE_MM, (unsigned)BARCODE_TEST_DRIVE);
		tm_printf((UB*)"[HWTEST][BAR-H02] drive speed %u cm/s is %s the main robot's line "
			  "speed (%u cm/s, vehicle_controller_task.h)\n", (unsigned)BARCODE_TEST_DRIVE,
			  BARCODE_TEST_DRIVE == VEHICLE_LINE_SPEED_CM_S ? "THE SAME AS" :
			  (BARCODE_TEST_DRIVE < VEHICLE_LINE_SPEED_CM_S ? "SLOWER than" : "FASTER than"),
			  (unsigned)VEHICLE_LINE_SPEED_CM_S);
	} else {
		tm_printf((UB*)"[HWTEST][BAR-H02] push the robot straight over the WHOLE label "
			  "(start and stop on white)\n");
	}
	tm_printf((UB*)"[HWTEST][BAR-H02] a PASS report follows %u mm after the label%s%s%s\n",
		  (unsigned)PASS_END_MM, BARCODE_TEST_EXPECT[0] ? "; expecting *" : "",
		  BARCODE_TEST_EXPECT, BARCODE_TEST_EXPECT[0] ? "*" : "");
	for (;;) {
		ir_readings_t readings;
		encoder_snapshot_t snapshot;
		uint32_t timestamp;
		int32_t count;
		int32_t edge_count;

		error = line_adc_reader_read_barcode(&reader, &readings, &timestamp);
		if (error != E_OK) report_error_forever("BAR-H02", "ADC read", error);
		error = encoder_source_snapshot(&snapshot);
		if (error != E_OK || !snapshot.valid)
			report_error_forever("BAR-H02", "encoder snapshot", error != E_OK ? error : E_IO);
		/* Straight travel gives negative raw left counts and positive right counts. */
		count = (int32_t)((-(int64_t)snapshot.left_count + snapshot.right_count) / 2);
		black = bench_barcode_black(readings.barcode.raw,
					    initialized ? black : readings.barcode.raw >= BARCODE_TEST_BLACK_ADC);
		if (readings.barcode.raw < s_bench_adc_min) s_bench_adc_min = readings.barcode.raw;
		if (readings.barcode.raw > s_bench_adc_max) s_bench_adc_max = readings.barcode.raw;
		if (!initialized) {
			s_debounce_black = black;
			initialized = barcode_scanner_init(&s_bench_scanner, &s_bench_barcode_config,
							   black, count);
			if (!initialized) report_error_forever("BAR-H02", "scanner init", E_SYS);
		} else {
			bool filtered_black = bench_debounce(black, count, &edge_count);
			if (!barcode_scanner_update(&s_bench_scanner, filtered_black, edge_count))
				report_error_forever("BAR-H02", "scanner update", E_SYS);
			pass_sample(readings.barcode.raw, filtered_black, edge_count, count,
				    timestamp);
		}
		if (s_drive.state != DRIVE_OFF) drive_step(timestamp, &snapshot, count);
		report_bench_barcode();
		if (s_report_pending && s_drive.state != DRIVE_RUN) {
			s_report_pending = false;
			pass_report(s_report_count);
		}
		if (s_drive.state != DRIVE_RUN && s_drive.state != DRIVE_DELAY &&
		    !s_pass_active && !s_bench_scanner.collecting && !s_bench_scanner.in_frame &&
		    timestamp - last_report >= SENSOR_TEST_INTERVAL_MS) {
			last_report = timestamp;
			tm_printf((UB*)"[HWTEST][BAR-H02] adc=%u black=%u left=%d right=%d mean=%d\n",
				  (unsigned)readings.barcode.raw, black ? 1U : 0U,
				  (int)snapshot.left_count, (int)snapshot.right_count, (int)count);
		}
		/* 2 ms samples (busy-wait) while driving or over a label; at rest
		 * on white yield to other tasks (one kernel tick, ~10 ms). */
		if (s_drive.state == DRIVE_RUN || s_drive.state == DRIVE_DELAY || s_pass_active ||
		    s_bench_scanner.collecting || black) {
			WaitUsec(BENCH_SAMPLE_US - BENCH_READ_US);
		} else {
			error = tk_dly_tsk(2U);
			if (error != E_OK) report_error_forever("BAR-H02", "delay", error);
		}
	}
}
#elif APP_SENSOR_TEST == 2 || APP_SENSOR_TEST == 6
#define IMU_LIVE_SAMPLE_DELAY_MS 50U
#define IMU_LIVE_REPORT_INTERVAL_MS 1000U
#define IMU_LIVE_STARTUP_SAMPLES 5U
#if APP_SENSOR_TEST == 6
/* Bench-only re-arm: clear the tilt flag after this long back on level ground. */
#define IMU_TILT_CLEAR_LEVEL_MS 500U
#define IMU_TILT_CLEAR_SAMPLES (IMU_TILT_CLEAR_LEVEL_MS / IMU_LIVE_SAMPLE_DELAY_MS)
#endif
#if APP_SENSOR_TEST == 2
#define IMU_PROBE_INTERVAL_MS 20U
#define IMU_PROBE_READY_WAIT_MS 100U
#define IMU_PROBE_ZYXDA 0x08U
#define IMU_PROBE_CTRL1 0x2FU
#define IMU_PROBE_CTRL4 0x80U

static ER imu_check_trace_clock(void)
{
	SYSTIM before, after;
	uint32_t ticks_before, ticks_after, elapsed_ms, elapsed_ticks;
	ER error;

	error = tk_get_otm(&before);
	if (error != E_OK) return error;
	ticks_before = in_w(TIMER_TIMERAWL);
	error = tk_dly_tsk(250U);
	if (error != E_OK) return error;
	ticks_after = in_w(TIMER_TIMERAWL);
	error = tk_get_otm(&after);
	if (error != E_OK) return error;
	elapsed_ms = after.lo - before.lo;
	elapsed_ticks = ticks_after - ticks_before;
	tm_printf((UB*)"[HWTEST][IMU] trace clock comparison: kernel_ms=%u timer_ticks=%u "
		  "(expected about 1000 ticks/ms)\n",
		  (unsigned)elapsed_ms, (unsigned)elapsed_ticks);
	return E_OK;
}

static void imu_report_identity(const lsm303dlhc_i2c_t *sensor)
{
	uint8_t reg = 0x0FU, identity = 0U;
	T_I2C_EXEC exec = {sensor->address, 1, &reg, 1, &identity};
	SZ actual = 0;
	ER error = tk_swri_dev(sensor->device, TDN_I2C_EXEC, &exec,
			       sizeof(exec), &actual);

	if (error != E_OK || actual != (SZ)sizeof(exec))
		tm_printf((UB*)"[HWTEST][IMU] WHO_AM_I read failed: %d size=%d\n",
			  (int)error, (int)actual);
	else
		tm_printf((UB*)"[HWTEST][IMU] WHO_AM_I: 0x%02x\n", (unsigned)identity);
}

static ER imu_write_probe_register(const lsm303dlhc_i2c_t *sensor,
				   uint8_t reg, uint8_t value)
{
	uint8_t data[2] = {reg, value};
	uint8_t readback = 0U;
	T_I2C_EXEC exec = {sensor->address, 1, &reg, 1, &readback};
	SZ actual = 0;
	ER error = tk_swri_dev(sensor->device, (W)sensor->address, data,
			       sizeof(data), &actual);

	if (error != E_OK || actual != (SZ)sizeof(data)) {
		tm_printf((UB*)"[HWTEST][IMU] probe register 0x%02x write failed: %d size=%d\n",
			  (unsigned)reg, (int)error, (int)actual);
		return error != E_OK ? error : E_IO;
	}
	actual = 0;
	error = tk_swri_dev(sensor->device, TDN_I2C_EXEC, &exec, sizeof(exec), &actual);
	if (error != E_OK || actual != (SZ)sizeof(exec)) {
		tm_printf((UB*)"[HWTEST][IMU] probe register 0x%02x readback failed: %d size=%d\n",
			  (unsigned)reg, (int)error, (int)actual);
		return error != E_OK ? error : E_IO;
	}
	if (readback != value) {
		tm_printf((UB*)"[HWTEST][IMU] probe register 0x%02x readback mismatch: "
			  "expected=0x%02x actual=0x%02x\n",
			  (unsigned)reg, (unsigned)value, (unsigned)readback);
		return E_IO;
	}
	return E_OK;
}

static ER imu_set_probe_mode(const lsm303dlhc_i2c_t *sensor)
{
	ER error = imu_write_probe_register(sensor, 0x23U, IMU_PROBE_CTRL4);

	if (error != E_OK) return error;
	return imu_write_probe_register(sensor, 0x20U, IMU_PROBE_CTRL1);
}

static ER imu_read_probe_register(const lsm303dlhc_i2c_t *sensor, uint8_t reg,
				  uint8_t *value)
{
	T_I2C_EXEC exec = {sensor->address, 1, &reg, 1, value};
	SZ actual = 0;
	ER error = tk_swri_dev(sensor->device, TDN_I2C_EXEC, &exec,
			       sizeof(exec), &actual);

	if (error != E_OK || actual != (SZ)sizeof(exec)) {
		tm_printf((UB*)"[HWTEST][IMU] register 0x%02x read failed: %d size=%d\n",
			  (unsigned)reg, (int)error, (int)actual);
		return error != E_OK ? error : E_IO;
	}
	return E_OK;
}

static ER imu_read_status(const lsm303dlhc_i2c_t *sensor, uint8_t *status)
{
	return imu_read_probe_register(sensor, 0x27U, status);
}

static ER imu_probe_bdu(const lsm303dlhc_i2c_t *sensor)
{
	const uint8_t low_regs[3] = {0x28U, 0x2AU, 0x2CU};
	uint8_t first[3], held[3], released[3], high;
	bool changed_while_held = false, changed_after_release = false;
	unsigned i;
	ER error;

	tm_printf((UB*)"[HWTEST][IMU] BDU probe: move the board continuously for 5 seconds\n");
	error = tk_dly_tsk(2000U);
	if (error != E_OK) return error;
	for (i = 0; i < 3U; i++) {
		error = imu_read_probe_register(sensor, low_regs[i], &first[i]);
		if (error != E_OK) return error;
	}
	error = tk_dly_tsk(1000U);
	if (error != E_OK) return error;
	for (i = 0; i < 3U; i++) {
		error = imu_read_probe_register(sensor, low_regs[i], &held[i]);
		if (error != E_OK) return error;
		if (held[i] != first[i]) changed_while_held = true;
	}
	for (i = 0; i < 3U; i++) {
		error = imu_read_probe_register(sensor, low_regs[i] + 1U, &high);
		if (error != E_OK) return error;
	}
	error = tk_dly_tsk(200U);
	if (error != E_OK) return error;
	for (i = 0; i < 3U; i++) {
		error = imu_read_probe_register(sensor, low_regs[i], &released[i]);
		if (error != E_OK) return error;
		if (released[i] != held[i]) changed_after_release = true;
		error = imu_read_probe_register(sensor, low_regs[i] + 1U, &high);
		if (error != E_OK) return error;
	}
	tm_printf((UB*)"[HWTEST][IMU] BDU low bytes: first=%02x,%02x,%02x held=%02x,%02x,%02x "
		  "released=%02x,%02x,%02x result=%s\n",
		  (unsigned)first[0], (unsigned)first[1], (unsigned)first[2],
		  (unsigned)held[0], (unsigned)held[1], (unsigned)held[2],
		  (unsigned)released[0], (unsigned)released[1], (unsigned)released[2],
		  changed_while_held ? "changed-while-held" :
		  changed_after_release ? "consistent-not-proof" : "inconclusive");
	return E_OK;
}

static ER imu_read_unverified(const lsm303dlhc_i2c_t *sensor,
			      const lsm303dlhc_axis_map_t *mapping,
			      accel_data_t *sample, uint8_t *status, uint8_t *after,
			      uint32_t *burst_us, uint32_t *status_us,
			      T_I2C_PROBE_TRACE *trace)
{
	uint8_t reg = 0xA7U;
	uint8_t bytes[7];
	T_I2C_EXEC exec = {sensor->address, 1, &reg, sizeof(bytes), bytes};
	SZ actual = 0;
	uint32_t start = in_w(TIMER_TIMERAWL);
	uint32_t burst_end, status_start, status_end;
	ER error = tk_swri_dev(sensor->device, TDN_I2C_EXEC, &exec, sizeof(exec), &actual);

	burst_end = in_w(TIMER_TIMERAWL);
	if (error != E_OK || actual != (SZ)sizeof(exec)) {
		tm_printf((UB*)"[HWTEST][IMU] direct read failed: %d size=%d\n",
			  (int)error, (int)actual);
		return error != E_OK ? error : E_IO;
	}
	error = dev_i2c_probe_trace(1U, trace);
	if (error != E_OK) {
		tm_printf((UB*)"[HWTEST][IMU] I2C trace read failed: %d\n", (int)error);
		return error;
	}
	if (trace->irq_count == 0U || trace->receive_irq_count == 0U) {
		tm_printf((UB*)"[HWTEST][IMU] I2C trace missing receive interrupt\n");
		return E_IO;
	}
	status_start = in_w(TIMER_TIMERAWL);
	error = imu_read_status(sensor, after);
	status_end = in_w(TIMER_TIMERAWL);
	if (error != E_OK) return error;
	if (!lsm303dlhc_accel_decode(&bytes[1], mapping, sample)) {
		tm_printf((UB*)"[HWTEST][IMU] direct read returned invalid accelerometer data\n");
		return E_IO;
	}
	*status = bytes[0];
	*burst_us = burst_end - start;
	*status_us = status_end - status_start;
	return E_OK;
}

static ER imu_check_freshness(const lsm303dlhc_i2c_t *sensor, uint8_t before,
			      uint8_t after)
{
	uint8_t ready;
	SYSTIM start, now;
	ER error;

	if (!(before & IMU_PROBE_ZYXDA) || (after & IMU_PROBE_ZYXDA)) {
		tm_printf((UB*)"[HWTEST][IMU] freshness inconclusive: before=0x%02x after=0x%02x\n",
			  (unsigned)before, (unsigned)after);
		return E_OK;
	}
	error = tk_get_otm(&start);
	if (error != E_OK) return error;
	for (;;) {
		error = tk_dly_tsk(1U);
		if (error != E_OK) return error;
		error = imu_read_status(sensor, &ready);
		if (error != E_OK) return error;
		error = tk_get_otm(&now);
		if (error != E_OK) return error;
		if (now.lo - start.lo > IMU_PROBE_READY_WAIT_MS ||
		    (!(ready & IMU_PROBE_ZYXDA) &&
		     now.lo - start.lo >= IMU_PROBE_READY_WAIT_MS)) {
			tm_printf((UB*)"[HWTEST][IMU] freshness timeout: before=0x%02x after=0x%02x "
				  "status=0x%02x elapsed=%u ms limit=%u ms\n",
				  (unsigned)before, (unsigned)after, (unsigned)ready,
				  (unsigned)(now.lo - start.lo), IMU_PROBE_READY_WAIT_MS);
			return E_TMOUT;
		}
		if (ready & IMU_PROBE_ZYXDA) break;
	}
	tm_printf((UB*)"[HWTEST][IMU] data-ready transition observed: before=0x%02x "
		  "after=0x%02x ready=0x%02x wait=%u ms\n",
		  (unsigned)before, (unsigned)after, (unsigned)ready,
		  (unsigned)(now.lo - start.lo));
	return E_OK;
}

#endif

static void sensor_test_task(INT stacd, void *exinf)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t config;
#if APP_SENSOR_TEST == 2
	accel_data_t initial_sample;
	uint32_t initial_timestamp;
	unsigned probe_count;
#else
	accel_data_t warmup_sample;
	uint32_t warmup_timestamp;
#endif
	imu_state_t imu;
	uint32_t last_report_ms = 0U;
	unsigned startup_samples = 0U;
#if APP_SENSOR_TEST == 6
	unsigned level_samples = 0U;
#endif
	ER error;

	(void)stacd;
	(void)exinf;
	error = lsm303dlhc_i2c_default_config(&config, 20U);
	if (error != E_OK) report_error_forever("IMU", "default config", error);
	config.profile = LSM303DLHC_I2C_PROFILE_32_FAMILY;
	error = lsm303dlhc_i2c_init(&sensor, &config);
	if (error != E_OK) report_error_forever("IMU", "sensor init", error);
#if APP_SENSOR_TEST == 2
	error = lsm303dlhc_i2c_read(&sensor, &initial_sample, &initial_timestamp);
	if (error != E_OK) report_error_forever("IMU", "initial reader sample", error);
	tm_printf((UB*)"[HWTEST][IMU] initial_open t_lower=%u ms raw_x=%d raw_y=%d raw_z=%d\n",
		  (unsigned)initial_timestamp, (int)initial_sample.raw_x,
		  (int)initial_sample.raw_y, (int)initial_sample.raw_z);
	error = imu_check_trace_clock();
	if (error != E_OK) report_error_forever("IMU", "trace clock comparison", error);
	imu_report_identity(&sensor);
	error = imu_set_probe_mode(&sensor);
	if (error != E_OK) report_error_forever("IMU", "probe mode", error);

	tm_printf((UB*)"[HWTEST][IMU] fast BDU probe CTRL1=0x2f CTRL4=0x80; "
		  "direct readings, freshness unverified\n");
	tm_printf((UB*)"[HWTEST][IMU] shared I2C wake-up fix enabled\n");
	error = imu_probe_bdu(&sensor);
	if (error != E_OK) report_error_forever("IMU", "BDU probe", error);
	for (probe_count = 0U; probe_count < 5U; probe_count++) {
		accel_data_t sample;
		SYSTIM now;
		uint32_t burst_us, status_us;
		uint8_t status, after;
		T_I2C_PROBE_TRACE trace;

		error = imu_read_unverified(&sensor, &config.mapping, &sample, &status, &after,
					    &burst_us, &status_us, &trace);
		if (error != E_OK) report_error_forever("IMU", "read", error);
		error = tk_get_otm(&now);
		if (error != E_OK) report_error_forever("IMU", "clock", error);
		error = imu_check_freshness(&sensor, status, after);
		if (error != E_OK) report_error_forever("IMU", "freshness check", error);
		tm_printf((UB*)"[HWTEST][IMU] unverified t_read=%u ms burst_us=%u status_us=%u "
			  "status=0x%02x raw_x=%d raw_y=%d raw_z=%d "
			  "ax_mg=%ld ay_mg=%ld az_mg=%ld\n",
			  (unsigned)now.lo, (unsigned)burst_us, (unsigned)status_us,
			  (unsigned)status, (int)sample.raw_x, (int)sample.raw_y,
			  (int)sample.raw_z,
			  (long)(sample.ax * 1000.0f / 9.80665f),
			  (long)(sample.ay * 1000.0f / 9.80665f),
			  (long)(sample.az * 1000.0f / 9.80665f));
		tm_printf((UB*)"[HWTEST][IMU] I2C trace driver_us=%u first_irq_us=%u "
			  "last_irq_us=%u irq_count=%u\n",
			  (unsigned)(trace.end_us - trace.start_us),
			  (unsigned)(trace.first_irq_us - trace.start_us),
			  (unsigned)(trace.last_irq_us - trace.start_us),
			  (unsigned)trace.irq_count);
		tm_printf((UB*)"[HWTEST][IMU] I2C phases recv_command_us=%u "
			  "first_rx_us=%u last_rx_us=%u rx_resume_us=%u "
			  "stop_arm_us=%u stop_pending=%u\n",
			  (unsigned)(trace.receive_command_us - trace.start_us),
			  (unsigned)(trace.first_receive_irq_us - trace.start_us),
			  (unsigned)(trace.last_receive_irq_us - trace.start_us),
			  (unsigned)(trace.receive_resume_us - trace.start_us),
			  (unsigned)(trace.stop_arm_us - trace.start_us),
			  (unsigned)trace.stop_pending);
		error = tk_dly_tsk(IMU_PROBE_INTERVAL_MS);
		if (error != E_OK) report_error_forever("IMU", "delay", error);
	}
	error = lsm303dlhc_i2c_close(&sensor);
	if (error != E_OK) report_error_forever("IMU", "probe close", error);
	sensor = (lsm303dlhc_i2c_t){0};
	error = lsm303dlhc_i2c_init(&sensor, &config);
	if (error != E_OK) report_error_forever("IMU", "vehicle reader init", error);
#else
	tm_printf((UB*)"[HWTEST][IMU] single-open tilt mode: no BDU probe or sensor reopen\n");
	/* Bench-only: keep the startup outlier visible without latching the detector. */
	error = lsm303dlhc_i2c_read(&sensor, &warmup_sample, &warmup_timestamp);
	if (error != E_OK) report_error_forever("IMU", "warm-up reader sample", error);
	tm_printf((UB*)"[HWTEST][IMU] warmup_sample t_lower=%u ms "
		  "raw_x=%d raw_y=%d raw_z=%d (not evaluated for tilt)\n",
		  (unsigned)warmup_timestamp, (int)warmup_sample.raw_x,
		  (int)warmup_sample.raw_y, (int)warmup_sample.raw_z);
	error = tk_dly_tsk(IMU_LIVE_SAMPLE_DELAY_MS);
	if (error != E_OK) report_error_forever("IMU", "warm-up delay", error);
#endif
	if (!imu_processor_init(&imu))
		report_error_forever("IMU", "tilt processor init", E_SYS);
	tm_printf((UB*)"[HWTEST][IMU] vehicle reader: CTRL1=0x2f CTRL4=0x80; "
		  "data-ready gating enabled; sample time is a lower bound\n");
#if APP_SENSOR_TEST == 6
	tm_printf((UB*)"[HWTEST][IMU] tilt-only check: flag clears after %u ms level "
		  "(bench only; vehicle still latches)\n", (unsigned)IMU_TILT_CLEAR_LEVEL_MS);
#else
	tm_printf((UB*)"[HWTEST][IMU] tilt-only check: reset on level ground between trials\n");
#endif
	for (;;) {
		accel_data_t sample;
		SYSTIM now;
		uint32_t timestamp;
		bool was_tilted = imu.tilt_detected;
		bool tilt_latched;
		bool tilt_cleared = false;
		bool report_startup = startup_samples < IMU_LIVE_STARTUP_SAMPLES;

		error = lsm303dlhc_i2c_read(&sensor, &sample, &timestamp);
		if (error != E_OK) report_error_forever("IMU", "vehicle reader", error);
		error = tk_get_otm(&now);
		if (error != E_OK) report_error_forever("IMU", "clock", error);
		/* No odometry in this diagnostic; tilt detection does not use distance. */
		if (!imu_processor_update(&imu, &sample, 0.0f, now.lo))
			report_error_forever("IMU", "tilt processor update", E_SYS);
		tilt_latched = !was_tilted && imu.tilt_detected;
#if APP_SENSOR_TEST == 6
		if (tilt_latched)
			tm_printf((UB*)"[HWTEST][IMU] TILT DETECTED\n");
		/* The processor leaves IMU_STATE_TILT once level; re-arm the latch here only. */
		if (imu.tilt_detected && imu.fsm_state != IMU_STATE_TILT) {
			if (++level_samples >= IMU_TILT_CLEAR_SAMPLES) {
				imu.tilt_detected = false;
				level_samples = 0U;
				tilt_cleared = true;
				tm_printf((UB*)"[HWTEST][IMU] TILT CLEARED (level for %u ms)\n",
					  (unsigned)IMU_TILT_CLEAR_LEVEL_MS);
			}
		} else {
			level_samples = 0U;
		}
#else
		if (tilt_latched)
			tm_printf((UB*)"[HWTEST][IMU] TILT DETECTED (latched until reset)\n");
#endif
		if (report_startup || tilt_latched || tilt_cleared ||
		    now.lo - last_report_ms >= IMU_LIVE_REPORT_INTERVAL_MS) {
			if (report_startup) startup_samples++;
			tm_printf((UB*)"[HWTEST][IMU] vehicle startup_sample=%u "
				  "t_lower=%u ms age_bound=%u ms "
				  "raw_x=%d raw_y=%d raw_z=%d ax_mg=%ld ay_mg=%ld az_mg=%ld "
				  "tilt_detected=%u\n",
				  report_startup ? startup_samples : 0U,
				  (unsigned)timestamp, (unsigned)(now.lo - timestamp),
				  (int)sample.raw_x, (int)sample.raw_y, (int)sample.raw_z,
				  (long)(sample.ax * 1000.0f / 9.80665f),
				  (long)(sample.ay * 1000.0f / 9.80665f),
				  (long)(sample.az * 1000.0f / 9.80665f),
				  imu.tilt_detected ? 1U : 0U);
			last_report_ms = now.lo;
		}
		error = tk_dly_tsk(IMU_LIVE_SAMPLE_DELAY_MS);
		if (error != E_OK) report_error_forever("IMU", "delay", error);
	}
}
#elif APP_SENSOR_TEST == 10
#define IMU_HUMP_TEST_COUNTS_PER_REV 2500U
#define IMU_HUMP_TEST_WHEEL_CIRC_CM 20.4f
#define IMU_HUMP_TEST_SAMPLE_DELAY_MS 50U
#define IMU_HUMP_TEST_REPORT_INTERVAL_MS 250U

static const odometry_producer_config_t s_imu_hump_odometry_config = {
	{IMU_HUMP_TEST_COUNTS_PER_REV, IMU_HUMP_TEST_WHEEL_CIRC_CM},
	{IMU_HUMP_TEST_COUNTS_PER_REV, IMU_HUMP_TEST_WHEEL_CIRC_CM},
	-1, 1
};

static void sensor_test_task(INT stacd, void *exinf)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t sensor_config;
	odometry_producer_t odometry_state = {0};
	wheel_odometry_message_t odometry = {0};
	uint32_t last_odometry_ms;
	bool encoder_faulted = false;
	encoder_snapshot_t snapshot;
	accel_data_t warmup_sample;
	accel_data_t sample;
	imu_state_t imu;
	SYSTIM now;
	uint32_t sample_timestamp;
	uint32_t warmup_timestamp;
	uint32_t last_report_ms;
	bool have_report = false;
	uint8_t last_state = 0U;
	ER error;

	(void)stacd;
	(void)exinf;
	error = encoder_source_register();
	if (error != E_OK) report_error_forever("IMU-HUMP", "encoder registration", error);
	error = gpio_dispatcher_start();
	if (error != E_OK) report_error_forever("IMU-HUMP", "GPIO dispatcher", error);
	error = lsm303dlhc_i2c_default_config(&sensor_config, 20U);
	if (error != E_OK) report_error_forever("IMU-HUMP", "default config", error);
	sensor_config.profile = LSM303DLHC_I2C_PROFILE_32_FAMILY;
	error = lsm303dlhc_i2c_init(&sensor, &sensor_config);
	if (error != E_OK) report_error_forever("IMU-HUMP", "sensor init", error);
	error = lsm303dlhc_i2c_read(&sensor, &warmup_sample, &warmup_timestamp);
	if (error != E_OK) report_error_forever("IMU-HUMP", "warm-up read", error);
	tm_printf((UB*)"[HWTEST][IMU-H02] warmup_sample t_lower=%u ms "
		  "raw_x=%d raw_y=%d raw_z=%d (not measured)\n",
		  (unsigned)warmup_timestamp, (int)warmup_sample.raw_x,
		  (int)warmup_sample.raw_y, (int)warmup_sample.raw_z);
	error = tk_dly_tsk(IMU_HUMP_TEST_SAMPLE_DELAY_MS);
	if (error != E_OK) report_error_forever("IMU-HUMP", "warm-up delay", error);
	/* Same processor setup and startup calibration as the vehicle IMU task. */
	if (!imu_processor_init_vehicle(&imu))
		report_error_forever("IMU-HUMP", "processor init", E_SYS);

	{
		imu_level_calibrator_t calibrator;
		imu_level_result_t result = IMU_LEVEL_INCOMPLETE;

		tm_printf((UB*)"[HWTEST][IMU-H02] level calibration: keep the robot still "
			  "on flat ground for about 1 s\n");
		while (result != IMU_LEVEL_OK) {
			imu_level_calibrator_reset(&calibrator);
			while (!imu_level_calibrator_full(&calibrator)) {
				error = lsm303dlhc_i2c_read(&sensor, &sample, &sample_timestamp);
				if (error != E_OK) report_error_forever("IMU-HUMP", "level read", error);
				if (!imu_level_calibrator_add(&calibrator, &sample))
					report_error_forever("IMU-HUMP", "level sample", E_PAR);
				error = tk_dly_tsk(IMU_HUMP_TEST_SAMPLE_DELAY_MS);
				if (error != E_OK) report_error_forever("IMU-HUMP", "level delay", error);
			}
			result = imu_processor_apply_level(&imu, &calibrator);
			if (result != IMU_LEVEL_OK)
				tm_printf((UB*)"[HWTEST][IMU-H02] level calibration %s; retrying\n",
					  result == IMU_LEVEL_MOVING ? "saw movement" : "found it not level");
		}
		tm_printf((UB*)"[HWTEST][IMU-H02] level offsets: pitch_mdeg=%ld roll_mdeg=%ld "
			  "(removed from every reading); mount_yaw_mdeg=%ld\n",
			  (long)(imu.pitch_offset_deg * 1000.0f),
			  (long)(imu.roll_offset_deg * 1000.0f),
			  (long)(IMU_MOUNT_YAW_DEG * 1000.0f));
	}

	error = encoder_source_snapshot(&snapshot);
	if (error != E_OK || !snapshot.valid)
		report_error_forever("IMU-HUMP", "initial encoder snapshot",
				     error != E_OK ? error : E_IO);
	error = tk_get_otm(&now);
	if (error != E_OK) report_error_forever("IMU-HUMP", "clock", error);
	if (!odometry_producer_init(&odometry_state, &s_imu_hump_odometry_config,
				    snapshot.left_count, snapshot.right_count, now.lo))
		report_error_forever("IMU-HUMP", "odometry init", E_SYS);

	last_report_ms = now.lo;
	last_odometry_ms = now.lo;
	/* tm_printf has no %f support; print the circumference in millimetres. */
	tm_printf((UB*)"[HWTEST][IMU-H02] odometry counts/rev=%u wheel_circumference_mm=%u "
		  "left_sign=-1 right_sign=+1\n",
		  (unsigned)IMU_HUMP_TEST_COUNTS_PER_REV,
		  (unsigned)(IMU_HUMP_TEST_WHEEL_CIRC_CM * 10.0f + 0.5f));
	tm_printf((UB*)"[HWTEST][IMU-H02] geometry: track_W_mm=%u wheelbase_L_mm=%u; "
		  "heights below are in 0.1 mm (100 = 1 cm)\n",
		  (unsigned)(IMU_TRACK_WIDTH_M * 1000.0f + 0.5f),
		  (unsigned)(IMU_WHEELBASE_M * 1000.0f + 0.5f));
	tm_printf((UB*)"[HWTEST][IMU-H02] motors are not started; keep motor power off. "
		  "Push straight forward over a known gentle ramp.\n");
	for (;;) {
		error = lsm303dlhc_i2c_read(&sensor, &sample, &sample_timestamp);
		if (error != E_OK) report_error_forever("IMU-HUMP", "sensor read", error);
		error = tk_get_otm(&now);
		if (error != E_OK) report_error_forever("IMU-HUMP", "clock", error);
		error = encoder_source_snapshot(&snapshot);
		if (error != E_OK)
			report_error_forever("IMU-HUMP", "encoder snapshot", error);
		/* A missed quadrature edge (e.g. a wheel jolting off the ramp) latches
		 * the encoder invalid until restart. The hump height does not use
		 * odometry, so freeze travel_mm and keep measuring. */
		if (!snapshot.valid && !encoder_faulted) {
			encoder_faulted = true;
			tm_printf((UB*)"[HWTEST][IMU-H02] encoder fault latched (missed edge); "
				  "travel_mm frozen at %ld, height measurement continues\n",
				  (long)odometry.distance_mm);
		}
		/* The kernel clock ticks every 10 ms and the producer rejects a repeated
		 * timestamp, so keep the previous distance until the clock advances. */
		if (!encoder_faulted && now.lo != last_odometry_ms) {
			if (!odometry_producer_update(&odometry_state, snapshot.left_count,
						      snapshot.right_count, now.lo, &odometry))
				report_error_forever("IMU-HUMP", "odometry update", E_SYS);
			last_odometry_ms = now.lo;
		}
		if (!imu_processor_update(&imu, &sample,
					  (float)odometry.distance_mm / 1000.0f, now.lo))
			report_error_forever("IMU-HUMP", "processor update", E_SYS);

		if (!have_report ||
		    (uint32_t)(now.lo - last_report_ms) >= IMU_HUMP_TEST_REPORT_INTERVAL_MS ||
		    (uint8_t)imu.fsm_state != last_state) {
			tm_printf((UB*)"[HWTEST][IMU-H02] t=%u travel_mm=%ld "
				  "pitch_mdeg=%ld roll_mdeg=%ld "
				  "left=%ld right=%ld ball=%ld "
				  "state=%u hump=%ld peak=%ld tilt=%u\n",
				  (unsigned)now.lo, (long)odometry.distance_mm,
				  (long)(imu.terrain_pitch_deg * 1000.0f),
				  (long)(imu.terrain_roll_deg * 1000.0f),
				  (long)(imu.wheel_left_m * 10000.0f),
				  (long)(imu.wheel_right_m * 10000.0f),
				  (long)(imu.ball_m * 10000.0f),
				  (unsigned)imu.fsm_state,
				  (long)(imu.current_hump_m * 10000.0f),
				  (long)(imu.hump_peak_m * 10000.0f),
				  imu.tilt_detected ? 1U : 0U);
			last_report_ms = now.lo;
			last_state = (uint8_t)imu.fsm_state;
			have_report = true;
		}
		error = tk_dly_tsk(IMU_HUMP_TEST_SAMPLE_DELAY_MS);
		if (error != E_OK) report_error_forever("IMU-HUMP", "delay", error);
	}
}
#elif APP_SENSOR_TEST == 3
static void sensor_test_task(INT stacd, void *exinf)
{
	static const obs_task_config_t task_config = {
		50U, 500U, 1000U, 5000U, 500U, 300U,
		NULL, NULL, NULL, NULL
	};
	obs_pico_port_config_t hardware;
	ER error;

	(void)stacd;
	(void)exinf;
	hardware = obs_pico_port_default_config();
	hardware.echo_voltage_verified_3v3_safe = true;
	hardware.servo_limits_verified = true;

	error = obs_pico_port_register();
	if (error != E_OK) report_error_forever("ULTRASONIC", "GPIO registration", error);
	error = gpio_dispatcher_start();
	if (error != E_OK) report_error_forever("ULTRASONIC", "GPIO dispatcher", error);
	error = obs_pico_port_task_start(&hardware, &task_config);
	if (error != E_OK) report_error_forever("ULTRASONIC", "sensor task start", error);

	tm_printf((UB*)"[HWTEST][ULTRASONIC] live distance readings; GP5 trigger, "
		  "GP4 echo; servo moves to the ahead angle\n");
	for (;;) {
		obstacle_event_message_t event;
		INT received = tk_rcv_mbf(g_mbf_obstacle_events, &event, TMO_FEVR);

		if (received != (INT)sizeof(event))
			report_error_forever("ULTRASONIC", "event receive",
					     received < E_OK ? (ER)received : E_IO);
		if (event.header.type != VEHICLE_MESSAGE_OBSTACLE_EVENT ||
		    event.header.version != VEHICLE_MESSAGE_VERSION)
			report_error_forever("ULTRASONIC", "invalid event", E_SYS);
		if (event.distance_mm == OBS_DIST_NONE) {
			tm_printf((UB*)"[HWTEST][ULTRASONIC] t=%u ms distance=NONE "
				  "(clear or beyond range)\n",
				  (unsigned)event.header.timestamp_ms);
		} else {
			tm_printf((UB*)"[HWTEST][ULTRASONIC] t=%u ms distance=%u mm\n",
				  (unsigned)event.header.timestamp_ms,
				  (unsigned)event.distance_mm);
		}
	}
}
#elif APP_SENSOR_TEST == 4
#define SERVO_TEST_PIN 15U
#define SERVO_TEST_SLICE 7U
#define SERVO_TEST_DIV_REG (PWM_BASE + PWM_CHx_DIV + (SERVO_TEST_SLICE * 0x14U))
#define SERVO_TEST_DIVIDER 125U
#define SERVO_TEST_WRAP 19999U
#define SERVO_TEST_DELAY_MS 5000U
#define SERVO_TEST_SLICE_BASE (PWM_BASE + SERVO_TEST_SLICE * 0x14U)

static void sensor_test_task(INT stacd, void *exinf)
{
	ER error;

	(void)stacd;
	(void)exinf;
	error = pwm_set_enabled(SERVO_TEST_PIN, FALSE);
	if (error != E_OK) report_error_forever("SERVO", "disable PWM", error);
	error = pwm_set_cc(SERVO_TEST_PIN, 0U);
	if (error != E_OK) report_error_forever("SERVO", "clear PWM", error);
#if SERVO_TEST_US == 0
	tm_printf((UB*)"[HWTEST][SERVO] PWM off. Build with SERVO_TEST_US to select one pulse.\n");
#else
	error = pwm_set_wrap(SERVO_TEST_PIN, SERVO_TEST_WRAP);
	if (error != E_OK) report_error_forever("SERVO", "PWM period", error);
	out_w(SERVO_TEST_DIV_REG, SERVO_TEST_DIVIDER << 4);
	error = pwm_set_pin(SERVO_TEST_PIN);
	if (error != E_OK) report_error_forever("SERVO", "PWM pin", error);

	tm_printf((UB*)"[HWTEST][SERVO] GP15 will output %u us in 5 seconds; "
		  "disconnect servo power now if unsafe.\n", (unsigned)SERVO_TEST_US);
	error = tk_dly_tsk(SERVO_TEST_DELAY_MS);
	if (error != E_OK) report_error_forever("SERVO", "delay", error);
	error = pwm_set_cc(SERVO_TEST_PIN, SERVO_TEST_US);
	if (error != E_OK) report_error_forever("SERVO", "pulse width", error);
	error = pwm_set_enabled(SERVO_TEST_PIN, TRUE);
	if (error != E_OK) report_error_forever("SERVO", "enable PWM", error);
	tm_printf((UB*)"[HWTEST][SERVO] GP15 configured for %u us; no automatic sweep.\n",
		  (unsigned)SERVO_TEST_US);
	tm_printf((UB*)"[HWTEST][SERVO] reset_done=%u gpio_ctrl=0x%x pad=0x%x "
		  "csr=0x%x div=0x%x top=%u cc=0x%x\n",
		  (unsigned)((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) != 0U),
		  (unsigned)in_w(GPIO_CTRL(SERVO_TEST_PIN)),
		  (unsigned)in_w(GPIO(SERVO_TEST_PIN)),
		  (unsigned)in_w(SERVO_TEST_SLICE_BASE + PWM_CHx_CSR),
		  (unsigned)in_w(SERVO_TEST_DIV_REG),
		  (unsigned)in_w(SERVO_TEST_SLICE_BASE + PWM_CHx_TOP),
		  (unsigned)in_w(SERVO_TEST_SLICE_BASE + PWM_CHx_CC));
#endif
	tk_slp_tsk(TMO_FEVR);
}
#elif APP_SENSOR_TEST == 11
/* Obstacle width measurement; body in subsystem5/obstacle/obs_width_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	obs_width_hwtest_run();
}
#elif APP_SENSOR_TEST == 12
/* Turn-in-place heading test (DRIVES MOTORS); subsystem2/motion/spin_heading_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	spin_heading_hwtest_run();
}
#elif APP_SENSOR_TEST == 13
/* Manoeuvres + wheel balance (DRIVES MOTORS); subsystem2/motion/motor_moves_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	motor_moves_hwtest_run();
}
#elif APP_SENSOR_TEST == 14
/* Wheel speed PID tuning (DRIVES MOTORS); subsystem2/motion/pid_tune_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	pid_tune_hwtest_run();
}
#elif APP_SENSOR_TEST == 15
/* Obstacle width by spin scan (DRIVES MOTORS); subsystem2/motion/spin_width_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	spin_width_hwtest_run();
}
#elif APP_SENSOR_TEST == 16
/* Obstacle width by a diagonal pass, then pull out (DRIVES MOTORS);
 * subsystem2/motion/diag_width_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	diag_width_hwtest_run();
}
#elif APP_SENSOR_TEST == 17
/* Line following wiggle/curve test (DRIVES MOTORS);
 * subsystem2/controller/line_follow_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	line_follow_hwtest_run();
}
#elif APP_SENSOR_TEST == 18
/* Junction detection + action (DRIVES MOTORS); subsystem2/controller/junction_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	junction_hwtest_run();
}
#elif APP_SENSOR_TEST == 19
/* Wi-Fi, MQTT and dashboard link test (no motors);
 * subsystem1/wifi/wifi_link_hwtest.c. */
static void sensor_test_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	wifi_link_hwtest_run();
}
#else
#error Unsupported APP_SENSOR_TEST diagnostic
#endif

ER hardware_sensor_test_start(void)
{
	ID task = app_start_task((FP)sensor_test_task, SENSOR_TEST_PRIORITY,
				 TP_PRC1, "hardware_sensor_test");

	return task < E_OK ? (ER)task : E_OK;
}

#endif
