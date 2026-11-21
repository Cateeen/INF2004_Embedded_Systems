#include "app.h"
#include "hardware_sensor_test.h"

#include "subsystem6/gpio/gpio_dispatcher.h"
#include "subsystem6/odometry/encoder_source.h"
#include "subsystem4/imu/lsm303dlhc_i2c.h"
#include "subsystem3/line_barcode/line_adc_reader.h"
#include "subsystem5/obstacle/obs_pico_port.h"
#include "subsystem5/obstacle/obs_task.h"
#include "vehicle_buffers.h"
#include "vehicle_messages.h"

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 7
#include "subsystem6/odometry/odometry_task.h"
#endif

#if APP_SENSOR_TEST == 2 || APP_SENSOR_TEST == 6
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
	{2500U, 20.4f}, {2500U, 20.4f}, 1, 1
};

static const line_barcode_task_config_t s_line_barcode_task_config = {
	LINE_BARCODE_TEST_PERIOD_MS,
	200U,
	{LINE_BARCODE_TEST_COUNTS_PER_MODULE, 25U, 0U, 0U, 0U},
	NULL, NULL, NULL
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
		tm_printf((UB*)"[HWTEST][BAR-H02] char=%c %s dir=%d narrow_max=%u "
			  "wide_min=%u gap=%u offending=%u\n",
			  info.symbol ? info.symbol : '?', barcode_reject_name(info.reject),
			  (int)info.direction, (unsigned)info.narrow_max, (unsigned)info.wide_min,
			  (unsigned)info.gap_counts, (unsigned)info.offending_counts);
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

static void sensor_test_task(INT stacd, void *exinf)
{
	line_adc_reader_t reader = {0};
	bool initialized = false;
	uint32_t last_report = 0U;
	ER error;

	(void)stacd;
	(void)exinf;
	init_bench_reader(&reader, "BAR-H02");
	error = encoder_source_register();
	if (error == E_OK) error = gpio_dispatcher_start();
	if (error != E_OK) report_error_forever("BAR-H02", "encoder startup", error);
	tm_printf((UB*)"[HWTEST][BAR-H02] GP28 barcode; keep motor power off; "
		  "module_counts=%u (0 = learn from a scan)\n",
		  (unsigned)BARCODE_TEST_MODULE_COUNTS);
	for (;;) {
		ir_readings_t readings;
		encoder_snapshot_t snapshot;
		uint32_t timestamp;
		int32_t count;

		error = line_adc_reader_read_barcode(&reader, &readings, &timestamp);
		if (error != E_OK) report_error_forever("BAR-H02", "ADC read", error);
		error = encoder_source_snapshot(&snapshot);
		if (error != E_OK || !snapshot.valid)
			report_error_forever("BAR-H02", "encoder snapshot", error != E_OK ? error : E_IO);
		/* Same wheel polarities as the vehicle calibration (+1, +1). */
		count = (int32_t)(((int64_t)snapshot.left_count + snapshot.right_count) / 2);
		if (!initialized) {
			initialized = barcode_scanner_init(&s_bench_scanner, &s_bench_barcode_config,
							   readings.barcode.black, count);
			if (!initialized) report_error_forever("BAR-H02", "scanner init", E_SYS);
		} else if (!barcode_scanner_update(&s_bench_scanner, readings.barcode.black, count)) {
			report_error_forever("BAR-H02", "scanner update", E_SYS);
		}
		report_bench_barcode();
		if (!s_bench_scanner.collecting && !s_bench_scanner.in_frame &&
		    timestamp - last_report >= SENSOR_TEST_INTERVAL_MS) {
			last_report = timestamp;
			tm_printf((UB*)"[HWTEST][BAR-H02] adc=%u black=%u left=%d right=%d\n",
				  (unsigned)readings.barcode.raw, readings.barcode.black ? 1U : 0U,
				  (int)snapshot.left_count, (int)snapshot.right_count);
		}
		error = tk_dly_tsk(10U);
		if (error != E_OK) report_error_forever("BAR-H02", "delay", error);
	}
}
#elif APP_SENSOR_TEST == 2 || APP_SENSOR_TEST == 6
#define IMU_LIVE_SAMPLE_DELAY_MS 50U
#define IMU_LIVE_REPORT_INTERVAL_MS 1000U
#define IMU_LIVE_STARTUP_SAMPLES 5U
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
	tm_printf((UB*)"[HWTEST][IMU] tilt-only check: reset on level ground between trials\n");
	for (;;) {
		accel_data_t sample;
		SYSTIM now;
		uint32_t timestamp;
		bool was_tilted = imu.tilt_detected;
		bool tilt_latched;
		bool report_startup = startup_samples < IMU_LIVE_STARTUP_SAMPLES;

		error = lsm303dlhc_i2c_read(&sensor, &sample, &timestamp);
		if (error != E_OK) report_error_forever("IMU", "vehicle reader", error);
		error = tk_get_otm(&now);
		if (error != E_OK) report_error_forever("IMU", "clock", error);
		/* No odometry in this diagnostic; tilt detection does not use distance. */
		if (!imu_processor_update(&imu, &sample, 0.0f, now.lo))
			report_error_forever("IMU", "tilt processor update", E_SYS);
		tilt_latched = !was_tilted && imu.tilt_detected;
		if (tilt_latched)
			tm_printf((UB*)"[HWTEST][IMU] TILT DETECTED (latched until reset)\n");
		if (report_startup || tilt_latched ||
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
