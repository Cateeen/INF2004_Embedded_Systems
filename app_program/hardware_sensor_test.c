#include "app.h"
#include "hardware_sensor_test.h"

#include "subsystem6/gpio/gpio_dispatcher.h"
#include "subsystem4/imu/lsm303dlhc_i2c.h"
#include "subsystem3/line_barcode/line_adc_reader.h"
#include "subsystem5/obstacle/obs_pico_port.h"
#include "subsystem5/obstacle/obs_task.h"
#include "vehicle_buffers.h"

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
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
#elif APP_SENSOR_TEST == 2
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

static void sensor_test_task(INT stacd, void *exinf)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t config;
	unsigned probe_count;
	ER error;

	(void)stacd;
	(void)exinf;
	error = lsm303dlhc_i2c_default_config(&config, 20U);
	if (error != E_OK) report_error_forever("IMU", "default config", error);
	config.profile = LSM303DLHC_I2C_PROFILE_32_FAMILY;
	error = lsm303dlhc_i2c_init(&sensor, &config);
	if (error != E_OK) report_error_forever("IMU", "sensor init", error);
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
	tm_printf((UB*)"[HWTEST][IMU] vehicle reader: CTRL1=0x2f CTRL4=0x80; "
		  "data-ready gating enabled; sample time is a lower bound\n");
	for (;;) {
		accel_data_t sample;
		SYSTIM now;
		uint32_t timestamp;

		error = lsm303dlhc_i2c_read(&sensor, &sample, &timestamp);
		if (error != E_OK) report_error_forever("IMU", "vehicle reader", error);
		error = tk_get_otm(&now);
		if (error != E_OK) report_error_forever("IMU", "clock", error);
		tm_printf((UB*)"[HWTEST][IMU] vehicle t_lower=%u ms age_bound=%u ms "
			  "raw_x=%d raw_y=%d raw_z=%d ax_mg=%ld ay_mg=%ld az_mg=%ld\n",
			  (unsigned)timestamp, (unsigned)(now.lo - timestamp),
			  (int)sample.raw_x, (int)sample.raw_y, (int)sample.raw_z,
			  (long)(sample.ax * 1000.0f / 9.80665f),
			  (long)(sample.ay * 1000.0f / 9.80665f),
			  (long)(sample.az * 1000.0f / 9.80665f));
		error = tk_dly_tsk(50U);
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
#error APP_SENSOR_TEST must be 0, 1 (IR), 2 (IMU), 3 (ultrasonic), or 4 (servo)
#endif

ER hardware_sensor_test_start(void)
{
	ID task = app_start_task((FP)sensor_test_task, SENSOR_TEST_PRIORITY,
				 TP_PRC1, "hardware_sensor_test");

	return task < E_OK ? (ER)task : E_OK;
}

#endif
