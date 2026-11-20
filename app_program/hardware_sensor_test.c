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

#if APP_SENSOR_TEST

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
static void imu_report_registers(const lsm303dlhc_i2c_t *sensor)
{
	static const uint8_t regs[] = {0x20U, 0x23U, 0x24U, 0x27U, 0x2EU, 0x2FU};
	unsigned i;

	for (i = 0U; i < sizeof(regs); i++) {
		uint8_t reg = regs[i];
		uint8_t value = 0U;
		T_I2C_EXEC exec = {sensor->address, 1, &reg, 1, &value};
		SZ actual = 0;
		ER error = tk_swri_dev(sensor->device, TDN_I2C_EXEC, &exec,
				       sizeof(exec), &actual);

		if (error != E_OK || actual != (SZ)sizeof(exec)) {
			tm_printf((UB*)"[HWTEST][IMU] diagnostic reg=0x%02x failed: %d size=%d\n",
				  (unsigned)reg, (int)error, (int)actual);
			continue;
		}
		tm_printf((UB*)"[HWTEST][IMU] diagnostic reg=0x%02x value=0x%02x\n",
			  (unsigned)reg, (unsigned)value);
	}
}

static void sensor_test_task(INT stacd, void *exinf)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t config;
	ER error;

	(void)stacd;
	(void)exinf;
	error = lsm303dlhc_i2c_default_config(&config, 20U);
	if (error != E_OK) report_error_forever("IMU", "default config", error);
	error = lsm303dlhc_i2c_init(&sensor, &config);
	if (error != E_OK) report_error_forever("IMU", "sensor init", error);

	tm_printf((UB*)"[HWTEST][IMU] live accelerometer XYZ readings\n");
	for (;;) {
		accel_data_t sample;
		uint32_t timestamp_ms;

		error = lsm303dlhc_i2c_read(&sensor, &sample, &timestamp_ms);
		if (error != E_OK) {
			imu_report_registers(&sensor);
			report_error_forever("IMU", "read", error);
		}
		tm_printf((UB*)"[HWTEST][IMU] t=%u ms raw_x=%d raw_y=%d raw_z=%d "
			  "ax_mg=%ld ay_mg=%ld az_mg=%ld\n",
			  (unsigned)timestamp_ms, (int)sample.raw_x, (int)sample.raw_y,
			  (int)sample.raw_z,
			  (long)(sample.ax * 1000.0f / 9.80665f),
			  (long)(sample.ay * 1000.0f / 9.80665f),
			  (long)(sample.az * 1000.0f / 9.80665f));
		error = tk_dly_tsk(SENSOR_TEST_INTERVAL_MS);
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
