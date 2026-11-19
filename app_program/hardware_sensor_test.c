#include "app.h"
#include "hardware_sensor_test.h"

#include "gpio/gpio_dispatcher.h"
#include "imu/lsm303dlhc_i2c.h"
#include "line_barcode/line_adc_reader.h"
#include "obstacle/obs_pico_port.h"
#include "obstacle/obs_task.h"
#include "vehicle_buffers.h"

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
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
		if (error != E_OK) report_error_forever("IMU", "read", error);
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
#if SENSOR_TEST_ECHO_3V3_SAFE
	hardware.echo_voltage_verified_3v3_safe = true;
#endif
#if SENSOR_TEST_SERVO_LIMITS_VERIFIED
	hardware.servo_limits_verified = true;
	hardware.servo_min_us = 1000U;
	hardware.servo_max_us = 2000U;
#endif
	if (!hardware.echo_voltage_verified_3v3_safe) {
		tm_printf((UB*)"[HWTEST][ULTRASONIC] blocked: verify GP4 Echo is <=3.3V, "
			  "then build with SENSOR_TEST_ECHO_3V3_SAFE=1\n");
		report_error_forever("ULTRASONIC", "safety check", E_PAR);
	}
	if (!hardware.servo_limits_verified) {
		tm_printf((UB*)"[HWTEST][ULTRASONIC] blocked: verify servo endpoints, "
			  "then build with SENSOR_TEST_SERVO_LIMITS_VERIFIED=1\n");
		report_error_forever("ULTRASONIC", "safety check", E_PAR);
	}

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
#else
#error APP_SENSOR_TEST must be 0, 1 (IR), 2 (IMU), or 3 (ultrasonic)
#endif

ER hardware_sensor_test_start(void)
{
	ID task = app_start_task((FP)sensor_test_task, SENSOR_TEST_PRIORITY,
				 TP_PRC1, "hardware_sensor_test");

	return task < E_OK ? (ER)task : E_OK;
}

#endif
