#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <tm/tmonitor.h>

#define APP_SENSOR_TEST 2
#include "lsm303dlhc_i2c.h"
#include "dev_i2c.h"
#include "i2c_probe_trace.h"

static char last_log[256];

int tm_printf(const UB *format, ...)
{
	va_list args;
	va_list copy;
	int result;

	va_start(args, format);
	va_copy(copy, args);
	vsnprintf(last_log, sizeof(last_log), (const char *)format, copy);
	va_end(copy);
	result = vprintf((const char *)format, args);
	va_end(args);
	return result;
}

#define CHECK(condition) do { \
	if (!(condition)) { \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		return 0; \
	} \
} while (0)

static uint8_t registers[256];
uint32_t imu_test_timer_us;
static UW current_time;
static uint32_t timer_ticks_per_ms;
static unsigned close_count;
static unsigned open_count;
static unsigned config_writes;
static bool short_exec;
static bool identity_read_fails;
static bool status_read_fails;
static bool ready_after_delay;
static bool ready_after_burst;
static uint8_t mismatch_register;
static bool bdu_simulated, bdu_changed_held, bdu_changed_released;
static uint8_t bdu_low_reads[3];
static bool trace_read_fails, trace_without_interrupts;
static imu_task_config_t captured_task_config;

ER dev_i2c_probe_trace(UW unit, T_I2C_PROBE_TRACE *trace)
{
	if (trace_read_fails || unit != 1U || trace == NULL) return E_IO;
	*trace = (T_I2C_PROBE_TRACE){
		.start_us = 100U, .first_irq_us = 150U, .receive_command_us = 200U,
		.first_receive_irq_us = 250U, .last_receive_irq_us = 450U,
		.receive_resume_us = 460U, .stop_arm_us = 470U, .stop_pending = 1U,
		.last_irq_us = 500U, .end_us = 700U,
		.irq_count = 9U, .receive_irq_count = 7U
	};
	if (trace_without_interrupts) trace->irq_count = 0U;
	return E_OK;
}

ER imu_task_start(const imu_task_config_t *task_config)
{
	ER er;

	if (task_config == NULL) return E_PAR;
	captured_task_config = *task_config;
	if (captured_task_config.initialize != NULL) {
		er = captured_task_config.initialize(captured_task_config.read_context);
		if (er != E_OK) return er;
	}
	return E_OK;
}

ID tk_opn_dev(const UB *devnm, uint32_t omode)
{
	if (strcmp((const char *)devnm, "iicb") != 0 ||
	    omode != (TD_UPDATE | TD_EXCL))
		return E_PAR;
	open_count++;
	return 7;
}

ER tk_cls_dev(ID dd, uint32_t option)
{
	if (dd != 7 || option != 0U) return E_PAR;
	close_count++;
	return E_OK;
}

ER tk_swri_dev(ID dd, W start, const void *buf, SZ size, SZ *asize)
{
	if (dd != 7 || buf == NULL || asize == NULL) return E_PAR;
	if (start == TDN_I2C_EXEC) {
		T_I2C_EXEC *exec = (T_I2C_EXEC *)buf;
		uint8_t reg;
		SZ i;
		if (size != (SZ)sizeof(*exec) || exec->sadr > 0x7FU ||
		    exec->snd_size != 1 || exec->snd_data == NULL ||
		    exec->rcv_data == NULL)
			return E_PAR;
		reg = exec->snd_data[0];
		imu_test_timer_us += reg == 0xA7U ? 850U : 370U;
		if (reg == 0x0FU && identity_read_fails) return E_IO;
		if (reg == 0x27U && status_read_fails) return E_IO;
		if (((reg & 0x7FU) == 0x28U && exec->rcv_size == 6) ||
		    (reg == 0xA7U && exec->rcv_size == 7)) {
			for (i = 0; i < exec->rcv_size; i++)
				exec->rcv_data[i] = registers[(reg & 0x7FU) + i];
			registers[0x27] = ready_after_burst && reg == 0xA7U ? 0x0FU : 0U;
		} else if (exec->rcv_size == 1) {
			exec->rcv_data[0] = registers[reg & 0x7FU];
			if (bdu_simulated && (reg == 0x28U || reg == 0x2AU || reg == 0x2CU)) {
				unsigned axis = (reg - 0x28U) / 2U;
				unsigned read = bdu_low_reads[axis]++;
				exec->rcv_data[0] = (uint8_t)(0x20U + axis * 0x10U);
				if ((read == 1U && bdu_changed_held) ||
				    (read == 2U && bdu_changed_released))
					exec->rcv_data[0] ^= 0x10U;
			}
			if (reg == mismatch_register) exec->rcv_data[0] ^= 0x80U;
		} else {
			return E_PAR;
		}
		*asize = short_exec ? (SZ)sizeof(*exec) - 1 : (SZ)sizeof(*exec);
		return E_OK;
	}
	if (size != 2 || (start != 0x18 && start != 0x19)) return E_PAR;
	{
		const uint8_t *data = buf;
		registers[data[0]] = data[1];
		if (data[0] == 0x20U || data[0] == 0x23U) config_writes++;
	}
	*asize = size;
	return E_OK;
}

ER tk_get_otm(SYSTIM *pk_tim)
{
	if (pk_tim == NULL) return E_PAR;
	pk_tim->hi = 0U;
	pk_tim->lo = current_time;
	return E_OK;
}

ER tk_dly_tsk(RELTIM dlytim)
{
	current_time += dlytim;
	imu_test_timer_us += dlytim * timer_ticks_per_ms;
	if (ready_after_delay) registers[0x27] = 0x08U;
	return E_OK;
}

static void reset_fake(void)
{
	memset(registers, 0, sizeof(registers));
	imu_test_timer_us = 0U;
	current_time = 100U;
	timer_ticks_per_ms = 1000U;
	close_count = 0U;
	open_count = 0U;
	config_writes = 0U;
	short_exec = false;
	identity_read_fails = true;
	status_read_fails = false;
	ready_after_delay = false;
	ready_after_burst = false;
	mismatch_register = 0U;
	bdu_simulated = false;
	bdu_changed_held = false;
	bdu_changed_released = false;
	memset(bdu_low_reads, 0, sizeof(bdu_low_reads));
	trace_read_fails = false;
	trace_without_interrupts = false;
	registers[0x0F] = 0x33U;
	registers[0x27] = 0x08U;
	registers[0x28] = 0x00U;
	registers[0x29] = 0x10U;
	registers[0x2A] = 0x00U;
	registers[0x2B] = 0x00U;
	registers[0x2C] = 0x80U;
	registers[0x2D] = 0x3EU;
}

static lsm303dlhc_i2c_config_t config(void)
{
	lsm303dlhc_i2c_config_t result = {0};
	(void)lsm303dlhc_i2c_default_config(&result, 2U);
	return result;
}

static int test_fresh_sample(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();
	accel_data_t sample = {0};
	uint32_t timestamp = 0U;

	reset_fake();
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	CHECK(open_count == 1U && sensor.initialized && !sensor.faulted);
	CHECK(setup.address == 0x19U && setup.mapping.sign[0] == -1 &&
	      setup.mapping.sign[1] == 1 && setup.mapping.sign[2] == -1);
	CHECK(registers[0x20] == 0x57U && registers[0x23] == 0x00U);
	CHECK(config_writes == 3U);
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_OK);
	CHECK(sample.raw_x == -256 && sample.raw_y == 0 && sample.raw_z == -1000);
	CHECK(sample.ax < -2.63f && sample.ax > -2.65f &&
	      sample.az < -10.29f && sample.az > -10.31f);
	CHECK(sample.i2c_ok && timestamp == 100U);
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);
	CHECK(close_count == 1U && registers[0x20] == 0U);
	return 1;
}

static int test_without_identity_check(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();
	accel_data_t sample = {0};
	uint32_t timestamp = 0U;

	reset_fake();
	registers[0x0F] = 0x32U;
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	CHECK(sensor.opened && sensor.initialized && !sensor.faulted);
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_OK);
	CHECK(sample.raw_x == -256 && sample.raw_z == -1000 && timestamp == 100U);
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);
	CHECK(close_count == 1U);
	return 1;
}

static int test_short_transfer(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();

	reset_fake();
	short_exec = true;
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_IO);
	CHECK(close_count == 1U && !sensor.opened && sensor.faulted);
	CHECK(registers[0x20] == 0U);
	return 1;
}

static int test_not_ready_latches_without_output_change(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();
	accel_data_t sample = {.raw_x = 12, .raw_y = 34, .raw_z = 56};
	uint32_t timestamp = 77U;

	reset_fake();
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	registers[0x27] = 0U;
	last_log[0] = '\0';
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_TMOUT);
	CHECK(strstr(last_log, "data-ready timeout: status=0x00 elapsed=2 ms limit=2 ms") != NULL);
	CHECK(sample.raw_x == 12 && sample.raw_y == 34 && sample.raw_z == 56);
	CHECK(timestamp == 77U && sensor.faulted);
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_OBJ);
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);
	return 1;
}

static int test_task_start_wires_transport(void)
{
	lsm303dlhc_i2c_config_t setup = config();
	imu_task_config_t task_config = {
		.period_ms = 10U,
		.sample_max_age_ms = 30U,
		.odometry_max_age_ms = 50U
	};
	accel_data_t sample = {0};
	uint32_t timestamp = 0U;

	reset_fake();
	setup.profile = LSM303DLHC_I2C_PROFILE_32_FAMILY;
	CHECK(lsm303dlhc_i2c_start_imu_task(&setup, &task_config) == E_OK);
	CHECK(registers[0x20] == 0x2FU && registers[0x23] == 0x80U);
	CHECK(captured_task_config.period_ms == 10U);
	CHECK(captured_task_config.sample_max_age_ms == 30U);
	CHECK(captured_task_config.odometry_max_age_ms == 50U);
	CHECK(captured_task_config.initialize != NULL);
	CHECK(captured_task_config.read == lsm303dlhc_i2c_read);
	CHECK(captured_task_config.read_context != NULL);
	CHECK(captured_task_config.read(captured_task_config.read_context,
				       &sample, &timestamp) == E_OK);
	CHECK(sample.raw_x == -256 && sample.raw_z == -1000 && timestamp == 100U);
	CHECK(lsm303dlhc_i2c_close(captured_task_config.read_context) == E_OK);
	return 1;
}

static int test_32_family_profile(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();
	accel_data_t sample;
	uint32_t timestamp;
	uint8_t reg;

	reset_fake();
	setup.profile = LSM303DLHC_I2C_PROFILE_32_FAMILY;
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	CHECK(registers[0x20] == 0x2FU && registers[0x23] == 0x80U);
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_OK);
	CHECK(timestamp == 100U && sample.raw_z == -1000);
	ready_after_delay = true;
	CHECK(tk_dly_tsk(50U) == E_OK);
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_OK);
	CHECK(timestamp == 100U);
	ready_after_delay = false;
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_TMOUT);
	CHECK(sensor.faulted);
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);

	for (reg = 0x20U; reg <= 0x23U; reg += 3U) {
		reset_fake();
		sensor = (lsm303dlhc_i2c_t){0};
		mismatch_register = reg;
		CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_IO);
		CHECK(sensor.faulted && !sensor.opened && registers[0x20] == 0U);
	}
	reset_fake();
	sensor = (lsm303dlhc_i2c_t){0};
	setup.profile = (lsm303dlhc_i2c_profile_t)2;
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_PAR);
	CHECK(open_count == 0U);
	return 1;
}

#define APP_H
#define HARDWARE_SENSOR_TEST_H
#define GPIO_DISPATCHER_H
#define LINE_ADC_READER_H
#define OBS_PICO_PORT_H
#define OBS_TASK_H
#define VEHICLE_BUFFERS_H
#define TP_PRC1 1U

typedef int INT;
typedef void (*FP)(INT, void *);

ID app_start_task(FP entry, int priority, unsigned core, const char *name)
{
	(void)entry;
	(void)priority;
	(void)core;
	(void)name;
	return 1;
}

#include "../../../app_program/hardware_sensor_test.c"

static int test_trace_clock_comparison(void)
{
	reset_fake();
	CHECK(imu_check_trace_clock() == E_OK);
	CHECK(strstr(last_log, "kernel_ms=250 timer_ticks=250000") != NULL);
	timer_ticks_per_ms = 25U;
	CHECK(imu_check_trace_clock() == E_OK);
	CHECK(strstr(last_log, "kernel_ms=250 timer_ticks=6250") != NULL);
	return 1;
}

static int test_unverified_hardware_read(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();
	accel_data_t sample = {0};
	uint32_t burst_us, status_us;
	uint8_t status = 0xFFU, after = 0xFFU;
	T_I2C_PROBE_TRACE trace;

	reset_fake();
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	identity_read_fails = false;
	registers[0x0F] = 0x32U;
	imu_report_identity(&sensor);
	CHECK(strstr(last_log, "WHO_AM_I: 0x32") != NULL);
	identity_read_fails = true;
	imu_report_identity(&sensor);
	CHECK(strstr(last_log, "WHO_AM_I read failed") != NULL);
	CHECK(!sensor.faulted && sensor.initialized);
	CHECK(imu_set_probe_mode(&sensor) == E_OK);
	CHECK(registers[0x20] == 0x2FU && registers[0x23] == 0x80U);
	short_exec = true;
	CHECK(imu_set_probe_mode(&sensor) == E_IO);
	CHECK(strstr(last_log, "probe register 0x23 readback failed") != NULL);
	short_exec = false;
	mismatch_register = 0x23U;
	CHECK(imu_set_probe_mode(&sensor) == E_IO);
	CHECK(strstr(last_log, "probe register 0x23 readback mismatch") != NULL);
	mismatch_register = 0U;
	registers[0x27] = 0U;
	imu_test_timer_us = UINT32_MAX - 500U;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_OK);
	CHECK(status == 0U && after == 0U && sample.raw_x == -256 && sample.raw_z == -1000);
	CHECK(burst_us == 850U && status_us == 370U);
	registers[0x29] = 0x20U;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_OK);
	CHECK(sample.raw_x == -512);
	short_exec = true;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_IO);
	CHECK(sample.raw_x == -512);
	short_exec = false;
	status_read_fails = true;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_IO);
	CHECK(sample.raw_x == -512);
	status_read_fails = false;
	trace_read_fails = true;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_IO);
	CHECK(sample.raw_x == -512);
	trace_read_fails = false;
	trace_without_interrupts = true;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_IO);
	CHECK(sample.raw_x == -512);
	trace_without_interrupts = false;
	registers[0x28] = 1U;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_IO);
	CHECK(sample.raw_x == -512);
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);
	return 1;
}

static int test_hardware_freshness_probe(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();
	accel_data_t sample;
	uint32_t burst_us, status_us;
	uint8_t status, after;
	T_I2C_PROBE_TRACE trace;

	reset_fake();
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_OK);
	CHECK(status == 0x08U && after == 0U && registers[0x27] == 0U);
	ready_after_delay = true;
	CHECK(imu_check_freshness(&sensor, status, after) == E_OK);
	CHECK(strstr(last_log, "data-ready transition observed") != NULL);

	ready_after_burst = true;
	registers[0x27] = 0xFFU;
	CHECK(imu_read_unverified(&sensor, &setup.mapping, &sample, &status, &after,
				  &burst_us, &status_us, &trace) == E_OK);
	CHECK(status == 0xFFU && after == 0x0FU);
	CHECK(imu_check_freshness(&sensor, status, after) == E_OK);
	CHECK(strstr(last_log, "freshness inconclusive") != NULL);
	ready_after_burst = false;
	registers[0x27] = 0U;
	CHECK(imu_check_freshness(&sensor, 0U, 0U) == E_OK);
	CHECK(strstr(last_log, "freshness inconclusive") != NULL);

	ready_after_delay = false;
	CHECK(imu_check_freshness(&sensor, 0x08U, 0U) == E_TMOUT);
	CHECK(strstr(last_log, "freshness timeout") != NULL);
	status_read_fails = true;
	CHECK(imu_check_freshness(&sensor, 0x08U, 0U) == E_IO);
	status_read_fails = false;
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);
	return 1;
}

static int test_hardware_bdu_probe(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();

	reset_fake();
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_OK);
	bdu_simulated = true;
	bdu_changed_released = true;
	CHECK(imu_probe_bdu(&sensor) == E_OK);
	CHECK(strstr(last_log, "result=consistent-not-proof") != NULL);

	memset(bdu_low_reads, 0, sizeof(bdu_low_reads));
	bdu_changed_held = true;
	CHECK(imu_probe_bdu(&sensor) == E_OK);
	CHECK(strstr(last_log, "result=changed-while-held") != NULL);

	memset(bdu_low_reads, 0, sizeof(bdu_low_reads));
	bdu_changed_held = false;
	bdu_changed_released = false;
	CHECK(imu_probe_bdu(&sensor) == E_OK);
	CHECK(strstr(last_log, "result=inconclusive") != NULL);
	short_exec = true;
	CHECK(imu_probe_bdu(&sensor) == E_IO);
	short_exec = false;
	CHECK(lsm303dlhc_i2c_close(&sensor) == E_OK);
	return 1;
}

int main(void)
{
	lsm303dlhc_i2c_config_t defaults = {0};
	CHECK(lsm303dlhc_i2c_default_config(&defaults, 10U) == E_OK);
	CHECK(defaults.address == 0x19U && defaults.data_ready_timeout_ms == 10U);
	CHECK(defaults.profile == LSM303DLHC_I2C_PROFILE_DLHC);
	CHECK(defaults.mapping.axis[0] == 0U && defaults.mapping.axis[1] == 1U &&
	      defaults.mapping.axis[2] == 2U);
	CHECK(defaults.mapping.sign[0] == -1 && defaults.mapping.sign[1] == 1 &&
	      defaults.mapping.sign[2] == -1);
	CHECK(lsm303dlhc_i2c_default_config(&defaults, 0U) == E_PAR);
	if (!test_fresh_sample() || !test_without_identity_check() ||
	    !test_short_transfer() || !test_not_ready_latches_without_output_change() ||
	    !test_task_start_wires_transport() || !test_unverified_hardware_read() ||
	    !test_hardware_freshness_probe() || !test_hardware_bdu_probe() ||
	    !test_trace_clock_comparison() ||
	    !test_32_family_profile())
		return 1;
	puts("[TEST] IMU-I2C-U01 PASS");
	puts("[TEST] IMU-I2C-U02 PASS");
	puts("[TEST] IMU-I2C-U03 PASS");
	puts("[TEST] IMU-I2C-U04 PASS");
	puts("[TEST] IMU-I2C-U05 PASS");
	puts("[TEST] IMU-I2C-U06 PASS");
	puts("[TEST] IMU-I2C-U07 PASS");
	puts("[TEST] IMU-I2C-U08 PASS");
	puts("[TEST] IMU-I2C-U09 PASS");
	return 0;
}
