#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <tm/tmonitor.h>

#include "lsm303dlhc_i2c.h"
#include "dev_i2c.h"

int tm_printf(const UB *format, ...)
{
	va_list args;
	int result;

	va_start(args, format);
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
static UW current_time;
static unsigned close_count;
static unsigned open_count;
static unsigned config_writes;
static bool short_exec;
static imu_task_config_t captured_task_config;

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
		if ((reg & 0x7FU) == 0x28U && exec->rcv_size == 6) {
			for (i = 0; i < 6; i++)
				exec->rcv_data[i] = registers[(reg & 0x7FU) + i];
			registers[0x27] = 0U;
		} else if (exec->rcv_size == 1) {
			exec->rcv_data[0] = registers[reg & 0x7FU];
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
	return E_OK;
}

static void reset_fake(void)
{
	memset(registers, 0, sizeof(registers));
	current_time = 100U;
	close_count = 0U;
	open_count = 0U;
	config_writes = 0U;
	short_exec = false;
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

static int test_identity_mismatch(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();

	reset_fake();
	registers[0x0F] = 0x32U;
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_IO);
	CHECK(close_count == 1U && !sensor.opened && sensor.faulted);
	CHECK(config_writes == 0U);
	return 1;
}

static int test_short_transfer(void)
{
	lsm303dlhc_i2c_t sensor = {0};
	lsm303dlhc_i2c_config_t setup = config();

	reset_fake();
	short_exec = true;
	CHECK(lsm303dlhc_i2c_init(&sensor, &setup) == E_IO);
	CHECK(close_count == 1U && !sensor.opened);
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
	CHECK(lsm303dlhc_i2c_read(&sensor, &sample, &timestamp) == E_TMOUT);
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
	CHECK(lsm303dlhc_i2c_start_imu_task(&setup, &task_config) == E_OK);
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

int main(void)
{
	lsm303dlhc_i2c_config_t defaults = {0};
	CHECK(lsm303dlhc_i2c_default_config(&defaults, 10U) == E_OK);
	CHECK(defaults.address == 0x19U && defaults.data_ready_timeout_ms == 10U);
	CHECK(defaults.mapping.axis[0] == 0U && defaults.mapping.axis[1] == 1U &&
	      defaults.mapping.axis[2] == 2U);
	CHECK(defaults.mapping.sign[0] == -1 && defaults.mapping.sign[1] == 1 &&
	      defaults.mapping.sign[2] == -1);
	CHECK(lsm303dlhc_i2c_default_config(&defaults, 0U) == E_PAR);
	if (!test_fresh_sample() || !test_identity_mismatch() ||
	    !test_short_transfer() || !test_not_ready_latches_without_output_change() ||
	    !test_task_start_wires_transport())
		return 1;
	puts("[TEST] IMU-I2C-U01 PASS");
	puts("[TEST] IMU-I2C-U02 PASS");
	puts("[TEST] IMU-I2C-U03 PASS");
	puts("[TEST] IMU-I2C-U04 PASS");
	return 0;
}
