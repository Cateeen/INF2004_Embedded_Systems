#include "lsm303dlhc_i2c.h"

#include "../../device/include/dev_i2c.h"

#define LSM303DLHC_WHO_AM_I       0x0FU
#define LSM303DLHC_CTRL_REG1_A    0x20U
#define LSM303DLHC_CTRL_REG4_A    0x23U
#define LSM303DLHC_STATUS_REG_A   0x27U
#define LSM303DLHC_OUT_X_L_A      0x28U
#define LSM303DLHC_AUTO_INCREMENT 0x80U
#define LSM303DLHC_WHO_AM_I_VALUE 0x33U
#define LSM303DLHC_CTRL1_100HZ_XYZ 0x57U
#define LSM303DLHC_CTRL4_NORMAL_2G 0x00U
#define LSM303DLHC_ZYXDA          0x08U
#define LSM303DLHC_MAX_READY_WAIT_MS 100U

static lsm303dlhc_i2c_t s_task_sensor;
static lsm303dlhc_i2c_config_t s_task_sensor_config;
static bool s_task_start_attempted;

static ER initialize_task_sensor(void *context)
{
	return lsm303dlhc_i2c_init(context, &s_task_sensor_config);
}

static ER read_fresh(void *context, lsm303dlhc_transfer_t *transfer);

static ER read_registers(lsm303dlhc_i2c_t *sensor, uint8_t reg,
			 uint8_t *data, uint8_t length)
{
	uint8_t address = reg;
	T_I2C_EXEC exec;
	SZ actual = 0;
	ER er;

	exec.sadr = sensor->address;
	exec.snd_size = 1;
	exec.snd_data = &address;
	exec.rcv_size = length;
	exec.rcv_data = data;
	er = tk_swri_dev(sensor->device, TDN_I2C_EXEC, &exec, sizeof(exec), &actual);
	if (er != E_OK) return er;
	return actual == (SZ)sizeof(exec) ? E_OK : E_IO;
}

static ER write_register(lsm303dlhc_i2c_t *sensor, uint8_t reg, uint8_t value)
{
	uint8_t data[2] = {reg, value};
	SZ actual = 0;
	ER er = tk_swri_dev(sensor->device, (W)sensor->address, data, sizeof(data), &actual);

	if (er != E_OK) return er;
	return actual == (SZ)sizeof(data) ? E_OK : E_IO;
}

ER lsm303dlhc_i2c_default_config(lsm303dlhc_i2c_config_t *config,
				 uint32_t data_ready_timeout_ms)
{
	lsm303dlhc_i2c_config_t imported = {
		.address = 0x19U,
		.data_ready_timeout_ms = data_ready_timeout_ms,
		.mapping = {{0U, 1U, 2U}, {-1, 1, -1}}
	};

	const uint8_t zeroes[6] = {0};
	accel_data_t decoded;

	if (config == NULL || data_ready_timeout_ms == 0U ||
	    data_ready_timeout_ms > LSM303DLHC_MAX_READY_WAIT_MS ||
	    !lsm303dlhc_accel_decode(zeroes, &imported.mapping, &decoded))
		return E_PAR;
	*config = imported;
	return E_OK;
}

static ER fail_init(lsm303dlhc_i2c_t *sensor, ER cause)
{
	ER stop_result = E_OK;
	ER close_result;

	if (sensor->measurement_may_be_active) {
		stop_result = write_register(sensor, LSM303DLHC_CTRL_REG1_A, 0U);
		if (stop_result == E_OK) sensor->measurement_may_be_active = false;
	}
	close_result = tk_cls_dev(sensor->device, 0U);
	if (close_result == E_OK) {
		sensor->device = 0;
		sensor->opened = false;
	}
	sensor->faulted = true;
	if (stop_result != E_OK) return stop_result;
	return close_result != E_OK ? close_result : cause;
}

ER lsm303dlhc_i2c_init(lsm303dlhc_i2c_t *sensor,
		       const lsm303dlhc_i2c_config_t *config)
{
	uint8_t identity, control1, control4;
	const uint8_t zeroes[6] = {0};
	accel_data_t decoded;
	SYSTIM now;
	ER er;

	if (sensor == NULL || config == NULL ||
	    (config->address != 0x18U && config->address != 0x19U) ||
	    config->data_ready_timeout_ms == 0U ||
	    config->data_ready_timeout_ms > LSM303DLHC_MAX_READY_WAIT_MS ||
	    !lsm303dlhc_accel_decode(zeroes, &config->mapping, &decoded))
		return E_PAR;
	if (sensor->opened || sensor->initialized || sensor->faulted ||
	    sensor->reader.initialized)
		return E_OBJ;
	sensor->device = tk_opn_dev((const UB *)"iicb", TD_UPDATE | TD_EXCL);
	if (sensor->device < E_OK) return sensor->device;
	sensor->opened = true;
	sensor->address = config->address;
	sensor->data_ready_timeout_ms = config->data_ready_timeout_ms;
	er = read_registers(sensor, LSM303DLHC_WHO_AM_I, &identity, 1U);
	if (er != E_OK) return fail_init(sensor, er);
	if (identity != LSM303DLHC_WHO_AM_I_VALUE) return fail_init(sensor, E_IO);
	er = write_register(sensor, LSM303DLHC_CTRL_REG1_A, 0U);
	if (er != E_OK) return fail_init(sensor, er);
	er = write_register(sensor, LSM303DLHC_CTRL_REG4_A, LSM303DLHC_CTRL4_NORMAL_2G);
	if (er != E_OK) return fail_init(sensor, er);
	er = tk_get_otm(&now);
	if (er != E_OK) return fail_init(sensor, er);
	sensor->next_sample_not_before_ms = now.lo;
	sensor->measurement_may_be_active = true;
	er = write_register(sensor, LSM303DLHC_CTRL_REG1_A, LSM303DLHC_CTRL1_100HZ_XYZ);
	if (er != E_OK) return fail_init(sensor, er);
	er = read_registers(sensor, LSM303DLHC_CTRL_REG1_A, &control1, 1U);
	if (er != E_OK) return fail_init(sensor, er);
	er = read_registers(sensor, LSM303DLHC_CTRL_REG4_A, &control4, 1U);
	if (er != E_OK) return fail_init(sensor, er);
	if (control1 != LSM303DLHC_CTRL1_100HZ_XYZ ||
	    control4 != LSM303DLHC_CTRL4_NORMAL_2G)
		return fail_init(sensor, E_IO);
	er = lsm303dlhc_reader_init(&sensor->reader, &config->mapping,
				    read_fresh, sensor);
	if (er != E_OK) return fail_init(sensor, er);
	sensor->initialized = true;
	sensor->faulted = false;
	return E_OK;
}

ER lsm303dlhc_i2c_start_imu_task(const lsm303dlhc_i2c_config_t *sensor_config,
				const imu_task_config_t *task_config)
{
	imu_task_config_t task;
	ER er;

	if (sensor_config == NULL || task_config == NULL) return E_PAR;
	if (s_task_start_attempted) return E_OBJ;
	s_task_sensor_config = *sensor_config;
	task = *task_config;
	task.initialize = initialize_task_sensor;
	task.read = lsm303dlhc_i2c_read;
	task.read_context = &s_task_sensor;
	er = imu_task_start(&task);
	if (er == E_OK) s_task_start_attempted = true;
	return er;
}

static ER read_fresh(void *context, lsm303dlhc_transfer_t *transfer)
{
	lsm303dlhc_i2c_t *sensor = context;
	uint32_t wait_start, next_timestamp;
	SYSTIM now;
	uint8_t status;
	ER er;

	er = tk_get_otm(&now);
	if (er != E_OK) return er;
	wait_start = now.lo;
	for (;;) {
		er = read_registers(sensor, LSM303DLHC_STATUS_REG_A, &status, 1U);
		if (er != E_OK) return er;
		if (status & LSM303DLHC_ZYXDA) break;
		er = tk_get_otm(&now);
		if (er != E_OK) return er;
		if (now.lo - wait_start >= sensor->data_ready_timeout_ms) return E_TMOUT;
		er = tk_dly_tsk(1U);
		if (er != E_OK) return er;
	}
	er = tk_get_otm(&now);
	if (er != E_OK) return er;
	next_timestamp = now.lo;
	er = read_registers(sensor, LSM303DLHC_OUT_X_L_A | LSM303DLHC_AUTO_INCREMENT,
			    transfer->bytes, sizeof(transfer->bytes));
	if (er != E_OK) return er;
	transfer->length = sizeof(transfer->bytes);
	transfer->fresh = true;
	transfer->timestamp_valid = true;
	transfer->timestamp_ms = sensor->next_sample_not_before_ms;
	sensor->next_sample_not_before_ms = next_timestamp;
	return E_OK;
}

ER lsm303dlhc_i2c_read(void *context, accel_data_t *sample, uint32_t *timestamp_ms)
{
	lsm303dlhc_i2c_t *sensor = context;
	ER er;

	if (sensor == NULL || sample == NULL || timestamp_ms == NULL) return E_PAR;
	if (!sensor->initialized || sensor->faulted) return E_OBJ;
	er = lsm303dlhc_reader_read(&sensor->reader, sample, timestamp_ms);
	if (er != E_OK) sensor->faulted = true;
	return er;
}

ER lsm303dlhc_i2c_close(lsm303dlhc_i2c_t *sensor)
{
	ER er, power_down_result = E_OK;

	if (sensor == NULL) return E_PAR;
	if (!sensor->opened) return E_OBJ;
	if (sensor->measurement_may_be_active) {
		power_down_result = write_register(sensor, LSM303DLHC_CTRL_REG1_A, 0U);
		if (power_down_result == E_OK) {
			sensor->measurement_may_be_active = false;
		} else {
			sensor->faulted = true;
		}
	}
	er = tk_cls_dev(sensor->device, 0U);
	if (er != E_OK) return er;
	sensor->opened = false;
	sensor->initialized = false;
	sensor->device = 0;
	return power_down_result;
}
