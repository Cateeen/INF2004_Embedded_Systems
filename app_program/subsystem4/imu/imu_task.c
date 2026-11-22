#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "imu_producer.h"
#include "imu_task.h"

#define IMU_TASK_PRIORITY 10
#define IMU_FAULT_RETRY_MS 1000U

typedef struct {
	imu_producer_t producer;
	wheel_odometry_message_t pending;
	imu_level_calibrator_t level;
	uint32_t start_ms;
	uint32_t last_sample_ms;
	ER delivery_error;
	bool have_pending;
	bool have_sample;
	/* Startup level calibration still running: estimates are sent (so the
	 * controller's IMU deadline holds) but marked tilted, so it cannot arm. */
	bool level_pending;
} imu_task_state_t;

static imu_task_config_t s_config;
static imu_task_state_t s_state;
static bool s_started;

static uint16_t receive_odometry(imu_task_state_t *state, uint32_t sample_ms)
{
	unsigned count;

	/* One retained message plus at most one buffer capacity per cycle. */
	for (count = 0U; count < VEHICLE_MB_ODOMETRY_CAPACITY + 1U; count++) {
		wheel_odometry_message_t message;
		uint32_t ahead;
		INT received;

		if (state->have_pending) {
			message = state->pending;
		} else {
			received = tk_rcv_mbf(g_mbf_imu_odometry, &message, TMO_POL);
			if (received == E_TMOUT) break;
			if (received != (INT)sizeof(message))
				return VEHICLE_FAULT_CODE_IMU_ODOMETRY;
		}
		if (message.header.type != VEHICLE_MESSAGE_WHEEL_ODOMETRY ||
		    message.header.version != VEHICLE_MESSAGE_VERSION || message.valid != 1U)
			return VEHICLE_FAULT_CODE_IMU_ODOMETRY;
		ahead = message.header.timestamp_ms - sample_ms;
		if (ahead != 0U && ahead <= INT32_MAX) {
			state->pending = message;
			state->have_pending = true;
			break;
		}
		state->have_pending = false;
		if (sample_ms - message.header.timestamp_ms >
		    state->producer.max_odometry_age_ms ||
		    !imu_producer_add_odometry(&state->producer, &message))
			return VEHICLE_FAULT_CODE_IMU_ODOMETRY;
	}
	if (state->producer.have_odometry) {
		if (sample_ms - state->producer.odometry.header.timestamp_ms >
		    state->producer.max_odometry_age_ms)
			return VEHICLE_FAULT_CODE_IMU_ODOMETRY;
	} else if (sample_ms - state->start_ms > state->producer.max_odometry_age_ms) {
		return VEHICLE_FAULT_CODE_IMU_ODOMETRY;
	}
	return 0U;
}

/* Collect still samples at startup; once enough are in, restart the
 * processor with the level offsets so nothing seen while uncalibrated (a
 * false hump from the mounting tilt) carries over. A moving or badly tilted
 * robot restarts the collection; the car stays blocked meanwhile. */
static uint16_t update_level(imu_task_state_t *state, const accel_data_t *sample)
{
	imu_state_t calibrated;
	imu_level_result_t result;

	if (!imu_level_calibrator_add(&state->level, sample))
		return VEHICLE_FAULT_CODE_IMU_SENSOR;
	if (!imu_level_calibrator_full(&state->level)) return 0U;
	if (!imu_processor_init_vehicle(&calibrated))
		return VEHICLE_FAULT_CODE_IMU_PROCESSING;
	result = imu_processor_apply_level(&calibrated, &state->level);
	imu_level_calibrator_reset(&state->level);
	if (result != IMU_LEVEL_OK) {
		tm_printf((UB*)"IMU level calibration %s; keep the robot still on level "
			  "ground, retrying\n",
			  result == IMU_LEVEL_MOVING ? "saw movement" : "found it not level");
		return 0U;
	}
	state->producer.processor = calibrated;
	state->level_pending = false;
	tm_printf((UB*)"IMU level calibrated: pitch_mdeg=%ld roll_mdeg=%ld\n",
		  (long)(calibrated.pitch_offset_deg * 1000.0f),
		  (long)(calibrated.roll_offset_deg * 1000.0f));
	return 0U;
}

static uint16_t process_sample(imu_task_state_t *state, const accel_data_t *sample,
			       uint32_t sample_ms)
{
	imu_estimate_message_t estimate;
	uint16_t fault;
	ER er;
	uint32_t elapsed;

	if (!sample->i2c_ok || !__builtin_isfinite(sample->ax) ||
	    !__builtin_isfinite(sample->ay) || !__builtin_isfinite(sample->az))
		return VEHICLE_FAULT_CODE_IMU_SENSOR;
	elapsed = sample_ms - (state->have_sample ? state->last_sample_ms : state->start_ms);
	if (elapsed > INT32_MAX || (state->have_sample && elapsed == 0U))
		return VEHICLE_FAULT_CODE_IMU_CLOCK;
	fault = receive_odometry(state, sample_ms);
	if (fault != 0U) return fault;
	if (state->producer.have_odometry) {
		bool calibrating = state->level_pending;

		if (!imu_producer_update(&state->producer, sample, NULL, sample_ms, &estimate))
			return VEHICLE_FAULT_CODE_IMU_PROCESSING;
		if (calibrating) {
			estimate.tilt_detected = 1U; /* not safe to arm yet */
			fault = update_level(state, sample);
			if (fault != 0U) return fault;
		}
		er = tk_snd_mbf(g_mbf_imu_estimates, &estimate, (INT)sizeof(estimate), TMO_POL);
		if (er != E_OK) {
			state->delivery_error = er;
			return VEHICLE_FAULT_CODE_IMU_DELIVERY;
		}
	}
	state->last_sample_ms = sample_ms;
	state->have_sample = true;
	return 0U;
}

static void report_imu_fault(uint16_t code, uint32_t timestamp_ms)
{
	fault_message_t fault = {0};
	bool sent = false;

	fault.header.type = VEHICLE_MESSAGE_FAULT;
	fault.header.version = VEHICLE_MESSAGE_VERSION;
	fault.header.sequence = 1U;
	fault.header.timestamp_ms = timestamp_ms;
	fault.source = code == VEHICLE_FAULT_CODE_IMU_DELIVERY ||
		       code == VEHICLE_FAULT_CODE_IMU_ODOMETRY ?
		       VEHICLE_FAULT_COMMUNICATION :
		       (code == VEHICLE_FAULT_CODE_IMU_CLOCK ||
			code == VEHICLE_FAULT_CODE_IMU_SCHEDULE ?
			VEHICLE_FAULT_SYSTEM : VEHICLE_FAULT_SENSOR);
	fault.code = code;
	fault.recoverable = 0U;
	tm_printf((UB*)"IMU stopped; fault code %u\n", (unsigned)code);
	for (;;) {
		ER er;
		if (!sent) {
			er = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
			if (er == E_OK) sent = true;
			else tm_printf((UB*)"IMU fault delivery failed: %d\n", (int)er);
		}
		er = tk_dly_tsk(IMU_FAULT_RETRY_MS);
		if (er < E_OK) {
			tm_printf((UB*)"IMU fault retry delay failed: %d\n", (int)er);
			er = tk_slp_tsk(TMO_FEVR);
			tm_printf((UB*)"IMU fault sleep returned: %d\n", (int)er);
		}
	}
}

static void imu_task(INT stacd, void *exinf)
{
	SYSTIM now;
	uint32_t timestamp_ms = 0U;
	ER er;

	(void)stacd;
	(void)exinf;
	er = tk_get_otm(&now);
	if (er != E_OK) report_imu_fault(VEHICLE_FAULT_CODE_IMU_CLOCK, timestamp_ms);
	timestamp_ms = now.lo;
	s_state.start_ms = timestamp_ms;
	if (s_config.initialize != NULL) {
		er = s_config.initialize(s_config.read_context);
		if (er != E_OK) {
			tm_printf((UB*)"IMU sensor initialization failed: %d\n", (int)er);
			report_imu_fault(VEHICLE_FAULT_CODE_IMU_SENSOR, timestamp_ms);
		}
	}
	tm_printf((UB*)"IMU level calibration: keep the robot still on level ground "
		  "for about 1 s; GO is blocked until it finishes\n");
	for (;;) {
		accel_data_t sample;
		uint32_t sample_ms;
		uint16_t fault;

		er = s_config.read(s_config.read_context, &sample, &sample_ms);
		if (er != E_OK) {
			tm_printf((UB*)"IMU sensor read failed: %d\n", (int)er);
			report_imu_fault(VEHICLE_FAULT_CODE_IMU_SENSOR, timestamp_ms);
		}
		er = tk_get_otm(&now);
		if (er != E_OK) report_imu_fault(VEHICLE_FAULT_CODE_IMU_CLOCK, timestamp_ms);
		timestamp_ms = now.lo;
		if (timestamp_ms - sample_ms > s_config.sample_max_age_ms)
			report_imu_fault(VEHICLE_FAULT_CODE_IMU_SENSOR, timestamp_ms);
		fault = process_sample(&s_state, &sample, sample_ms);
		if (fault != 0U) {
			if (fault == VEHICLE_FAULT_CODE_IMU_DELIVERY)
				tm_printf((UB*)"IMU delivery failed: %d\n", (int)s_state.delivery_error);
			report_imu_fault(fault, timestamp_ms);
		}
		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) report_imu_fault(VEHICLE_FAULT_CODE_IMU_SCHEDULE, timestamp_ms);
	}
}

ER imu_task_start(const imu_task_config_t *config)
{
	T_CTSK task = {0};
	imu_task_state_t initial = {0};
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->read == NULL || config->period_ms == 0U ||
	    config->period_ms > INT32_MAX || config->sample_max_age_ms == 0U ||
	    config->sample_max_age_ms > INT32_MAX ||
	    !imu_producer_init(&initial.producer, config->odometry_max_age_ms) ||
	    !imu_processor_init_vehicle(&initial.producer.processor))
		return E_PAR;
	imu_level_calibrator_reset(&initial.level);
	initial.level_pending = true;
	if (g_mbf_imu_odometry <= 0 || g_mbf_imu_estimates <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	s_config = *config;
	s_state = initial;
	task.task = (FP)imu_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = IMU_TASK_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK) tm_printf((UB*)"IMU task cleanup failed: %d\n", (int)cleanup);
		else s_started = false;
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

#if APP_TEST
#include "../../test_report.h"
#include "imu_processor_config.h"

static BOOL drain_test_buffer(ID buffer)
{
	union {
		wheel_odometry_message_t odometry;
		imu_estimate_message_t estimate;
	} message;
	INT result;

	do {
		result = tk_rcv_mbf(buffer, &message, TMO_POL);
	} while (result > 0);
	return result == E_TMOUT;
}

static wheel_odometry_message_t test_odometry(uint32_t sequence, uint32_t timestamp)
{
	wheel_odometry_message_t message = {0};
	message.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = sequence;
	message.header.timestamp_ms = timestamp;
	message.valid = 1U;
	return message;
}

static BOOL task_flow_test(void)
{
	imu_task_state_t state = {0};
	accel_data_t sample = {.az = 9.80665f, .i2c_ok = true};
	wheel_odometry_message_t odometry;
	imu_estimate_message_t estimate;
	BOOL ok;

	ok = imu_producer_init(&state.producer, 10U);
	state.start_ms = 100U;
	ok = process_sample(&state, &sample, 100U) == 0U &&
	     tk_rcv_mbf(g_mbf_imu_estimates, &estimate, TMO_POL) == E_TMOUT && ok;
	odometry = test_odometry(UINT32_MAX, 102U);
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	odometry = test_odometry(0U, 103U);
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_sample(&state, &sample, 101U) == 0U && state.have_pending &&
	     !state.producer.have_odometry && ok;
	ok = process_sample(&state, &sample, 103U) == 0U && !state.have_pending &&
	     state.producer.odometry.header.sequence == 0U && ok;
	ok = tk_rcv_mbf(g_mbf_imu_estimates, &estimate, TMO_POL) == (INT)sizeof(estimate) &&
	     estimate.header.type == VEHICLE_MESSAGE_IMU_ESTIMATE &&
	     estimate.header.sequence == 1U && estimate.header.timestamp_ms == 103U &&
	     estimate.validity == VEHICLE_IMU_VALID_ROLL_PITCH && ok;
	ok = process_sample(&state, &sample, 104U) == 0U &&
	     tk_rcv_mbf(g_mbf_imu_estimates, &estimate, TMO_POL) == (INT)sizeof(estimate) &&
	     estimate.header.sequence == 2U && ok;
	odometry = test_odometry(2U, 105U);
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_sample(&state, &sample, 105U) == VEHICLE_FAULT_CODE_IMU_ODOMETRY && ok;
	ok = drain_test_buffer(g_mbf_imu_odometry) && ok;
	ok = drain_test_buffer(g_mbf_imu_estimates) && ok;
	state = (imu_task_state_t){0};
	ok = imu_producer_init(&state.producer, 10U) && ok;
	state.start_ms = UINT32_MAX - 2U;
	odometry = test_odometry(UINT32_MAX, UINT32_MAX - 1U);
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_sample(&state, &sample, UINT32_MAX) == 0U &&
	     tk_rcv_mbf(g_mbf_imu_estimates, &estimate, TMO_POL) == (INT)sizeof(estimate) && ok;
	odometry = test_odometry(0U, 1U);
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_sample(&state, &sample, 2U) == 0U &&
	     tk_rcv_mbf(g_mbf_imu_estimates, &estimate, TMO_POL) == (INT)sizeof(estimate) &&
	     estimate.header.timestamp_ms == 2U && ok;
	ok = drain_test_buffer(g_mbf_imu_odometry) && ok;
	ok = drain_test_buffer(g_mbf_imu_estimates) && ok;
	return ok;
}

static BOOL task_fault_test(void)
{
	imu_task_state_t state = {0};
	accel_data_t sample = {.az = 9.80665f, .i2c_ok = true};
	wheel_odometry_message_t odometry = test_odometry(1U, 100U);
	BOOL ok;

	ok = imu_producer_init(&state.producer, 10U);
	state.start_ms = 100U;
	ok = process_sample(&state, &sample, 111U) == VEHICLE_FAULT_CODE_IMU_ODOMETRY && ok;
	sample.i2c_ok = false;
	ok = process_sample(&state, &sample, 100U) == VEHICLE_FAULT_CODE_IMU_SENSOR && ok;
	sample.i2c_ok = true;
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_sample(&state, &sample, 100U) == 0U && ok;
	ok = process_sample(&state, &sample, 100U) == VEHICLE_FAULT_CODE_IMU_CLOCK && ok;
	ok = process_sample(&state, &sample, 101U) == 0U && ok;
	ok = process_sample(&state, &sample, 102U) == VEHICLE_FAULT_CODE_IMU_DELIVERY &&
	     state.delivery_error == E_TMOUT && ok;
	ok = process_sample(&state, &sample, 111U) == VEHICLE_FAULT_CODE_IMU_ODOMETRY && ok;
	ok = drain_test_buffer(g_mbf_imu_odometry) && ok;
	ok = drain_test_buffer(g_mbf_imu_estimates) && ok;
	state = (imu_task_state_t){0};
	ok = imu_producer_init(&state.producer, 10U) && ok;
	state.start_ms = 100U;
	odometry.valid = 0U;
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;
	ok = process_sample(&state, &sample, 100U) == VEHICLE_FAULT_CODE_IMU_ODOMETRY && ok;
	sample.ax = __builtin_huge_valf();
	ok = process_sample(&state, &sample, 100U) == VEHICLE_FAULT_CODE_IMU_SENSOR && ok;
	ok = drain_test_buffer(g_mbf_imu_odometry) && ok;
	ok = drain_test_buffer(g_mbf_imu_estimates) && ok;
	return ok;
}

/* Feed one sample, then take the estimate it produced. */
static BOOL level_step(imu_task_state_t *state, const accel_data_t *sample,
		       uint32_t sample_ms, imu_estimate_message_t *estimate)
{
	return process_sample(state, sample, sample_ms) == 0U &&
	       tk_rcv_mbf(g_mbf_imu_estimates, estimate, TMO_POL) == (INT)sizeof(*estimate);
}

static BOOL task_level_test(void)
{
	imu_task_state_t state = {0};
	/* Board tilted 8.6 deg nose-up on the chassis, as measured on the robot. */
	accel_data_t mounted = {.ax = -1.466f, .az = 9.696f, .i2c_ok = true};
	accel_data_t bumped = mounted;
	wheel_odometry_message_t odometry = test_odometry(0U, 100U);
	imu_estimate_message_t estimate;
	uint32_t t = 100U;
	unsigned i;
	BOOL ok;

	ok = imu_producer_init(&state.producer, 60000U) &&
	     imu_processor_init_vehicle(&state.producer.processor);
	imu_level_calibrator_reset(&state.level);
	state.level_pending = true;
	state.start_ms = t;
	ok = tk_snd_mbf(g_mbf_imu_odometry, &odometry, (INT)sizeof(odometry), TMO_POL) == E_OK && ok;

	/* Movement during calibration restarts it; the car stays blocked. */
	bumped.ax += 1.0f;
	for (i = 0U; i < IMU_LEVEL_SAMPLES; i++) {
		ok = level_step(&state, (i == 5U) ? &bumped : &mounted, t, &estimate) &&
		     estimate.tilt_detected == 1U && ok;
		t += 50U;
	}
	ok = state.level_pending && state.level.count == 0U && ok;

	/* Still samples: blocked until the last one, then calibrated. */
	for (i = 0U; i < IMU_LEVEL_SAMPLES; i++) {
		ok = level_step(&state, &mounted, t, &estimate) &&
		     estimate.tilt_detected == 1U && ok;
		t += 50U;
	}
	ok = !state.level_pending &&
	     state.producer.processor.pitch_offset_deg > 8.5f &&
	     state.producer.processor.pitch_offset_deg < 8.7f && ok;

	/* Calibrated: the mounting tilt reads as flat, with no hump or tilt. */
	ok = level_step(&state, &mounted, t, &estimate) &&
	     estimate.tilt_detected == 0U && !state.producer.processor.hump_active &&
	     estimate.pitch_mdeg > -100 && estimate.pitch_mdeg < 100 && ok;
	ok = drain_test_buffer(g_mbf_imu_odometry) && ok;
	ok = drain_test_buffer(g_mbf_imu_estimates) && ok;
	return ok;
}

BOOL imu_task_test(void)
{
	BOOL flow_ok;
	BOOL fault_ok;
	BOOL level_ok;
	TEST_INIT();

	flow_ok = task_flow_test();
	fault_ok = task_fault_test();
	level_ok = task_level_test();
	TEST_CHECK("IMU-U05", flow_ok, "task odometry and estimate delivery");
	TEST_CHECK("IMU-U06", fault_ok, "task sensor, timing and delivery faults");
	TEST_CHECK("IMU-U12", level_ok, "startup level calibration blocks arming until still");
	TEST_SUMMARY("IMU task");
	return flow_ok && fault_ok && level_ok;
}
#endif
