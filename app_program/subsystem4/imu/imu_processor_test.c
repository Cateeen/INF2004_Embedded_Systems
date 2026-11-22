#include "../../app.h"
#include "imu_processor_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "imu_processor.h"
#include "imu_processor_config.h"
#include "imu_producer.h"
#include "lsm303dlhc_accel.h"
#include "lsm303dlhc_reader.h"

static BOOL terrain_test(void)
{
	imu_state_t state;
	const accel_data_t flat = {.az = GRAVITY_MS2};
	const accel_data_t ramp = {.ax = -1.4f, .az = 9.70f};

	return imu_processor_init(&state) &&
	       imu_processor_update(&state, &flat, 0.0f, 10U) &&
	       state.fsm_state == IMU_STATE_MONITORING &&
	       imu_processor_update(&state, &ramp, 0.0f, 20U) &&
	       state.fsm_state == IMU_STATE_HUMP_RISING &&
	       imu_processor_update(&state, &ramp, 0.30f, 50U) &&
	       imu_processor_update(&state, &flat, 0.30f, 80U) &&
	       state.fsm_state == IMU_STATE_HUMP_FALLING &&
	       state.hump_peak_m > 0.015f &&
	       imu_processor_update(&state, &flat, 0.30f, 100U) &&
	       state.fsm_state == IMU_STATE_MONITORING && !state.hump_active;
}

static BOOL safety_test(void)
{
	imu_state_t state;
	const accel_data_t flat = {.az = GRAVITY_MS2};
	const accel_data_t ramp = {.ax = -1.4f, .az = 9.70f};
	const accel_data_t lifted_roll = {.ay = 5.0f, .az = 8.4f};
	const accel_data_t overturned = {.az = -5.0f};
	const accel_data_t invalid = {.ax = __builtin_huge_valf(), .az = GRAVITY_MS2};

	return imu_processor_init(&state) &&
	       imu_processor_update(&state, &ramp, 0.0f, 5U) &&
	       state.hump_active &&
	       imu_processor_update(&state, &lifted_roll, 0.0f, 10U) &&
	       state.tilt_detected && !state.hump_active &&
	       state.fsm_state == IMU_STATE_TILT &&
	       (state.packed_telemetry & 7U) == IMU_STATE_TILT &&
	       imu_processor_update(&state, &flat, 0.0f, 20U) &&
	       state.fsm_state == IMU_STATE_MONITORING &&
	       imu_processor_update(&state, &overturned, 0.0f, 30U) &&
	       state.tilt_detected && state.fsm_state == IMU_STATE_TILT &&
	       (state.packed_telemetry & 7U) == IMU_STATE_TILT &&
	       !imu_processor_update(&state, &invalid, 0.0f, 40U) &&
	       state.fsm_state == IMU_STATE_TILT &&
	       !imu_processor_update(&state, NULL, 0.0f, 40U) &&
	       !imu_processor_init(NULL);
}

static BOOL pickup_tilt_test(void)
{
	imu_state_t state;
	const accel_data_t flat = {.az = GRAVITY_MS2};
	const accel_data_t ramp = {.ax = -1.4f, .az = 9.70f};
	const accel_data_t picked_up = {.ay = 2.8f, .az = 9.4f};
	const accel_data_t low_az = {.az = 6.4f};

	return imu_processor_init(&state) &&
	       imu_processor_update(&state, &flat, 0.0f, 10U) &&
	       !state.tilt_detected &&
	       imu_processor_update(&state, &ramp, 0.0f, 20U) &&
	       state.hump_active &&
	       imu_processor_update(&state, &picked_up, 0.0f, 30U) &&
	       state.fsm_state == IMU_STATE_TILT && state.tilt_detected &&
	       !state.hump_active &&
	       imu_processor_update(&state, &flat, 0.0f, 40U) &&
	       state.fsm_state == IMU_STATE_MONITORING && state.tilt_detected &&
	       imu_processor_init(&state) &&
	       imu_processor_update(&state, &low_az, 0.0f, 50U) &&
	       state.fsm_state == IMU_STATE_TILT && state.tilt_detected;
}

static BOOL hump_height_test(void)
{
	imu_state_t state;
	const accel_data_t flat = {.az = GRAVITY_MS2};
	const accel_data_t ramp_30_deg = {.ax = -4.903325f, .az = 8.492808f};

	if (!imu_processor_init(&state) ||
	    !imu_processor_update(&state, &ramp_30_deg, 0.0f, 10U) ||
	    state.fsm_state != IMU_STATE_HUMP_RISING ||
	    !imu_processor_update(&state, &ramp_30_deg, 0.10f, 20U) ||
	    state.current_hump_m < 0.049f || state.current_hump_m > 0.051f ||
	    !imu_processor_update(&state, &ramp_30_deg, 0.20f, 30U) ||
	    !imu_processor_update(&state, &flat, 0.20f, 40U) ||
	    state.fsm_state != IMU_STATE_HUMP_FALLING ||
	    state.hump_peak_m < 0.099f || state.hump_peak_m > 0.101f ||
	    ((state.packed_telemetry >> 6U) & 0x3FU) != 40U ||
	    !imu_processor_update(&state, &flat, 0.20f, 50U) ||
	    state.fsm_state != IMU_STATE_MONITORING)
		return false;

	return imu_processor_init(&state) &&
	       imu_processor_update(&state, &ramp_30_deg, 0.0f, 100U) &&
	       imu_processor_update(&state, &ramp_30_deg, 0.20f, 5101U) &&
	       state.fsm_state == IMU_STATE_MONITORING && !state.hump_active &&
	       state.current_hump_m == 0.0f && state.hump_peak_m == 0.0f;
}

static BOOL clock_test(void)
{
	imu_state_t first;
	imu_state_t second;
	const accel_data_t ramp = {.ax = -1.4f, .az = 9.70f};

	return imu_processor_init(&first) && imu_processor_init(&second) &&
	       imu_processor_update(&first, &ramp, 0.0f, 100U) &&
	       imu_processor_update(&second, &ramp, 0.0f, 4100U) &&
	       imu_processor_update(&first, &ramp, 0.0f, 5101U) &&
	       first.fsm_state == IMU_STATE_MONITORING &&
	       imu_processor_update(&second, &ramp, 0.0f, 5101U) &&
	       second.fsm_state == IMU_STATE_HUMP_RISING &&
	       imu_processor_init(&first) &&
	       imu_processor_update(&first, &ramp, 0.0f, UINT32_MAX - 1000U) &&
	       imu_processor_update(&first, &ramp, 0.0f, 4000U) &&
	       first.fsm_state == IMU_STATE_MONITORING;
}

static BOOL accel_decode_test(void)
{
	const uint8_t bytes[6] = {0x00U, 0x80U, 0xF0U, 0x7FU, 0x00U, 0x10U};
	uint8_t invalid[6] = {0};
	lsm303dlhc_axis_map_t mapping = {{2U, 0U, 1U}, {-1, 1, -1}};
	accel_data_t sample = {0};
	BOOL ok;

	ok = lsm303dlhc_accel_decode(bytes, &mapping, &sample) &&
	     sample.raw_x == -256 && sample.raw_y == -2048 &&
	     sample.raw_z == -2047 && !sample.i2c_ok &&
	     sample.ax < -2.63f && sample.ax > -2.65f &&
	     sample.ay < -21.08f && sample.ay > -21.10f;
	mapping.axis[1] = 2U;
	ok = !lsm303dlhc_accel_decode(bytes, &mapping, &sample) &&
	     sample.raw_x == -256 && ok;
	mapping.axis[1] = 0U;
	mapping.sign[2] = 0;
	ok = !lsm303dlhc_accel_decode(bytes, &mapping, &sample) &&
	     sample.raw_y == -2048 && ok;
	mapping.sign[2] = -1;
	invalid[0] = 1U;
	ok = !lsm303dlhc_accel_decode(invalid, &mapping, &sample) &&
	     !lsm303dlhc_accel_decode(NULL, &mapping, &sample) &&
	     sample.raw_z == -2047 && ok;
	mapping.axis[0] = 0U;
	mapping.axis[1] = 1U;
	mapping.axis[2] = 2U;
	mapping.sign[0] = -1;
	mapping.sign[1] = 1;
	mapping.sign[2] = 1;
	ok = lsm303dlhc_accel_decode(bytes, &mapping, &sample) &&
	     sample.raw_x == 2048 && sample.raw_y == 2047 && sample.raw_z == 256 && ok;
	return ok;
}

typedef struct {
	lsm303dlhc_transfer_t transfer;
	ER result;
	unsigned calls;
} test_accel_transport_t;

static ER test_read_fresh(void *context, lsm303dlhc_transfer_t *transfer)
{
	test_accel_transport_t *transport = context;
	transport->calls++;
	*transfer = transport->transfer;
	return transport->result;
}

static BOOL accel_reader_test(void)
{
	const lsm303dlhc_axis_map_t mapping = {{0U, 1U, 2U}, {1, -1, 1}};
	lsm303dlhc_axis_map_t invalid = {{0U, 0U, 2U}, {1, 1, 1}};
	lsm303dlhc_reader_t reader = {0};
	test_accel_transport_t transport = {0};
	accel_data_t sample = {0};
	uint32_t timestamp = 0U;
	BOOL ok;
	unsigned i;

	ok = lsm303dlhc_reader_init(&reader, &invalid, test_read_fresh, &transport) == E_PAR &&
	     lsm303dlhc_reader_init(&reader, &mapping, NULL, &transport) == E_PAR &&
	     lsm303dlhc_reader_init(&reader, &mapping, test_read_fresh, &transport) == E_OK &&
	     lsm303dlhc_reader_init(&reader, &mapping, test_read_fresh, &transport) == E_OBJ;
	transport.transfer.bytes[1] = 0x10U;
	transport.transfer.bytes[3] = 0x20U;
	transport.transfer.length = 6U;
	transport.transfer.fresh = true;
	transport.transfer.timestamp_valid = true;
	transport.transfer.timestamp_ms = UINT32_MAX;
	ok = lsm303dlhc_reader_read(&reader, &sample, &timestamp) == E_OK &&
	     sample.raw_x == 256 && sample.raw_y == -512 && sample.i2c_ok &&
	     timestamp == UINT32_MAX && transport.calls == 1U && ok;
	for (i = 0U; i < 6U; i++) {
		reader = (lsm303dlhc_reader_t){0};
		transport.transfer.length = 6U;
		transport.transfer.fresh = true;
		transport.transfer.timestamp_valid = true;
		transport.transfer.bytes[0] = 0U;
		transport.result = E_OK;
		if (i == 0U) transport.transfer.length = 5U;
		if (i == 1U) transport.transfer.fresh = false;
		if (i == 2U) transport.transfer.timestamp_valid = false;
		if (i == 3U) transport.transfer.bytes[0] = 1U;
		if (i == 4U) transport.result = E_TMOUT;
		if (i == 5U) transport.result = 1;
		ok = lsm303dlhc_reader_init(&reader, &mapping, test_read_fresh, &transport) == E_OK &&
		     ok;
		sample.raw_x = 77;
		timestamp = 42U;
		ok = lsm303dlhc_reader_read(&reader, &sample, &timestamp) ==
		     (i == 4U ? E_TMOUT : E_IO) &&
		     reader.faulted && sample.raw_x == 77 && timestamp == 42U && ok;
		{
			unsigned calls = transport.calls;
			ok = lsm303dlhc_reader_read(&reader, &sample, &timestamp) == E_OBJ &&
			     transport.calls == calls && ok;
		}
	}
	return ok;
}

static BOOL producer_test(void)
{
	imu_producer_t producer;
	imu_estimate_message_t message = {0};
	wheel_odometry_message_t odometry = {0};
	accel_data_t accel = {.az = GRAVITY_MS2, .i2c_ok = true};
	BOOL ok;

	ok = !imu_producer_init(&producer, 0U) &&
	     !imu_producer_init(&producer, UINT32_MAX) &&
	     imu_producer_init(&producer, 10U);
	message.header.sequence = 99U;
	ok = !imu_producer_update(&producer, &accel, NULL, 100U, &message) &&
	     message.header.sequence == 99U && !producer.have_sample && ok;
	odometry.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	odometry.header.version = VEHICLE_MESSAGE_VERSION;
	odometry.header.sequence = 0U;
	odometry.header.timestamp_ms = 102U;
	ok = !imu_producer_update(&producer, &accel, &odometry, 103U, &message) &&
	     !producer.have_odometry && message.header.sequence == 99U && ok;
	odometry.valid = 1U;
	ok = !imu_producer_update(&producer, &accel, &odometry, 100U, &message) &&
	     !producer.have_odometry && message.header.sequence == 99U && ok;
	odometry.header.timestamp_ms = 100U;
	ok = imu_producer_update(&producer, &accel, &odometry, 100U, &message) &&
	     !message.shock_detected && !message.tilt_detected &&
	     message.header.type == VEHICLE_MESSAGE_IMU_ESTIMATE &&
	     message.header.sequence == 1U && message.header.timestamp_ms == 100U &&
	     message.validity == VEHICLE_IMU_VALID_ROLL_PITCH &&
	     message.roll_mdeg == 0 && message.pitch_mdeg == 0 &&
	     message.yaw_mdeg == 0 && message.gyro_x == 0 && ok;
	accel.az = -5.0f;
	ok = imu_producer_update(&producer, &accel, NULL, 101U, &message) &&
	     message.tilt_detected && !message.shock_detected &&
	     (message.terrain_telemetry & 7U) == IMU_STATE_TILT &&
	     message.roll_mdeg >= 179000 && message.roll_mdeg <= 180001 && ok;
	odometry.header.sequence = 1U;
	odometry.header.timestamp_ms = 102U;
	accel.az = 50.0f;
	ok = imu_producer_update(&producer, &accel, &odometry, 103U, &message) &&
	     !message.shock_detected && message.tilt_detected &&
	     (message.terrain_telemetry & 7U) == IMU_STATE_MONITORING && ok;
	odometry.header.sequence = 3U;
	odometry.header.timestamp_ms = 104U;
	ok = !imu_producer_update(&producer, &accel, &odometry, 104U, &message) &&
	     !imu_producer_update(&producer, &accel, NULL, 113U, &message) &&
	     !imu_producer_update(&producer, &accel, NULL, 103U, &message) &&
	     producer.sequence == 3U && message.header.sequence == 3U && ok;
	accel.i2c_ok = false;
	ok = !imu_producer_update(&producer, &accel, NULL, 104U, &message) &&
	     producer.sequence == 3U && ok;
	accel.i2c_ok = true;
	accel.ax = __builtin_huge_valf();
	ok = !imu_producer_update(&producer, &accel, NULL, 104U, &message) &&
	     producer.sequence == 3U && ok;

	accel.ax = 0.0f;
	accel.az = GRAVITY_MS2;
	odometry.header.sequence = UINT32_MAX;
	odometry.header.timestamp_ms = UINT32_MAX - 5U;
	ok = imu_producer_init(&producer, 10U) &&
	     imu_producer_update(&producer, &accel, &odometry, UINT32_MAX - 1U, &message) &&
	     ok;
	odometry.header.sequence = 0U;
	odometry.header.timestamp_ms = 1U;
	ok = imu_producer_update(&producer, &accel, &odometry, 2U, &message) &&
	     message.header.timestamp_ms == 2U && ok;
	return ok;
}

BOOL imu_processor_test(void)
{
	BOOL terrain_ok;
	BOOL safety_ok;
	BOOL pickup_tilt_ok;
	BOOL hump_height_ok;
	BOOL clock_ok;
	BOOL producer_ok;
	BOOL accel_ok;
	BOOL reader_ok;
	TEST_INIT();

	terrain_ok = terrain_test();
	safety_ok = safety_test();
	pickup_tilt_ok = pickup_tilt_test();
	hump_height_ok = hump_height_test();
	clock_ok = clock_test();
	producer_ok = producer_test();
	accel_ok = accel_decode_test();
	reader_ok = accel_reader_test();
	TEST_CHECK("IMU-U01", terrain_ok, "flat, ramp and hump transitions");
	TEST_CHECK("IMU-U02", safety_ok, "tilt and invalid samples");
	TEST_CHECK("IMU-U09", pickup_tilt_ok, "pickup roll, low vertical acceleration and tilt latch");
	TEST_CHECK("IMU-U10", hump_height_ok, "known ramp height, telemetry quantization and timeout");
	TEST_CHECK("IMU-U03", clock_ok, "independent timeouts and clock wrap");
	TEST_CHECK("IMU-U04", producer_ok, "typed estimates and odometry validation");
	TEST_CHECK("IMU-U07", accel_ok, "LSM303DLHC acceleration decoding");
	TEST_CHECK("IMU-U08", reader_ok, "fresh accelerometer transfer and fault latch");
	TEST_SUMMARY("imu");
	return terrain_ok && safety_ok && pickup_tilt_ok && hump_height_ok &&
	       clock_ok && producer_ok && accel_ok && reader_ok;
}
#endif
