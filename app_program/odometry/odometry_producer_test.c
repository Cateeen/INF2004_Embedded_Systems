#include "../app.h"
#include "../vehicle_buffers.h"
#include "odometry_producer_test.h"

#if APP_TEST
#include "../test_report.h"
#include "odometry_producer.h"
#include "odometry_publish.h"

static BOOL calculation_test(void)
{
	odometry_producer_t state;
	const odometry_producer_config_t config = {{2500U, 20.0f}, {2500U, 20.0f}, 1, -1};
	wheel_odometry_message_t message;

	return odometry_producer_init(&state, &config, 0, 0, 10U) &&
	       odometry_producer_update(&state, 2500, -2500, 110U, &message) &&
	       message.header.type == VEHICLE_MESSAGE_WHEEL_ODOMETRY &&
	       message.header.version == VEHICLE_MESSAGE_VERSION &&
	       message.header.sequence == 1U && message.header.timestamp_ms == 110U &&
	       message.left_count == 2500 && message.right_count == 2500 &&
	       message.distance_mm == 200 && message.valid == 1U &&
	       odometry_producer_update(&state, 1250, -1250, 210U, &message) &&
	       message.header.sequence == 2U && message.distance_mm == 100;
}

static BOOL boundary_test(void)
{
	odometry_producer_t state;
	odometry_producer_config_t config = {{2500U, 20.0f}, {2500U, 20.0f}, 1, -1};
	wheel_odometry_message_t message = {0};

	if (!odometry_producer_init(&state, &config, INT32_MAX - 1, INT32_MIN + 2,
	                            UINT32_MAX - 50U) ||
	    !odometry_producer_update(&state, INT32_MIN + 1, INT32_MAX, 49U, &message) ||
	    state.left_total != 3 || state.right_total != 3 ||
	    message.left_count != INT32_MIN + 1 || message.right_count != INT32_MIN + 1)
		return false;
	if (odometry_producer_update(&state, INT32_MIN + 1, INT32_MAX, 49U, &message) ||
	    odometry_producer_update(&state, INT32_MIN + 1, INT32_MAX, 48U, &message) ||
	    state.sequence != 1U || message.header.sequence != 1U)
		return false;
	if (odometry_producer_update(&state, 1, INT32_MAX, 50U, &message) ||
	    state.sequence != 1U || message.header.sequence != 1U)
		return false;
	config.right_sign = 0;
	if (odometry_producer_init(&state, &config, 0, 0, 0U)) return false;
	config.right_sign = -1;
	config.left.counts_per_revolution = 0U;
	if (odometry_producer_init(&state, &config, 0, 0, 0U)) return false;
	config.left.counts_per_revolution = 2500U;
	if (!odometry_producer_init(&state, &config, 0, 0, 0U)) return false;
	state.left_total = INT64_MAX;
	if (odometry_producer_update(&state, 1, 0, 1U, &message) ||
	    state.left_total != INT64_MAX || state.sequence != 0U)
		return false;
	state.left_total = INT64_MAX / 2;
	state.right_total = INT64_MAX / 2;
	if (odometry_producer_update(&state, 1, -1, 1U, &message) ||
	    state.left_total != INT64_MAX / 2 || state.sequence != 0U)
		return false;
	config.left.counts_per_revolution = 1U;
	config.right.counts_per_revolution = 1U;
	config.left.wheel_circumference_cm = 1.0f;
	config.right.wheel_circumference_cm = 1.0f;
	return odometry_producer_init(&state, &config, 0, 0, 0U) &&
	       odometry_producer_update(&state, 16777217, -16777217, 1U, &message) &&
	       message.distance_mm == 167772170;
}

static BOOL drain_buffer(ID buffer)
{
	wheel_odometry_message_t message;
	INT result;
	do {
		result = tk_rcv_mbf(buffer, &message, TMO_POL);
	} while (result > 0);
	return result == E_TMOUT;
}

static BOOL publish_test(void)
{
	wheel_odometry_message_t sent = {0}, line = {0}, imu = {0};
	odometry_producer_t producer;
	const odometry_producer_config_t config = {{2500U, 20.0f}, {2500U, 20.0f}, 1, -1};
	odometry_publish_result_t result;
	BOOL ok;
	unsigned i;

	if (!odometry_producer_init(&producer, &config, 0, 0, 0U) ||
	    !odometry_producer_update(&producer, 2500, -2500, 100U, &sent))
		return FALSE;
	result = odometry_publish(&sent);
	sent.left_count = 0;
	sent.distance_mm = 0;
	ok = result.line == E_OK && result.imu == E_OK;
	ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) == (INT)sizeof(line) && ok;
	ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == (INT)sizeof(imu) && ok;
	ok = ok && line.header.sequence == 1U && imu.header.sequence == 1U &&
	     line.left_count == 2500 && imu.left_count == 2500 &&
	     line.distance_mm == 200 && imu.distance_mm == 200;

	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_line_odometry, &sent, (INT)sizeof(sent), TMO_POL) != E_OK)
			ok = FALSE;
	}
	result = odometry_publish(&sent);
	ok = result.line == E_TMOUT && result.imu == E_OK && ok;
	ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == (INT)sizeof(imu) && ok;
	ok = drain_buffer(g_mbf_line_odometry) && ok;
	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_imu_odometry, &sent, (INT)sizeof(sent), TMO_POL) != E_OK)
			ok = FALSE;
	}
	result = odometry_publish(&sent);
	ok = result.line == E_OK && result.imu == E_TMOUT && ok;
	ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) == (INT)sizeof(line) && ok;
	sent.valid = 0U;
	result = odometry_publish(&sent);
	ok = result.line == E_PAR && result.imu == E_PAR && ok;
	result = odometry_publish(NULL);
	ok = result.line == E_PAR && result.imu == E_PAR && ok;
	ok = drain_buffer(g_mbf_line_odometry) && ok;
	ok = drain_buffer(g_mbf_imu_odometry) && ok;
	return ok;
}

static BOOL motion_publish_test(void)
{
	wheel_odometry_message_t sent = {0}, line, imu, motion;
	odometry_publish_result_t result;
	BOOL ok;
	unsigned i;

	sent.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	sent.header.version = VEHICLE_MESSAGE_VERSION;
	sent.header.sequence = 7U;
	sent.left_count = 123;
	sent.valid = 1U;
	result = odometry_publish(&sent);
	ok = result.line == E_OK && result.imu == E_OK && !result.motion_attempted;
	ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) == (INT)sizeof(line) && ok;
	ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == (INT)sizeof(imu) && ok;
	ok = tk_rcv_mbf(g_mbf_motion_odometry, &motion, TMO_POL) == E_TMOUT && ok;
	result = odometry_publish_with_motion(&sent);
	sent.left_count = 456;
	ok = result.line == E_OK && result.imu == E_OK && result.motion_attempted &&
	     result.motion == E_OK && ok;
	ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) == (INT)sizeof(line) && ok;
	ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == (INT)sizeof(imu) && ok;
	ok = tk_rcv_mbf(g_mbf_motion_odometry, &motion, TMO_POL) == (INT)sizeof(motion) &&
	     line.left_count == 123 && imu.left_count == 123 && motion.left_count == 123 && ok;
	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_motion_odometry, &sent, (INT)sizeof(sent), TMO_POL) != E_OK)
			ok = FALSE;
	}
	result = odometry_publish_with_motion(&sent);
	ok = result.line == E_OK && result.imu == E_OK && result.motion_attempted &&
	     result.motion == E_TMOUT && ok;
	ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) == (INT)sizeof(line) && ok;
	ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == (INT)sizeof(imu) && ok;
	ok = drain_buffer(g_mbf_motion_odometry) && ok;
	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_line_odometry, &sent, (INT)sizeof(sent), TMO_POL) != E_OK)
			ok = FALSE;
	}
	result = odometry_publish_with_motion(&sent);
	ok = result.line == E_TMOUT && result.imu == E_OK &&
	     result.motion_attempted && result.motion == E_OK && ok;
	ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == (INT)sizeof(imu) && ok;
	ok = tk_rcv_mbf(g_mbf_motion_odometry, &motion, TMO_POL) == (INT)sizeof(motion) && ok;
	sent.valid = 0U;
	result = odometry_publish_with_motion(&sent);
	ok = result.line == E_PAR && result.imu == E_PAR &&
	     !result.motion_attempted && ok;
	ok = drain_buffer(g_mbf_line_odometry) && ok;
	ok = drain_buffer(g_mbf_imu_odometry) && ok;
	ok = drain_buffer(g_mbf_motion_odometry) && ok;
	return ok;
}

BOOL odometry_producer_test(void)
{
	BOOL calculation_ok;
	BOOL boundary_ok;
	BOOL publish_ok;
	BOOL motion_ok;
	TEST_INIT();

	calculation_ok = calculation_test();
	boundary_ok = boundary_test();
	publish_ok = publish_test();
	motion_ok = motion_publish_test();
	TEST_CHECK("ODO-U04", calculation_ok, "signed wheel counts and distance");
	TEST_CHECK("ODO-U05", boundary_ok, "wrap and invalid sample rejection");
	TEST_CHECK("INT-04", publish_ok, "independent odometry buffer delivery");
	TEST_CHECK("INT-05", motion_ok, "optional motion odometry delivery");
	TEST_SUMMARY("odometry producer");
	return calculation_ok && boundary_ok && publish_ok && motion_ok;
}
#endif
