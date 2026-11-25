#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "encoder_source.h"
#include "odometry_publish.h"
#include "odometry_task.h"

#define ODOMETRY_TASK_PRIORITY 10
#define ODOMETRY_FAULT_RETRY_MS 1000U

static odometry_producer_config_t s_config;
static RELTIM s_period_ms;
static bool s_started;
static bool s_publish_motion;

static uint16_t process_snapshot(odometry_producer_t *producer,
				 const encoder_snapshot_t *snapshot,
				 uint32_t timestamp_ms,
				 bool publish_motion,
				 odometry_publish_result_t *delivery)
{
	wheel_odometry_message_t message;

	if (!snapshot->valid) return VEHICLE_FAULT_CODE_ODOMETRY_SOURCE;
	if (!odometry_producer_update(producer, snapshot->left_count,
				      snapshot->right_count, timestamp_ms, &message))
		return VEHICLE_FAULT_CODE_ODOMETRY_CALCULATION;
	*delivery = publish_motion ? odometry_publish_with_motion(&message) :
				    odometry_publish(&message);
	if (delivery->line != E_OK || delivery->imu != E_OK ||
	    (publish_motion && (!delivery->motion_attempted || delivery->motion != E_OK)))
		return VEHICLE_FAULT_CODE_ODOMETRY_DELIVERY;
	return 0U;
}

static void report_fault(uint16_t code, uint32_t timestamp_ms)
{
	fault_message_t fault = {0};
	bool sent = false;

	fault.header.type = VEHICLE_MESSAGE_FAULT;
	fault.header.version = VEHICLE_MESSAGE_VERSION;
	fault.header.sequence = 1U;
	fault.header.timestamp_ms = timestamp_ms;
	fault.source = code == VEHICLE_FAULT_CODE_ODOMETRY_DELIVERY ?
		       VEHICLE_FAULT_COMMUNICATION :
		       (code == VEHICLE_FAULT_CODE_ODOMETRY_CLOCK ||
			code == VEHICLE_FAULT_CODE_ODOMETRY_SCHEDULE ?
			VEHICLE_FAULT_SYSTEM : VEHICLE_FAULT_SENSOR);
	fault.code = code;
	fault.recoverable = 0U;
	tm_printf((UB*)"odometry stopped; fault code %u\n", (unsigned)code);
	if (code == VEHICLE_FAULT_CODE_ODOMETRY_SOURCE) {
		/* Which wheel and why the encoder decoder gave up (this task is
		 * pinned to core 0, as encoder_source_fault requires). */
		encoder_fault_t f;

		if (encoder_source_fault(&f) >= E_OK && f.reason != ENCODER_FAULT_NONE) {
			tm_printf((UB*)"  ENCODER FAULT: %s wheel, reason %u (1 both channels "
				  "jumped, over the limit of %u; 2 lost edge; 3 bad level), "
				  "%u glitches and %u both-channel jumps tolerated before it\n",
				  f.wheel == 1U ? "LEFT" : "RIGHT", (unsigned)f.reason,
				  (unsigned)ENCODER_AMBIGUOUS_LIMIT, (unsigned)f.glitches,
				  (unsigned)f.ambiguous);
			tm_printf((UB*)"  fault detail: edges 0x%02x, state AB %u%u -> %u%u, count %d\n",
				  (unsigned)f.events, (unsigned)((f.previous_state >> 1) & 1U),
				  (unsigned)(f.previous_state & 1U),
				  (unsigned)((f.current_state >> 1) & 1U),
				  (unsigned)(f.current_state & 1U), (int)f.count);
		}
	}

	for (;;) {
		ER er;
		if (!sent) {
			er = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
			if (er == E_OK) {
				sent = true;
			} else {
				tm_printf((UB*)"odometry fault delivery failed: %d\n", (int)er);
			}
		}
		er = tk_dly_tsk(ODOMETRY_FAULT_RETRY_MS);
		if (er < E_OK) {
			tm_printf((UB*)"odometry fault retry delay failed: %d\n", (int)er);
			er = tk_slp_tsk(TMO_FEVR);
			tm_printf((UB*)"odometry fault sleep returned: %d\n", (int)er);
		}
	}
}

static void odometry_task(INT stacd, void *exinf)
{
	odometry_producer_t producer = {0};
	encoder_snapshot_t snapshot;
	odometry_publish_result_t delivery = {0};
	SYSTIM now;
	uint32_t last_timestamp = 0U;
	uint16_t fault;
	ER er;

	(void)stacd;
	(void)exinf;
	er = encoder_source_snapshot(&snapshot);
	if (er < E_OK || !snapshot.valid) {
		tm_printf((UB*)"odometry initial snapshot failed: %d\n", (int)er);
		report_fault(VEHICLE_FAULT_CODE_ODOMETRY_SOURCE, last_timestamp);
	}
	er = tk_get_otm(&now);
	if (er < E_OK) {
		tm_printf((UB*)"odometry clock failed: %d\n", (int)er);
		report_fault(VEHICLE_FAULT_CODE_ODOMETRY_CLOCK, last_timestamp);
	}
	last_timestamp = now.lo;
	if (!odometry_producer_init(&producer, &s_config, snapshot.left_count,
				    snapshot.right_count, last_timestamp))
		report_fault(VEHICLE_FAULT_CODE_ODOMETRY_CALCULATION, last_timestamp);

	for (;;) {
		er = tk_dly_tsk(s_period_ms);
		if (er < E_OK) {
			tm_printf((UB*)"odometry delay failed: %d\n", (int)er);
			report_fault(VEHICLE_FAULT_CODE_ODOMETRY_SCHEDULE, last_timestamp);
		}
		er = encoder_source_snapshot(&snapshot);
		if (er < E_OK) {
			tm_printf((UB*)"odometry snapshot failed: %d\n", (int)er);
			report_fault(VEHICLE_FAULT_CODE_ODOMETRY_SOURCE, last_timestamp);
		}
		er = tk_get_otm(&now);
		if (er < E_OK) {
			tm_printf((UB*)"odometry clock failed: %d\n", (int)er);
			report_fault(VEHICLE_FAULT_CODE_ODOMETRY_CLOCK, last_timestamp);
		}
		last_timestamp = now.lo;
		fault = process_snapshot(&producer, &snapshot, last_timestamp,
					 s_publish_motion, &delivery);
		if (fault != 0U) {
			if (fault == VEHICLE_FAULT_CODE_ODOMETRY_DELIVERY)
				tm_printf((UB*)"odometry delivery failed: line=%d imu=%d motion_attempted=%u motion=%d\n",
					  (int)delivery.line, (int)delivery.imu,
					  (unsigned)delivery.motion_attempted, (int)delivery.motion);
			report_fault(fault, last_timestamp);
		}
	}
}

static ER start_odometry(const odometry_producer_config_t *config, RELTIM period_ms,
			 bool publish_motion)
{
	odometry_producer_t validation;
	encoder_snapshot_t snapshot;
	T_CTSK task = {0};
	ID id;
	ER er;

#if TK_SUPPORT_SMP
	if (tk_get_prc() != TP_PRC1) return E_CTX;
#endif
	if (s_started) return E_OBJ;
	if (period_ms == 0U || period_ms > INT32_MAX ||
	    !odometry_producer_init(&validation, config, 0, 0, 0U))
		return E_PAR;
	if (g_mbf_line_odometry <= 0 || g_mbf_imu_odometry <= 0 || g_mbf_faults <= 0 ||
	    (publish_motion && g_mbf_motion_odometry <= 0))
		return E_OBJ;
	er = encoder_source_snapshot(&snapshot);
	if (er < E_OK) return er;
	if (!snapshot.valid) return E_IO;

	s_config = *config;
	s_period_ms = period_ms;
	s_publish_motion = publish_motion;
	task.task = (FP)odometry_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = ODOMETRY_TASK_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC1;
#endif
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er < E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup < E_OK) {
			tm_printf((UB*)"odometry task cleanup failed: %d\n", (int)cleanup);
		} else {
			s_started = false;
		}
		return er;
	}
	return E_OK;
}

ER odometry_task_start(const odometry_producer_config_t *config, RELTIM period_ms)
{
	return start_odometry(config, period_ms, false);
}

ER odometry_task_start_with_motion(const odometry_producer_config_t *config,
				   RELTIM period_ms)
{
	return start_odometry(config, period_ms, true);
}

#if APP_TEST
#include "../../test_report.h"

static BOOL drain_odometry_buffer(ID buffer)
{
	wheel_odometry_message_t message;
	INT result;

	do {
		result = tk_rcv_mbf(buffer, &message, TMO_POL);
	} while (result > 0);
	return result == E_TMOUT;
}

BOOL odometry_task_test(void)
{
	const odometry_producer_config_t config = {{2500U, 20.0f}, {2500U, 20.0f}, 1, -1};
	odometry_producer_t producer = {0};
	encoder_snapshot_t snapshot = {2500, -2500, true};
	odometry_publish_result_t delivery = {0};
	wheel_odometry_message_t line = {0}, imu = {0};
	BOOL step_ok;
	BOOL fault_ok;
	BOOL motion_ok;
	unsigned i;
	TEST_INIT();

	step_ok = odometry_producer_init(&producer, &config, 0, 0, 0U);
	if (step_ok) {
		step_ok = process_snapshot(&producer, &snapshot, 100U, false, &delivery) == 0U;
		step_ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) ==
			  (INT)sizeof(line) && step_ok;
		step_ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) ==
			  (INT)sizeof(imu) && step_ok;
		step_ok = step_ok && line.distance_mm == 200 && imu.distance_mm == 200 &&
			  line.header.sequence == 1U && imu.header.sequence == 1U;
	}

	snapshot.valid = false;
	fault_ok = process_snapshot(&producer, &snapshot, 200U, false, &delivery) ==
		   VEHICLE_FAULT_CODE_ODOMETRY_SOURCE && producer.sequence == 1U;
	fault_ok = tk_rcv_mbf(g_mbf_line_odometry, &line, TMO_POL) == E_TMOUT && fault_ok;
	fault_ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) == E_TMOUT && fault_ok;
	snapshot.valid = true;
	fault_ok = process_snapshot(&producer, &snapshot, 100U, false, &delivery) ==
		   VEHICLE_FAULT_CODE_ODOMETRY_CALCULATION && fault_ok;

	line.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	line.header.version = VEHICLE_MESSAGE_VERSION;
	line.valid = 1U;
	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_line_odometry, &line, (INT)sizeof(line), TMO_POL) != E_OK)
			fault_ok = FALSE;
	}
	fault_ok = process_snapshot(&producer, &snapshot, 200U, false, &delivery) ==
		   VEHICLE_FAULT_CODE_ODOMETRY_DELIVERY && fault_ok;
	fault_ok = delivery.line == E_TMOUT && delivery.imu == E_OK && fault_ok;
	fault_ok = tk_rcv_mbf(g_mbf_imu_odometry, &imu, TMO_POL) ==
		   (INT)sizeof(imu) && fault_ok;
	fault_ok = drain_odometry_buffer(g_mbf_line_odometry) && fault_ok;
	fault_ok = drain_odometry_buffer(g_mbf_imu_odometry) && fault_ok;

	motion_ok = odometry_producer_init(&producer, &config, 0, 0, 0U);
	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		if (tk_snd_mbf(g_mbf_motion_odometry, &line, (INT)sizeof(line), TMO_POL) != E_OK)
			motion_ok = FALSE;
	}
	motion_ok = process_snapshot(&producer, &snapshot, 100U, true, &delivery) ==
		    VEHICLE_FAULT_CODE_ODOMETRY_DELIVERY &&
		    delivery.line == E_OK && delivery.imu == E_OK &&
		    delivery.motion_attempted && delivery.motion == E_TMOUT && motion_ok;
	motion_ok = drain_odometry_buffer(g_mbf_line_odometry) && motion_ok;
	motion_ok = drain_odometry_buffer(g_mbf_imu_odometry) && motion_ok;
	motion_ok = drain_odometry_buffer(g_mbf_motion_odometry) && motion_ok;

	TEST_CHECK("ODO-U08", step_ok, "task step delivers simulated snapshot");
	TEST_CHECK("ODO-U09", fault_ok, "task step rejects invalid snapshots and delivery failures");
	TEST_CHECK("ODO-U10", motion_ok, "full motion buffer stops odometry");
	TEST_SUMMARY("odometry task");
	return step_ok && fault_ok && motion_ok;
}
#endif
