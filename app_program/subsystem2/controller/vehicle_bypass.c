/*
 * vehicle_bypass.c - the vehicle's obstacle bypass task. See vehicle_bypass.h.
 */
#include "../../app.h"
#include "vehicle_bypass.h"
#include "../motion/motion_task.h"
#include "../../subsystem1/wifi/dashboard_link.h"

#ifndef APP_VEHICLE
#define APP_VEHICLE 0
#endif

#if APP_VEHICLE

#define VEHICLE_BYPASS_PRIORITY   10
#define VEHICLE_BYPASS_STACK      4096   /* scan result + tm_printf */
/* The controller has already seen the motion task idle, so the lend comes
 * on its next cycle (10 ms); this only guards against a stuck handshake. */
#define VEHICLE_BYPASS_BORROW_MS  500U

static obstacle_bypass_config_t s_config;
static ID s_task;
static bool s_started;

/*
 * Shared between the controller task and the bypass task (either core).
 * Every access is __atomic. The ahead reading is a pair, so it uses a
 * sequence count (odd while being written; one writer, the controller).
 * s_state is a handshake: IDLE -> RUNNING (controller), RUNNING -> DONE
 * (bypass task, after s_result), DONE -> IDLE (controller), so each change
 * has exactly one writer.
 */
static uint32_t s_seq;
static uint16_t s_mm;
static uint32_t s_ts;
static bool s_have_reading;
static bool s_abort;
static uint8_t s_state;                  /* vehicle_bypass_state_t */
static uint8_t s_result;                 /* obstacle_bypass_status_t */

static bool io_latest_ahead(void *context, uint16_t *mm, uint32_t *timestamp_ms)
{
	uint32_t a, b;
	uint16_t d;
	uint32_t t;

	(void)context;
	if (!__atomic_load_n(&s_have_reading, __ATOMIC_ACQUIRE)) return false;
	do {
		a = __atomic_load_n(&s_seq, __ATOMIC_ACQUIRE);
		d = __atomic_load_n(&s_mm, __ATOMIC_RELAXED);
		t = __atomic_load_n(&s_ts, __ATOMIC_RELAXED);
		__atomic_thread_fence(__ATOMIC_ACQUIRE);
		b = __atomic_load_n(&s_seq, __ATOMIC_RELAXED);
	} while (a != b || (a & 1U) != 0U);
	*mm = d;
	*timestamp_ms = t;
	return true;
}

static bool io_abort(void *context)
{
	(void)context;
	return __atomic_load_n(&s_abort, __ATOMIC_ACQUIRE);
}

/* Progress to the dashboard (queued, never blocks; no-op without MQTT). */
static void io_event(void *context, const obstacle_bypass_event_t *event)
{
	(void)context;
	switch (event->kind) {
	case OBSTACLE_BYPASS_EVENT_PROFILE:
		(void)dashboard_report_obstacle_profile(event->width_mm, event->clear_left_mm,
			event->clear_right_mm,
			event->pass_right ? DASHBOARD_SIDE_RIGHT : DASHBOARD_SIDE_LEFT);
		break;
	case OBSTACLE_BYPASS_EVENT_PASSING:
		(void)dashboard_report_avoid_phase(DASHBOARD_AVOID_SEARCH);
		break;
	case OBSTACLE_BYPASS_EVENT_REJOINED:
		(void)dashboard_report_line_rejoined();
		break;
	default:
		break;
	}
}

static obstacle_bypass_status_t run_with_motors(void)
{
	static const obstacle_bypass_io_t io = {
		io_latest_ahead, io_abort, NULL, NULL, "[BYPASS] ", io_event
	};
	obstacle_bypass_status_t st;

	/* The motion task owns the motor driver: borrow it for the run. */
	if (!motion_task_borrow_motors(io_abort, NULL, VEHICLE_BYPASS_BORROW_MS)) {
		tm_printf((UB*)"[BYPASS] the motion task did not lend the motors\n");
		return OBSTACLE_BYPASS_ABORTED;
	}
	st = io_abort(NULL) ? OBSTACLE_BYPASS_ABORTED : obstacle_bypass_run(&s_config, &io);
	motion_task_return_motors();
	return st;
}

static void vehicle_bypass_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	for (;;) {
		obstacle_bypass_status_t st;
		ER er = tk_slp_tsk(TMO_FEVR);

		if (er != E_OK) {
			tm_printf((UB*)"[BYPASS] sleep failed: %d\n", (int)er);
			tk_dly_tsk(1000U);
			continue;
		}
		if (__atomic_load_n(&s_state, __ATOMIC_ACQUIRE) != VEHICLE_BYPASS_RUNNING) continue;
		tm_printf((UB*)"[BYPASS] obstacle: scanning, then driving round it\n");
		st = run_with_motors();
		tm_printf((UB*)"[BYPASS] finished: %s\n", obstacle_bypass_status_text(st));
		/* The motors are back with the motion task before DONE is seen. */
		__atomic_store_n(&s_result, (uint8_t)st, __ATOMIC_RELAXED);
		__atomic_store_n(&s_state, (uint8_t)VEHICLE_BYPASS_DONE, __ATOMIC_RELEASE);
	}
}

ER vehicle_bypass_start(const obstacle_bypass_config_t *config)
{
	T_CTSK task = {0};
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL) return E_PAR;
	s_config = *config;
	task.task = (FP)vehicle_bypass_task;
	task.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC1;          /* encoder_source_snapshot() needs core 0 */
#endif
	task.itskpri = VEHICLE_BYPASS_PRIORITY;
	task.stksz = VEHICLE_BYPASS_STACK;
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		(void)tk_del_tsk(id);
		return er < E_OK ? er : E_SYS;
	}
	s_task = id;
	s_started = true;
	return E_OK;
}

void vehicle_bypass_feed(uint16_t distance_mm, uint32_t timestamp_ms)
{
	uint32_t seq = __atomic_load_n(&s_seq, __ATOMIC_RELAXED);

	__atomic_store_n(&s_seq, seq + 1U, __ATOMIC_RELAXED);
	__atomic_thread_fence(__ATOMIC_RELEASE);
	__atomic_store_n(&s_mm, distance_mm, __ATOMIC_RELAXED);
	__atomic_store_n(&s_ts, timestamp_ms, __ATOMIC_RELAXED);
	__atomic_store_n(&s_seq, seq + 2U, __ATOMIC_RELEASE);
	__atomic_store_n(&s_have_reading, true, __ATOMIC_RELEASE);
}

bool vehicle_bypass_request(void)
{
	if (!s_started || __atomic_load_n(&s_state, __ATOMIC_ACQUIRE) != VEHICLE_BYPASS_IDLE)
		return false;
	__atomic_store_n(&s_abort, false, __ATOMIC_RELAXED);
	__atomic_store_n(&s_state, (uint8_t)VEHICLE_BYPASS_RUNNING, __ATOMIC_RELEASE);
	if (tk_wup_tsk(s_task) != E_OK) {
		__atomic_store_n(&s_state, (uint8_t)VEHICLE_BYPASS_IDLE, __ATOMIC_RELEASE);
		return false;
	}
	return true;
}

void vehicle_bypass_abort(void)
{
	__atomic_store_n(&s_abort, true, __ATOMIC_RELEASE);
}

vehicle_bypass_state_t vehicle_bypass_state(obstacle_bypass_status_t *result)
{
	vehicle_bypass_state_t state =
		(vehicle_bypass_state_t)__atomic_load_n(&s_state, __ATOMIC_ACQUIRE);

	if (state == VEHICLE_BYPASS_DONE && result != NULL)
		*result = (obstacle_bypass_status_t)__atomic_load_n(&s_result, __ATOMIC_RELAXED);
	return state;
}

void vehicle_bypass_ack(void)
{
	if (__atomic_load_n(&s_state, __ATOMIC_ACQUIRE) == VEHICLE_BYPASS_DONE)
		__atomic_store_n(&s_state, (uint8_t)VEHICLE_BYPASS_IDLE, __ATOMIC_RELEASE);
}

#else /* !APP_VEHICLE: the controller links these but never starts a run. */

ER vehicle_bypass_start(const obstacle_bypass_config_t *config)
{
	(void)config;
	return E_NOSPT;
}

void vehicle_bypass_feed(uint16_t distance_mm, uint32_t timestamp_ms)
{
	(void)distance_mm;
	(void)timestamp_ms;
}

bool vehicle_bypass_request(void)
{
	return false;
}

void vehicle_bypass_abort(void)
{
}

vehicle_bypass_state_t vehicle_bypass_state(obstacle_bypass_status_t *result)
{
	(void)result;
	return VEHICLE_BYPASS_IDLE;
}

void vehicle_bypass_ack(void)
{
}

#endif /* APP_VEHICLE */
