#include "../../app.h"
#include "encoder_bringup.h"

#if APP_ODOMETRY_TEST || APP_SENSOR_TEST == 5
#include "encoder_source.h"
#include "../gpio/gpio_dispatcher.h"

#define ENCODER_COUNTS_PER_REV 2500
#define WHEEL_CIRCUMFERENCE_MM 200
#define ENCODER_REPORT_MS 1000U
#define ENCODER_TASK_PRIORITY 10

static void encoder_bringup_task(INT stacd, void *exinf)
{
	ER er;

	(void)stacd;
	(void)exinf;
	er = encoder_source_register();
	if (er >= E_OK) er = gpio_dispatcher_start();
	if (er < E_OK) {
		for (;;) {
			tm_printf((UB*)"encoder bring-up initialization failed: %d\n", (int)er);
			tk_dly_tsk(1000);
		}
	}
	tm_printf((UB*)"encoder bring-up ready: GP16/17 left, GP18/19 right; motors untouched\n");
	for (;;) {
		encoder_snapshot_t snapshot;
		int32_t left_mm;
		int32_t right_mm;

		er = encoder_source_snapshot(&snapshot);
		if (er < E_OK) {
			tm_printf((UB*)"encoder snapshot failed: %d\n", (int)er);
			tk_dly_tsk(ENCODER_REPORT_MS);
			continue;
		}
		left_mm = (int32_t)(((int64_t)snapshot.left_count * WHEEL_CIRCUMFERENCE_MM) /
				    ENCODER_COUNTS_PER_REV);
		right_mm = (int32_t)(((int64_t)snapshot.right_count * WHEEL_CIRCUMFERENCE_MM) /
				     ENCODER_COUNTS_PER_REV);
		tm_printf((UB*)"encoder: left=%d (%d mm) right=%d (%d mm)%s\n",
			  (int)snapshot.left_count, (int)left_mm,
			  (int)snapshot.right_count, (int)right_mm,
			  snapshot.valid ? "" : " INVALID: ambiguous or missed edges");
		tk_dly_tsk(ENCODER_REPORT_MS);
	}
}

ER encoder_bringup_start(void)
{
	T_CTSK task = {0};
	ID id;
	ER er;

	task.task = (FP)encoder_bringup_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = ENCODER_TASK_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC1;
#endif
	id = tk_cre_tsk(&task);
	if (id < E_OK) {
		return id;
	}
	er = tk_sta_tsk(id, 0);
	if (er < E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup < E_OK) {
			tm_printf((UB*)"encoder task cleanup failed: %d\n", (int)cleanup);
		}
		return er;
	}
	return E_OK;
}
#endif
