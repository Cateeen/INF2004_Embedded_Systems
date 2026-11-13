/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() creates vehicle buffers first. When EXAMPLES=1,
 *	it then creates every example subsystem's objects and starts its tasks.
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The example system (enabled with EXAMPLES=1): one task per folder
 *	------------------------------------------------------------
 *	  subsystem1/  LED       blinks GP19, paced by a cyclic handler
 *	  subsystem2/  Producer  makes a "sample" every 300 ms     (core 1)
 *	  subsystem3/  Consumer  collects samples into statistics  (core 0)
 *	  subsystem4/  Monitor   prints reports, pauses/resumes the LED
 *	  subsystem5/  (empty template for the fifth student)
 *
 *	How they talk to each other (the interfaces, see TEAM.md section 5):
 *	  IF-01  S2 -> S3  mailbox mbx_samples      (owned by S3)
 *	  IF-02  S3 -> S2  semaphore sem_slots + memory pool mpf_samples
 *	                   (owned by S2: S3 hands slots and blocks back)
 *	  IF-03  S3 -> S4  message buffer mbf_summary + event flag flg_events
 *	                   (owned by S4)
 *	  IF-04  S4 -> S3  reads app_stats under mutex mtx_stats (owned by S3)
 *	  IF-05  S4 -> S1  suspends/resumes the LED task tid_led (owned by S1)
 *	The subsystem that RECEIVES or OWNS the data creates the object and
 *	publishes its ID in its header; the other side just uses it.
 *
 *	micro T-Kernel APIs used, and where to find them:
 *	  tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *	              tk_dly_tsk tk_sus_tsk tk_rsm_tsk tk_chg_pri tk_ref_tsk
 *	              tk_get_tid tk_get_prc                         app_main.c, S1-S4
 *	  semaphore   tk_cre_sem tk_wai_sem tk_sig_sem tk_ref_sem       S2, S3, S4
 *	  mem. pool   tk_cre_mpf tk_get_mpf tk_rel_mpf tk_ref_mpf       S2, S3, S4
 *	  mailbox     tk_cre_mbx tk_snd_mbx tk_rcv_mbx                  S2, S3
 *	  mutex       tk_cre_mtx tk_loc_mtx tk_unl_mtx                  S3, S4
 *	  msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf                  S3, S4
 *	  event flag  tk_cre_flg tk_set_flg tk_wai_flg                  S3, S4
 *	  cyclic      tk_cre_cyc tk_ref_cyc                             S1
 *	  alarm       tk_cre_alm tk_sta_alm tk_stp_alm                  S4
 *	  time        tk_get_tim                                        S2, S4
 *	  system      tk_ref_ver                                        app_main.c
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "vehicle_buffers.h"
#include "odometry/odometry_test.h"
#include "odometry/odometry_producer_test.h"
#include "odometry/odometry_task.h"
#include "odometry/encoder_bringup.h"
#include "line_barcode/line_observation_test.h"
#include "line_barcode/barcode_scanner_test.h"
#include "line_barcode/line_barcode_task.h"
#include "line_barcode/line_adc_reader.h"
#include "imu/imu_processor_test.h"
#include "imu/imu_task.h"
#include "obstacle/obs_profile_test.h"
#include "obstacle/obs_task.h"
#include "motion/wheel_speed_test.h"
#include "motion/motion_task.h"
#include "controller/car_gate_test.h"
#include "controller/line_steering_test.h"
#include "controller/controller_monitor.h"
#include "controller/vehicle_controller_task.h"
#include "wifi/wifi_service.h"
#include "wifi/wifi_mqtt_bridge.h"
#include "gpio/gpio_irq_test.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;
	ER	er;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	er = vehicle_buffers_init();
	if (er < E_OK) {
		tm_printf((UB*)"vehicle buffer initialization failed; tasks not started\n");
		for (;;) {
			tk_dly_tsk(1000);
		}
	}
#if APP_EXAMPLES
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");
#endif

#if APP_TEST
	if (!vehicle_buffers_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] INT-03 FAIL  vehicle buffers; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!odometry_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] ODO FAIL  odometry calculations; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!odometry_producer_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] ODO-PRODUCER FAIL  producer tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!odometry_task_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] ODO-TASK FAIL  task tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!line_observation_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] LINE FAIL  line classification; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!barcode_scanner_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] BAR FAIL  barcode decoder; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!line_barcode_task_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] LB-TASK FAIL  task tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!line_adc_reader_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] LINE-ADC FAIL  reader tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!imu_processor_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] IMU FAIL  processing; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!imu_task_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] IMU-TASK FAIL  task tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!obs_profile_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] OBS FAIL  profiling; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!obs_task_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] OBS-TASK FAIL  task tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!wheel_speed_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] MOT FAIL  wheel speed control; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!motion_task_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] MOT-TASK FAIL  simulation tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!car_gate_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] VC FAIL  controller gate; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!line_steering_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] VC FAIL  line steering; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!controller_monitor_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] VC-MONITOR FAIL  monitor tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!vehicle_controller_task_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] VC-TASK FAIL  vehicle controller tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!wifi_service_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] WIFI FAIL  service tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
		if (!wifi_mqtt_bridge_test()) {
			for (;;) {
				tm_printf((UB*)"[TEST] WIFI-BRIDGE FAIL  bridge tests; tasks not started\n");
				tk_dly_tsk(1000);
			}
		}
	}
	if (!gpio_irq_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] GPIO FAIL  interrupt routing; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
#endif

	/* Phase 2: start the opt-in template example tasks. */
#if APP_EXAMPLES
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	tm_printf((UB*)"template example subsystems started\n\n");
#else
#if APP_ODOMETRY_TEST
	er = encoder_bringup_start();
	if (er < E_OK) {
		for (;;) {
			tm_printf((UB*)"encoder task start failed: %d\n", (int)er);
			tk_dly_tsk(1000);
		}
	}
	tm_printf((UB*)"encoder bring-up task started\n\n");
#else
	tm_printf((UB*)"template examples disabled; no vehicle tasks started yet\n\n");
#endif
#endif

#if APP_TEST
	/* With examples enabled, run every example subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
#if APP_EXAMPLES
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
#endif
	tm_printf((UB*)"[TEST] INT-03 PASS  vehicle buffer startup result (repeated)\n");
	tm_printf((UB*)"[TEST] ODO PASS  odometry startup tests (repeated)\n");
	tm_printf((UB*)"[TEST] ODO-PRODUCER PASS  producer tests (repeated)\n");
	tm_printf((UB*)"[TEST] ODO-TASK PASS  task tests (repeated)\n");
	tm_printf((UB*)"[TEST] LINE PASS  line classification tests (repeated)\n");
	tm_printf((UB*)"[TEST] BAR PASS  barcode decoder tests (repeated)\n");
	tm_printf((UB*)"[TEST] LB-TASK PASS  task tests (repeated)\n");
	tm_printf((UB*)"[TEST] LINE-ADC PASS  reader tests (repeated)\n");
	tm_printf((UB*)"[TEST] IMU PASS  IMU processing tests (repeated)\n");
	tm_printf((UB*)"[TEST] OBS PASS  obstacle profile tests (repeated)\n");
	tm_printf((UB*)"[TEST] MOT PASS  wheel speed tests (repeated)\n");
	tm_printf((UB*)"[TEST] VC PASS  controller gate tests (repeated)\n");
	tm_printf((UB*)"[TEST] GPIO PASS  interrupt routing tests (repeated)\n");
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

#if APP_TEST && !APP_EXAMPLES
	/* Keep the startup test result visible after USB serial connects. */
	for (;;) {
		tk_dly_tsk(5000);
		tm_printf((UB*)"[TEST] INT-03 PASS  vehicle buffer startup result (repeated)\n");
		tm_printf((UB*)"[TEST] ODO PASS  odometry startup tests (repeated)\n");
		tm_printf((UB*)"[TEST] ODO-PRODUCER PASS  producer tests (repeated)\n");
		tm_printf((UB*)"[TEST] ODO-TASK PASS  task tests (repeated)\n");
		tm_printf((UB*)"[TEST] LINE PASS  line classification tests (repeated)\n");
		tm_printf((UB*)"[TEST] BAR PASS  barcode decoder tests (repeated)\n");
		tm_printf((UB*)"[TEST] LB-TASK PASS  task tests (repeated)\n");
		tm_printf((UB*)"[TEST] LINE-ADC PASS  reader tests (repeated)\n");
		tm_printf((UB*)"[TEST] IMU PASS  IMU processing tests (repeated)\n");
		tm_printf((UB*)"[TEST] OBS PASS  obstacle profile tests (repeated)\n");
		tm_printf((UB*)"[TEST] MOT PASS  wheel speed tests (repeated)\n");
		tm_printf((UB*)"[TEST] VC PASS  controller gate tests (repeated)\n");
		tm_printf((UB*)"[TEST] GPIO PASS  interrupt routing tests (repeated)\n");
	}
#else
	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
#endif
}
