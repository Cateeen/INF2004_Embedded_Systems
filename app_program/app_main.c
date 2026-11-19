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
#include "odometry/encoder_source.h"
#include "gpio/gpio_dispatcher.h"
#include "line_barcode/line_observation_test.h"
#include "line_barcode/barcode_scanner_test.h"
#include "line_barcode/line_barcode_task.h"
#include "line_barcode/line_adc_reader.h"
#include "imu/imu_processor_test.h"
#include "imu/imu_task.h"
#include "imu/lsm303dlhc_i2c.h"
#include "obstacle/obs_profile_test.h"
#include "obstacle/obs_task.h"
#include "obstacle/obs_pico_port.h"
#include "motion/wheel_speed_test.h"
#include "motion/motion_task.h"
#include "controller/car_gate_test.h"
#include "controller/line_steering_test.h"
#include "controller/controller_monitor.h"
#include "controller/vehicle_controller_task.h"
#include "wifi/wifi_service.h"
#include "wifi/wifi_mqtt_bridge.h"
#include "gpio/gpio_irq_test.h"
#include "hardware_sensor_test.h"

#ifndef APP_VEHICLE
#define APP_VEHICLE 0		/* only the Pico makefile defines this */
#endif

#if APP_VEHICLE
/*
 * === Final-integration startup (VEHICLE=1) ==============================
 *
 * Hardware facts, kept in ONE place.  Set each verification flag to 1 ONLY
 * after measuring it on the robot (TEAM.md section 2): startup refuses to
 * run the vehicle without them, because the safety layer faults unless the
 * obstacle stream is alive, and the obstacle task needs verified electrical
 * and servo settings.
 */
#define VEHICLE_ECHO_VERIFIED_3V3_SAFE	1	/* GP4 divider measured <= 3.3 V */
#define VEHICLE_SERVO_LIMITS_VERIFIED	1	/* endpoints tested off the stops */
#define VEHICLE_SERVO_MIN_US		1000U
#define VEHICLE_SERVO_MAX_US		2000U

/* Odometry calibration, verified with ODO-H01 (encoder bring-up). */
#define VEHICLE_COUNTS_PER_REV		2500U
#define VEHICLE_WHEEL_CIRC_CM		20.4f
#define VEHICLE_LEFT_SIGN		1
#define VEHICLE_RIGHT_SIGN		1

/*
 * Trusted local inputs for the arming controller.  go stays false until a
 * real GO input (switch or verified local source) is wired here, so the
 * controller sits in WAITING: every sensor runs, motors stay braked and
 * nothing can arm.  Flip go to start line following once the robot is safe.
 */
static bool vehicle_read_inputs(void *context, vehicle_controller_inputs_t *inputs)
{
	(void)context;
	inputs->go = false;
	inputs->emergency_stopped = false;
	return true;
}

static void vehicle_start(void)
{
#if !VEHICLE_ECHO_VERIFIED_3V3_SAFE || !VEHICLE_SERVO_LIMITS_VERIFIED
	/* Repeat forever so a late serial monitor still sees why. */
	for (;;) {
		tm_printf((UB*)"vehicle startup blocked: hardware not verified\n");
#if !VEHICLE_ECHO_VERIFIED_3V3_SAFE
		tm_printf((UB*)"  - measure the HC-SR04 echo divider on GP4 (<= 3.3V),\n");
		tm_printf((UB*)"    then set VEHICLE_ECHO_VERIFIED_3V3_SAFE in app_main.c\n");
#endif
#if !VEHICLE_SERVO_LIMITS_VERIFIED
		tm_printf((UB*)"  - test the servo pulse endpoints (no mechanical stops),\n");
		tm_printf((UB*)"    then set VEHICLE_SERVO_LIMITS_VERIFIED in app_main.c\n");
#endif
		tm_printf((UB*)"no vehicle tasks started\n\n");
		tk_dly_tsk(5000);
	}
#else
	/* Calibration shared by odometry and motion feedback. */
	static const wheel_odometry_config_t wheel_calibration = {
		VEHICLE_COUNTS_PER_REV, VEHICLE_WHEEL_CIRC_CM
	};
	static const odometry_producer_config_t odometry_calibration = {
		{VEHICLE_COUNTS_PER_REV, VEHICLE_WHEEL_CIRC_CM},
		{VEHICLE_COUNTS_PER_REV, VEHICLE_WHEEL_CIRC_CM},
		VEHICLE_LEFT_SIGN, VEHICLE_RIGHT_SIGN
	};
	/* PID gains and base/max PWM are S5 placeholders: tune on blocks. */
	static motion_task_config_t motion_config = {
		{{{0}, {0}, 150U, 250U},
		 {2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U}},
		10U,	/* period_ms */
		100U,	/* command_max_age_ms */
		500U,	/* max_duration_ms */
		20U	/* max_linear_speed_cm_s */
	};
	/* Sensor deadlines; producers below publish well inside them. */
	static const vehicle_controller_task_config_t controller_config = {
		{200U, 200U, 500U, 5000U, 500U, 300U},
		50U,	/* period_ms, <= half the motion status period */
		500U,	/* motion_status_max_age_ms */
		vehicle_read_inputs, NULL
	};
	static const line_barcode_task_config_t line_config = {
		20U,	/* period_ms */
		200U,	/* odometry_max_age_ms */
		/* counts_per_module must be calibrated to the printed code. */
		{250U, 25U, 0U, 0U, 0U},
		NULL, NULL, NULL	/* reader installed by the ADC helper */
	};
	static const imu_task_config_t imu_config = {
		50U,	/* period_ms */
		200U,	/* sample_max_age_ms */
		200U,	/* odometry_max_age_ms */
		NULL, NULL, NULL	/* reader installed by the I2C helper */
	};
	static const obs_task_config_t obstacle_config = {
		50U,	/* period_ms */
		500U,	/* ahead_max_age_ms */
		1000U,	/* scan_max_age_ms */
		5000U,	/* scan_timeout_ms */
		500U,	/* command_max_age_ms */
		300U,	/* emergency_distance_mm */
		NULL, NULL, NULL, NULL	/* callbacks installed by the port */
	};
	/* Static: start functions may keep pointers past this task's exit. */
	static lsm303dlhc_i2c_config_t imu_sensor_config;
	static obs_pico_port_config_t obstacle_hardware;

	motion_config.control.feedback.left = wheel_calibration;
	motion_config.control.feedback.right = wheel_calibration;

	/* GPIO interrupt clients register before the dispatcher starts. */
	CHECK(encoder_source_register(), "encoder_source_register");
	CHECK(obs_pico_port_register(), "obs_pico_port_register");
	CHECK(gpio_dispatcher_start(), "gpio_dispatcher_start");

	/* Consumers before producers: motion drains commands and odometry,
	   the controller drains every sensor stream. */
	CHECK(motion_task_start_hardware(&motion_config), "motion_task_start_hardware");
	CHECK(vehicle_controller_task_start(&controller_config),
	      "vehicle_controller_task_start");

	CHECK(odometry_task_start_with_motion(&odometry_calibration, 50U),
	      "odometry_task_start");
	CHECK(line_adc_reader_start_default_task(10U, &line_config),
	      "line_adc_reader_start_default_task");
	CHECK(lsm303dlhc_i2c_default_config(&imu_sensor_config, 20U),
	      "lsm303dlhc_i2c_default_config");
	CHECK(lsm303dlhc_i2c_start_imu_task(&imu_sensor_config, &imu_config),
	      "lsm303dlhc_i2c_start_imu_task");

	obstacle_hardware = obs_pico_port_default_config();
	obstacle_hardware.echo_voltage_verified_3v3_safe = true;
	obstacle_hardware.servo_limits_verified = true;
	obstacle_hardware.servo_min_us = VEHICLE_SERVO_MIN_US;
	obstacle_hardware.servo_max_us = VEHICLE_SERVO_MAX_US;
	CHECK(obs_pico_port_task_start(&obstacle_hardware, &obstacle_config),
	      "obs_pico_port_task_start");

#if defined(TM_MQTT) && TM_MQTT
	CHECK(wifi_service_start_mqtt(100U, 1000U, 1000U, 1000U),
	      "wifi_service_start_mqtt");
#endif

	tm_printf((UB*)"vehicle tasks started; controller WAITING until GO\n\n");
#endif
}

/*
 * GPIO/encoder/obstacle registration demands core 0 (tk_get_prc() ==
 * TP_PRC1), but usermain may be scheduled on either core, so the whole
 * startup sequence runs in this one-shot task pinned to TP_PRC1.
 */
LOCAL void vehicle_startup_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	vehicle_start();	/* never returns when startup is blocked */
	tk_ext_tsk();
}
#endif /* APP_VEHICLE */

#if (APP_TEST || APP_VEHICLE || APP_SENSOR_TEST) && \
	defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
/* usb_console.c: 0 not started, 1 started, 2 enumerated, 3 host asserted DTR */
IMPORT UW tm_usb_state(void);

/*
 * Hold the test run or vehicle startup until a serial monitor opens the
 * port, so no boot message is printed before anyone can read it.  DTR
 * (state 3) is how a terminal announces itself, but some terminal programs
 * never assert it, so once the USB link is enumerated (state 2) a timeout
 * lets the run proceed anyway.
 */
LOCAL void wait_for_serial_monitor(void)
{
	UW	enumerated_ms = 0;

	tm_printf((UB*)"waiting for serial monitor...\n");
	while (tm_usb_state() < 3U) {
		if (tm_usb_state() == 2U) {
			enumerated_ms += 100U;
			if (enumerated_ms >= 60000U) break;	/* DTR-less terminal */
		}
		tk_dly_tsk(100);
	}
	tk_dly_tsk(500);	/* let the terminal finish opening before output bursts */
}
#endif

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
#if APP_SENSOR_TEST
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
	wait_for_serial_monitor();
#endif
	er = hardware_sensor_test_start();
	if (er < E_OK) {
		for (;;) {
			tm_printf((UB*)"hardware sensor test task failed to start: %d\n",
				  (int)er);
			tk_dly_tsk(5000);
		}
	}
	tk_slp_tsk(TMO_FEVR);
	return 0;
#endif
#if APP_EXAMPLES
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");
#endif

#if APP_TEST
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
	wait_for_serial_monitor();
#endif
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
#elif APP_VEHICLE
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
	wait_for_serial_monitor();
#endif
	/* Pinned to core 0: the GPIO IRQ registrations reject other cores. */
	if (app_start_task((FP)vehicle_startup_task, 11, TP_PRC1,
			   "vehicle_startup") < E_OK) {
		for (;;) {
			tm_printf((UB*)"vehicle startup task failed to start\n");
			tk_dly_tsk(5000);
		}
	}
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
