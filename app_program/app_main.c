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
 *	So usermain() creates vehicle buffers first and starts the selected
 *	vehicle or diagnostic tasks before sleeping.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "vehicle_buffers.h"
#include "subsystem6/odometry/odometry_test.h"
#include "subsystem6/odometry/odometry_producer_test.h"
#include "subsystem6/odometry/odometry_task.h"
#include "subsystem6/odometry/encoder_bringup.h"
#include "subsystem6/odometry/encoder_source.h"
#include "subsystem6/gpio/gpio_dispatcher.h"
#include "subsystem3/line_barcode/line_observation_test.h"
#include "subsystem3/line_barcode/junction_detector.h"
#include "subsystem3/line_barcode/straddle_junction.h"
#include "subsystem3/line_barcode/barcode_scanner_test.h"
#include "subsystem3/line_barcode/line_barcode_task.h"
#include "subsystem3/line_barcode/line_adc_reader.h"
#include "subsystem4/imu/imu_processor_test.h"
#include "subsystem4/imu/imu_task.h"
#include "subsystem4/imu/lsm303dlhc_i2c.h"
#include "subsystem5/obstacle/obs_profile_test.h"
#include "subsystem5/obstacle/spin_scan_test.h"
#include "subsystem5/obstacle/diag_pass_test.h"
#include "subsystem5/obstacle/obs_task.h"
#include "subsystem5/obstacle/obs_pico_port.h"
#include "subsystem2/motion/wheel_speed_test.h"
#include "subsystem2/motion/motion_task.h"
#include "subsystem2/controller/car_gate_test.h"
#include "subsystem2/controller/line_steering_test.h"
#include "subsystem2/controller/line_steering.h"
#include "subsystem2/controller/straddle_steering.h"
#include "subsystem2/controller/controller_monitor.h"
#include "subsystem2/controller/vehicle_controller_task.h"
#include "subsystem2/controller/vehicle_bypass.h"
#include "subsystem1/wifi/wifi_service.h"
#include "subsystem1/wifi/wifi_mqtt_bridge.h"
#include "subsystem1/wifi/dashboard_link.h"
#include "subsystem1/wifi/vehicle_telemetry.h"
#include "subsystem6/gpio/gpio_irq_test.h"
#include "hardware_sensor_test.h"

#include <sys/sysdef.h>
#include <tk/syslib.h>
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

#ifndef APP_NO_ULTRASONIC
#define APP_NO_ULTRASONIC 0	/* make NO_ULTRASONIC=1 */
#endif
#ifndef APP_VEHICLE
#define APP_VEHICLE 0		/* only the Pico makefile defines this */
#endif

#if APP_VEHICLE || APP_TEST
typedef struct {
	bool go;
	bool candidate_pressed;
	bool stable_pressed;
	bool ready;
	bool restart;		/* dashboard Start while GO was on: on next cycle */
	unsigned stable_samples;
} go_button_t;

static bool go_button_sample(go_button_t *button, bool pressed)
{
	if (pressed != button->candidate_pressed) {
		button->candidate_pressed = pressed;
		button->stable_samples = 1U;
	} else if (button->stable_samples < 2U) {
		button->stable_samples++;
	}
	if (button->stable_samples < 2U) return button->go;
	if (!button->ready) {
		if (!pressed) button->ready = true;
		return false;
	}
	if (pressed != button->stable_pressed) {
		button->stable_pressed = pressed;
		if (pressed) button->go = !button->go;
	}
	return button->go;
}
#endif

#if APP_TEST
#include "test_report.h"

static BOOL go_button_test(void)
{
	go_button_t button = {0};
	BOOL startup_ok, toggle_ok, bounce_ok;
	TEST_INIT();

	startup_ok = !go_button_sample(&button, true);
	startup_ok = !go_button_sample(&button, true) && startup_ok;
	startup_ok = !go_button_sample(&button, false) && startup_ok;
	startup_ok = !go_button_sample(&button, false) && startup_ok;
	TEST_CHECK("GO-U01", startup_ok, "held button at startup cannot enable GO");

	bounce_ok = !go_button_sample(&button, true);
	bounce_ok = !go_button_sample(&button, false) && bounce_ok;
	bounce_ok = !go_button_sample(&button, false) && bounce_ok;
	TEST_CHECK("GO-U02", bounce_ok, "a single noisy sample cannot toggle GO");

	toggle_ok = !go_button_sample(&button, true);
	toggle_ok = go_button_sample(&button, true) && toggle_ok;
	toggle_ok = go_button_sample(&button, true) && toggle_ok;
	toggle_ok = go_button_sample(&button, false) && toggle_ok;
	toggle_ok = go_button_sample(&button, false) && toggle_ok;
	toggle_ok = go_button_sample(&button, true) && toggle_ok;
	toggle_ok = !go_button_sample(&button, true) && toggle_ok;
	TEST_CHECK("GO-U03", toggle_ok, "distinct stable presses toggle GO once");

	TEST_SUMMARY("GO button");
	return startup_ok && bounce_ok && toggle_ok;
}
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
#define VEHICLE_SERVO_LIMITS_VERIFIED	1	/* 1500/1700/1900 us; measured +/-20 deg */
#define VEHICLE_GO_PIN			20U
/* 1: the dashboard's Start / Stop buttons act on the GO toggle exactly like
 * the GO button (Start = a press that turns GO on, Stop = GO off). Every
 * safety check still has to pass before the robot moves. 0: local button only. */
#define VEHICLE_DASHBOARD_RUN		1

/* Odometry calibration, verified with ODO-H01 (encoder bring-up). */
#define VEHICLE_COUNTS_PER_REV		2500U
#define VEHICLE_WHEEL_CIRC_CM		20.4f
/* Motion task speed limit for the faster wheel; the PID gains were verified
 * up to this speed (MOT-H01). Line steering must stay inside it. */
#define VEHICLE_MAX_SPEED_CM_S		20U
_Static_assert(VEHICLE_LINE_SPEED_CM_S + VEHICLE_LINE_TURN_CM_S <= VEHICLE_MAX_SPEED_CM_S &&
	       VEHICLE_LINE_PIVOT_SPEED_CM_S + VEHICLE_LINE_PIVOT_TURN_CM_S <= VEHICLE_MAX_SPEED_CM_S,
	       "straddle line following speed + turn must fit the motion task speed limit");
/* Wheels are mirror-mounted: the left encoder counts down when driving
 * forward (verified with BAR-H02), so it is negated. */
#define VEHICLE_LEFT_SIGN		(-1)
#define VEHICLE_RIGHT_SIGN		1

static go_button_t s_go_button;

/* Dashboard Start / Stop on the GO toggle. Start while GO is already on
 * (e.g. stopped after a lost line, which needs a fresh press) turns it off
 * for one cycle and back on, so the controller sees a rising edge. Reads one
 * atomic word: no network access, never blocks. */
static void vehicle_dashboard_run(go_button_t *button)
{
#if VEHICLE_DASHBOARD_RUN
	dashboard_run_command_t command;

	if (button->restart) {
		button->restart = false;
		button->go = true;
		return;
	}
	if (!dashboard_take_run_command(&command)) return;
	if (command == DASHBOARD_RUN_STOP) {
		button->go = false;
		tm_printf((UB*)"dashboard: STOP\n");
	} else if (command == DASHBOARD_RUN_START) {
		if (button->go) {
			button->go = false;
			button->restart = true;
		} else {
			button->go = true;
		}
		tm_printf((UB*)"dashboard: START\n");
	}
#else
	(void)button;
#endif
}

/* The controller stopped the robot by itself with GO still on: turn the
 * toggle off, so the next single press (or dashboard Start) starts it. Runs
 * in the controller task, like vehicle_read_inputs. */
static void vehicle_go_reset(void *context)
{
	go_button_t *button = context;

	if (!button->go && !button->restart) return;
	button->go = false;
	button->restart = false;
	tm_printf((UB*)"GO is off: press GO once to start\n");
}

/* GO pressed while faulted: reboot the whole system, as the Pico SDK's
 * watchdog_reboot() does -- the watchdog resets everything except the
 * oscillators, and scratch[4] = 0 so the boot ROM starts the flash image
 * normally. A latched fault has already sent the motors an emergency stop.
 * Note: the HC-SR04 stays powered, so a stuck ultrasonic sensor still
 * needs a full power-off. */
#define VEHICLE_PSM_WDSEL      (0x40010000U + 0x08U)
#define VEHICLE_PSM_WDSEL_ALL  0x0001fffcU	/* all but ROSC and XOSC */
#define VEHICLE_WATCHDOG_CTRL  (0x40058000U + 0x00U)
#define VEHICLE_WATCHDOG_SCRATCH4 (0x40058000U + 0x1cU)
#define VEHICLE_WATCHDOG_TRIGGER 0x80000000U

static void vehicle_restart(void *context)
{
	(void)context;
	tk_dly_tsk(100);	/* let the console send the last lines */
	out_w(VEHICLE_WATCHDOG_SCRATCH4, 0U);
	out_w(VEHICLE_PSM_WDSEL, VEHICLE_PSM_WDSEL_ALL);
	out_w(VEHICLE_WATCHDOG_CTRL, VEHICLE_WATCHDOG_TRIGGER);
	for (;;) {
		/* The reset takes effect at once. */
	}
}

static bool vehicle_read_inputs(void *context, vehicle_controller_inputs_t *inputs)
{
	go_button_t *button = context;

	(void)go_button_sample(button, gpio_get_val(VEHICLE_GO_PIN) == 0U);
	vehicle_dashboard_run(button);
	inputs->go = button->go && button->ready;
	inputs->emergency_stopped = false;
	return true;
}

/* Junction turns come from the dashboard (barcode letter -> turn); without
 * one, or before any barcode, dashboard_next_junction() returns LEFT. */
_Static_assert(VEHICLE_JUNCTION_STRAIGHT == DASHBOARD_TURN_STRAIGHT &&
	       VEHICLE_JUNCTION_LEFT == DASHBOARD_TURN_LEFT &&
	       VEHICLE_JUNCTION_RIGHT == DASHBOARD_TURN_RIGHT &&
	       VEHICLE_JUNCTION_UTURN == DASHBOARD_TURN_UTURN,
	       "controller junction turns must match dashboard_turn_t");

static uint8_t vehicle_junction_turn(void *context)
{
	(void)context;
	return (uint8_t)dashboard_next_junction();
}

static void vehicle_junction_done(void *context, uint8_t turn)
{
	(void)context;
	(void)dashboard_report_action_done((dashboard_turn_t)turn);	/* no-op without MQTT */
}

/* Each decoded barcode letter goes to the dashboard, which maps it to the
 * next junction turn and sends that back (dashboard_next_junction()). */
static void vehicle_barcode(void *context, const char *payload, uint8_t length)
{
	uint8_t i;

	(void)context;
	for (i = 0U; i < length; i++) {
		tm_printf((UB*)"barcode decoded: %c\n", payload[i]);
		(void)dashboard_report_barcode(payload[i]);	/* no-op without MQTT */
	}
}

/*
 * The radio task starts at boot on core 0 at priority 4 (above every vehicle
 * task) and downloads the CYW43 firmware in cyw43_init(), which keeps core 0
 * busy for a long stretch. Starting the vehicle meanwhile starves the core-0
 * startup, odometry and obstacle tasks, the sensor streams miss the
 * controller's deadlines and it faults at once. So wait for that phase
 * (init_complete) first; joining the network and MQTT come later in the
 * radio's light 2 ms poll loop. A radio that never finishes does not block
 * the vehicle for longer than VEHICLE_RADIO_WAIT_MS.
 */
#define VEHICLE_RADIO_WAIT_MS	15000U

static void vehicle_wait_for_radio(void)
{
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
	T_CYW43_UTK_STATUS st;
	UW waited = 0U;

	for (;;) {
		cyw43_utk_get_status(&st);
		if (st.init_complete) break;
		if (waited >= VEHICLE_RADIO_WAIT_MS) {
			tm_printf((UB*)"Wi-Fi radio still starting after %u ms; starting the vehicle anyway\n",
				  (unsigned)VEHICLE_RADIO_WAIT_MS);
			return;
		}
		tk_dly_tsk(50);
		waited += 50U;
	}
	tm_printf((UB*)"Wi-Fi radio started (%s) after %u ms; starting the vehicle\n",
		  st.ready ? "ok" : "FAILED", (unsigned)waited);
	tk_dly_tsk(200);	/* let the post-init burst settle */
#endif
}

/* Line task: a barcode's start '*' has been read (letter not yet decoded). */
static void vehicle_barcode_started(void *context)
{
	(void)context;
	(void)dashboard_report_barcode_detected();	/* no-op without MQTT */
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
		tm_printf((UB*)"  - verify servo clearance and scan angles on the robot,\n");
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
	/* Wheel speed PID {kp, ki, kd, target (set per command), base %, max %},
	 * tuned with SENSOR_TEST=pid_tune on the mat at 15 and 20 cm/s: steady
	 * error within 2.5 % per wheel forward and reverse, no oscillation
	 * (MOT-H01). Base 22 % is the floor PWM at 15 cm/s; KI 4 makes up the
	 * extra ~7 % needed at 20 cm/s within ~1 s.
	 * Feed-forward {+1.4 %/(cm/s) around 15 cm/s}, from those two points:
	 * each wheel starts at the PWM for its own target, so line-steering
	 * turns land at once instead of waiting for KI (line_follow test:
	 * 4/18 commanded reached only ~6/17 with the flat 22 % base). */
	static motion_task_config_t motion_config = {
		{{{0}, {0}, 150U, 250U},
		 {2.0f, 4.0f, 0.0f, 10.0f, 22U, 60U, 1.4f, 15.0f}},
		10U,	/* period_ms */
		100U,	/* command_max_age_ms */
		500U,	/* max_duration_ms */
		VEHICLE_MAX_SPEED_CM_S	/* max_linear_speed_cm_s */
	};
	/* Sensor deadlines; producers below publish well inside them. */
	static const vehicle_controller_task_config_t controller_config = {
		/* line, IMU, ahead max age; scan timeout; min line confidence;
		 * obstacle stop mm; 3 s for each stream's first message;
		 * straddle line sensors (arming needs the floor, not the tape
		 * under a sensor: centred reads both white). */
		{200U, 200U, 500U, 5000U, 500U, 200U, 3000U, true,
		 /* NO_ULTRASONIC=1: an ultrasonic that never answers turns
		  * obstacle detection off instead of faulting. */
		 APP_NO_ULTRASONIC != 0},
		50U,	/* period_ms, <= half the motion status period */
		500U,	/* motion_status_max_age_ms */
		vehicle_read_inputs, &s_go_button,
		vehicle_junction_turn, vehicle_junction_done, NULL,
		vehicle_barcode, NULL,
		/* Obstacle at <= 200 mm while following: scan it, drive round it
		 * and rejoin the line (vehicle_bypass, OBS-H03 manoeuvre). */
		true,
		/* Stopped by itself: GO toggles off, one press restarts. */
		vehicle_go_reset,
		/* Faulted: GO (or dashboard Start) reboots the robot. */
		vehicle_restart
	};
	static const line_barcode_task_config_t line_config = {
		10U,	/* period_ms: as the line_follow and barcode bench tests */
		200U,	/* odometry_max_age_ms */
		/* 35 counts = smallest narrow module expected (bench-calibrated
		 * code); bigger labels adapt, as the module is learned per frame. */
		{35U, 25U, 0U, 0U, 0U},
		NULL, NULL, NULL,	/* reader installed by the ADC helper */
		/* Junction detection on: mm of wheel travel per odometry count. */
		VEHICLE_WHEEL_CIRC_CM * 10.0f / (float)VEHICLE_COUNTS_PER_REV,
		vehicle_barcode_started, NULL,
		/* Barcode colour as SENSOR_TEST=barcode: hysteresis black >= 1000,
		 * white <= 600; a change counts after 4 counts (0.3 mm) of travel. */
		1000U, 600U, 4U,
		/* Line sensors as SENSOR_TEST=line_follow LF_STRADDLE=1: black >=
		 * 1500, white <= 900 (tape ~2300-3900, floor ~170-300). */
		1500U, 900U
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
		2500U,	/* scan_max_age_ms: 9 points x 3 polls x 50 ms ~ 1.3 s */
		5000U,	/* scan_timeout_ms */
		500U,	/* command_max_age_ms */
		/* An "emergency" reading this close (stops at once, no latch).
		 * Under the 200 mm stop distance: the robot stops at ~200 mm and
		 * the bypass works from there (it looks across at the obstacle's
		 * side from ~20 cm). */
		120U,	/* emergency_distance_mm */
		NULL, NULL, NULL, NULL	/* callbacks installed by the port */
	};
	/* Static: start functions may keep pointers past this task's exit. */
	static lsm303dlhc_i2c_config_t imu_sensor_config;
	static obs_pico_port_config_t obstacle_hardware;
	ER er;

	vehicle_wait_for_radio();
	motion_config.control.feedback.left = wheel_calibration;
	motion_config.control.feedback.right = wheel_calibration;

	er = gpio_set_pin(VEHICLE_GO_PIN, GPIO_MODE_IN);
	if (er != E_OK) {
		tm_printf((UB*)"GO button initialization failed: %d; tasks not started\n",
			  (int)er);
		return;
	}
	clr_w(GPIO(VEHICLE_GO_PIN), GPIO_PDE);
	set_w(GPIO(VEHICLE_GO_PIN), GPIO_PUE);

	/* GPIO interrupt clients register before the dispatcher starts. */
	CHECK(encoder_source_register(), "encoder_source_register");
	CHECK(obs_pico_port_register(), "obs_pico_port_register");
	CHECK(gpio_dispatcher_start(), "gpio_dispatcher_start");
	{
		/* Obstacle bypass task (core 0: encoders), bench-tested defaults. */
		static obstacle_bypass_config_t bypass_config;

		bypass_config = obstacle_bypass_default_config();
		CHECK(vehicle_bypass_start(&bypass_config), "vehicle_bypass_start");
	}

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
	imu_sensor_config.profile = LSM303DLHC_I2C_PROFILE_32_FAMILY;
	CHECK(lsm303dlhc_i2c_start_imu_task(&imu_sensor_config, &imu_config),
	      "lsm303dlhc_i2c_start_imu_task");

	obstacle_hardware = obs_pico_port_default_config();
	obstacle_hardware.echo_voltage_verified_3v3_safe = true;
	obstacle_hardware.servo_limits_verified = true;
	CHECK(obs_pico_port_task_start(&obstacle_hardware, &obstacle_config),
	      "obs_pico_port_task_start");

#if defined(TM_MQTT) && TM_MQTT
	/* Before the MQTT service so early events are queued, not dropped. */
	CHECK(dashboard_link_start(), "dashboard_link_start");
	CHECK(wifi_service_start_mqtt(100U, 1000U, 1000U, 1000U),
	      "wifi_service_start_mqtt");
	/* The controller's per-cycle snapshot becomes robot/telemetry. */
	CHECK(vehicle_telemetry_enable(), "vehicle_telemetry_enable");
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

#if (APP_TEST || APP_SENSOR_TEST) && \
	defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
/* usb_console.c: 0 not started, 1 started, 2 enumerated, 3 host asserted DTR */
IMPORT UW tm_usb_state(void);

/*
 * Hold the test run or sensor test until a serial monitor opens the
 * port, so no boot message is printed before anyone can read it.  DTR
 * (state 3) is how a terminal announces itself, but some terminal programs
 * never assert it, so once the USB link is enumerated (state 2) a timeout
 * lets the run proceed anyway.  With no USB host at all (running on
 * battery, or rebooted after the cable was pulled) the link never
 * enumerates, so after a short wait the run starts without a monitor;
 * output stays queued in the console ring until a host connects.
 */
LOCAL void wait_for_serial_monitor(void)
{
	UW	enumerated_ms = 0;
	UW	no_host_ms = 0;

	tm_printf((UB*)"waiting for serial monitor...\n");
	while (tm_usb_state() < 3U) {
		if (tm_usb_state() == 2U) {
			enumerated_ms += 100U;
			if (enumerated_ms >= 60000U) break;	/* DTR-less terminal */
		} else {
			no_host_ms += 100U;
			if (no_host_ms >= 5000U) break;		/* no USB host */
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
#if APP_SENSOR_TEST != 5
	er = vehicle_buffers_init();
	if (er < E_OK) {
		tm_printf((UB*)"vehicle buffer initialization failed; tasks not started\n");
		for (;;) {
			tk_dly_tsk(1000);
		}
	}
#endif
#if APP_SENSOR_TEST
	/* The Wi-Fi test (19) starts at once, like the vehicle build, so the
	 * dashboard link comes up with or without a serial monitor; its output
	 * stays queued in the USB console ring until a monitor opens. */
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC && APP_SENSOR_TEST != 19
	wait_for_serial_monitor();
#endif
#if APP_SENSOR_TEST == 5
	tm_printf((UB*)"starting encoder-only diagnostic\n");
	er = encoder_bringup_start();
#else
	er = hardware_sensor_test_start();
#endif
	if (er < E_OK) {
		for (;;) {
#if APP_SENSOR_TEST == 5
			tm_printf((UB*)"encoder task failed to start: %d\n", (int)er);
#else
			tm_printf((UB*)"hardware sensor test task failed to start: %d\n",
				  (int)er);
#endif
			tk_dly_tsk(5000);
		}
	}
	tk_slp_tsk(TMO_FEVR);
	return 0;
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
	if (!junction_detector_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] JN FAIL  junction detector; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!straddle_junction_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] JN FAIL  straddle junction detector; tasks not started\n");
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
	if (!spin_scan_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] OBS FAIL  spin scan width; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!diag_pass_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] OBS FAIL  diagonal pass width; tasks not started\n");
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
	if (!go_button_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] GO FAIL  button tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!straddle_steering_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] VC FAIL  straddle steering; tasks not started\n");
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
	}
	if (!wifi_mqtt_bridge_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] WIFI-BRIDGE FAIL  bridge tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!dashboard_link_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] DASHBOARD FAIL  dashboard link tests; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
	if (!gpio_irq_test()) {
		for (;;) {
			tm_printf((UB*)"[TEST] GPIO FAIL  interrupt routing; tasks not started\n");
			tk_dly_tsk(1000);
		}
	}
#endif

	/* Phase 2: start the selected vehicle or diagnostic tasks. */
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
	/* No wait for a serial monitor: the vehicle runs on battery without
	 * one. Boot messages printed before a terminal opens are lost. */
	/* Pinned to core 0: the GPIO IRQ registrations reject other cores. */
	if (app_start_task((FP)vehicle_startup_task, 11, TP_PRC1,
			   "vehicle_startup") < E_OK) {
		for (;;) {
			tm_printf((UB*)"vehicle startup task failed to start\n");
			tk_dly_tsk(5000);
		}
	}
#else
	tm_printf((UB*)"no vehicle tasks started\n\n");
#endif

#if APP_TEST
	tm_printf((UB*)"[TEST] === test run start ===\n");
	tm_printf((UB*)"[TEST] INT-03 PASS  vehicle buffer startup result (repeated)\n");
	tm_printf((UB*)"[TEST] ODO PASS  odometry startup tests (repeated)\n");
	tm_printf((UB*)"[TEST] ODO-PRODUCER PASS  producer tests (repeated)\n");
	tm_printf((UB*)"[TEST] ODO-TASK PASS  task tests (repeated)\n");
	tm_printf((UB*)"[TEST] LINE PASS  line classification tests (repeated)\n");
	tm_printf((UB*)"[TEST] JN PASS  junction detector tests (repeated)\n");
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

#if APP_TEST
	/* Keep the startup test result visible after USB serial connects. */
	for (;;) {
		tk_dly_tsk(5000);
		tm_printf((UB*)"[TEST] INT-03 PASS  vehicle buffer startup result (repeated)\n");
		tm_printf((UB*)"[TEST] ODO PASS  odometry startup tests (repeated)\n");
		tm_printf((UB*)"[TEST] ODO-PRODUCER PASS  producer tests (repeated)\n");
		tm_printf((UB*)"[TEST] ODO-TASK PASS  task tests (repeated)\n");
		tm_printf((UB*)"[TEST] LINE PASS  line classification tests (repeated)\n");
		tm_printf((UB*)"[TEST] JN PASS  junction detector tests (repeated)\n");
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
