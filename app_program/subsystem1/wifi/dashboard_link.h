/*
 *	dashboard_link.h
 *	One-line reporting calls from any subsystem to the Mission Control
 *	dashboard, and the junction instruction the dashboard sends back.
 *
 *	Integration guide: subsystem1/wifi/DASHBOARD_INTEGRATION.md
 *
 *	Reporting (dashboard_report_*)
 *	  - Callable from any application task on either core.
 *	  - Never blocks: the message is copied into a small queue that the
 *	    network task drains. E_OK means queued, not delivered.
 *	  - E_TMOUT means the queue was full and this message was dropped;
 *	    E_OBJ means dashboard_link_start() has not run (e.g. MQTT=0 build).
 *	    Callers may ignore both: telemetry loss must never stop the car.
 *	  - Report EVENTS (edges), not every control-loop sample. The queue
 *	    holds DASHBOARD_LINK_QUEUE_CAPACITY messages.
 *
 *	Junction instruction (dashboard_next_junction)
 *	  - The dashboard maps each decoded barcode letter to a turn and sends
 *	    it on robot/command. Until one arrives the answer is
 *	    DASHBOARD_TURN_LEFT, which matches the dashboard's display.
 */

#ifndef DASHBOARD_LINK_H
#define DASHBOARD_LINK_H

#include <tk/tkernel.h>
#include "dashboard_msg.h"

#define DASHBOARD_LINK_QUEUE_CAPACITY 8U
#define DASHBOARD_DEFAULT_TURN        DASHBOARD_TURN_LEFT

/* Run commands from the dashboard's Start / Stop buttons. Values equal
 * WIFI_COMMAND_START / WIFI_COMMAND_STOP in wifi_command.h. */
typedef enum {
	DASHBOARD_RUN_START = 1,
	DASHBOARD_RUN_STOP  = 4
} dashboard_run_command_t;

/* Call once from the startup owner, after vehicle_buffers_init() and
 * before wifi_service_start_mqtt(). Returns E_NOSPT in a non-MQTT build
 * (all report calls then return E_OBJ, which is harmless). */
ER dashboard_link_start(void);

/* ---- Subsystem 3: line + barcode --------------------------------------- */
/* Bars seen under the barcode sensor, before decoding finishes. */
ER dashboard_report_barcode_detected(void);
/* Decoded character, A-Z (lower case accepted). The dashboard replies with
 * the mapped turn, read back through dashboard_next_junction(). */
ER dashboard_report_barcode(char letter);

/* ---- Subsystem 2: controller / motion ---------------------------------- */
/* The junction turn has been completed. */
ER dashboard_report_action_done(dashboard_turn_t turn);
/* Car lifted: motors stopped. Ends the run timer on the dashboard. */
ER dashboard_report_pickup(void);
/* WIFI_TELEMETRY_VEHICLE_* value carried in robot/heartbeat. Not queued;
 * the latest value is sent with the next heartbeat. */
void dashboard_set_vehicle_state(uint8_t vehicle_state);

/* ---- Subsystem 4: IMU / terrain ---------------------------------------- */
/* Live height while on a hump. Rate-limit this (<= 5 Hz, only while the
 * terrain state is HUMP). */
ER dashboard_report_hump_height(uint16_t height_mm);
/* Highest point of one hump, sent once when the car leaves it. */
ER dashboard_report_hump_peak(uint16_t peak_mm);

/* ---- Subsystem 5: obstacle --------------------------------------------- */
ER dashboard_report_obstacle(uint16_t distance_mm);
ER dashboard_report_obstacle_profile(uint16_t width_mm, uint16_t clear_left_mm,
				     uint16_t clear_right_mm, dashboard_side_t bypass_side);
ER dashboard_report_avoid_phase(dashboard_avoid_phase_t phase);
ER dashboard_report_line_rejoined(void);

/* ---- From the dashboard ------------------------------------------------- */
/* Turn to take at the next junction: the mapping of the most recently
 * decoded barcode, or DASHBOARD_DEFAULT_TURN if none has arrived since
 * boot, the last dashboard Start, or dashboard_junction_reset(). */
dashboard_turn_t dashboard_next_junction(void);
/* Return to DASHBOARD_DEFAULT_TURN, e.g. after the turn is executed if
 * each barcode should apply to one junction only. */
void dashboard_junction_reset(void);
/* Single consumer (the vehicle controller). Returns true once per new
 * Start/Stop press and writes it to command. */
bool dashboard_take_run_command(dashboard_run_command_t *command);
/* Number of valid robot/command messages received since boot (wraps).
 * Any task may read it; used by the Wi-Fi link diagnostic. */
uint32_t dashboard_command_count(void);

#if APP_TEST
BOOL dashboard_link_test(void);
#endif

#endif /* DASHBOARD_LINK_H */
