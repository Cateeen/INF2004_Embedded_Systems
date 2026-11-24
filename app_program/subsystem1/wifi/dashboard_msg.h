/*
 *	dashboard_msg.h
 *	Wire formats shared with the Mission Control dashboard (dashboard/).
 *
 *	Pure functions only: no kernel calls, no globals. That keeps the exact
 *	bytes the dashboard parses unit-testable on the host as well as on the
 *	target. Every formatter is bounded by DASHBOARD_MSG_PAYLOAD_MAX and
 *	returns 0 (writing nothing usable) if an argument is out of range.
 *
 *	Topics (must match dashboard/server.js):
 *	  robot/event             JSON {"type":"...", ...} discrete events
 *	  robot/telemetry/detail  JSON object of optional numeric fields
 *	  robot/barcode           one ASCII letter A-Z
 *	  robot/heartbeat         2 bytes, see dashboard_msg_heartbeat()
 *	  robot/command           2 bytes from the dashboard, see wifi_command.h
 */

#ifndef DASHBOARD_MSG_H
#define DASHBOARD_MSG_H

#include <stdbool.h>
#include <stdint.h>

/* Longest formatted message is OBSTACLE_PROFILE at 99 bytes. */
#define DASHBOARD_MSG_PAYLOAD_MAX   112U
#define DASHBOARD_MSG_HEARTBEAT_LEN 2U

typedef enum {
	DASHBOARD_TOPIC_EVENT = 0,
	DASHBOARD_TOPIC_DETAIL,
	DASHBOARD_TOPIC_BARCODE,
	DASHBOARD_TOPIC_COUNT
} dashboard_topic_t;

/* Values equal the junction argument the dashboard sends on robot/command,
 * so a decoded argument can be stored without translation. 1 (reverse) is
 * never sent for a junction and is rejected. */
typedef enum {
	DASHBOARD_TURN_STRAIGHT = 0,
	DASHBOARD_TURN_LEFT     = 2,
	DASHBOARD_TURN_RIGHT    = 3,
	DASHBOARD_TURN_UTURN    = 4
} dashboard_turn_t;

typedef enum {
	DASHBOARD_SIDE_LEFT = 0,
	DASHBOARD_SIDE_RIGHT
} dashboard_side_t;

typedef enum {
	DASHBOARD_AVOID_SCANNING = 0,
	DASHBOARD_AVOID_BYPASS,
	DASHBOARD_AVOID_SEARCH,
	DASHBOARD_AVOID_REJOINED
} dashboard_avoid_phase_t;

typedef enum {
	DASHBOARD_EVENT_BARCODE_DETECTED = 0,
	DASHBOARD_EVENT_LINE_REJOINED,
	DASHBOARD_EVENT_PICKUP_STOP
} dashboard_simple_event_t;

/* MQTT topic string for a dashboard_topic_t, or NULL if out of range. */
const char *dashboard_msg_topic(uint8_t topic);

/* Each formatter writes at most DASHBOARD_MSG_PAYLOAD_MAX bytes (no NUL is
 * counted in the length; MQTT payloads are length-delimited) and returns
 * the length, or 0 on a bad argument. */
uint16_t dashboard_msg_simple_event(dashboard_simple_event_t event, char *out);
uint16_t dashboard_msg_action_done(dashboard_turn_t turn, char *out);
uint16_t dashboard_msg_hump_peak(uint16_t peak_mm, char *out);
uint16_t dashboard_msg_hump_height(uint16_t height_mm, char *out);
uint16_t dashboard_msg_obstacle(uint16_t distance_mm, char *out);
uint16_t dashboard_msg_obstacle_profile(uint16_t width_mm, uint16_t clear_left_mm,
					uint16_t clear_right_mm, dashboard_side_t side,
					char *out);
uint16_t dashboard_msg_avoid_phase(dashboard_avoid_phase_t phase, char *out);
uint16_t dashboard_msg_barcode(char letter, char *out);

/* Packs robot/heartbeat: byte 0 = (1 << 5) | (vehicle_state << 2),
 * byte 1 = sequence & 0x7f. vehicle_state is a WIFI_TELEMETRY_VEHICLE_*
 * value (0..4). Returns false and leaves out unchanged if it is not. */
bool dashboard_msg_heartbeat(uint8_t vehicle_state, uint8_t sequence,
			     uint8_t out[DASHBOARD_MSG_HEARTBEAT_LEN]);

/* True if argument is a valid junction turn from the dashboard. */
bool dashboard_msg_turn_valid(uint8_t argument);

#endif /* DASHBOARD_MSG_H */
