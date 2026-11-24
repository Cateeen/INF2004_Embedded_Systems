/*
 *	wifi_link_hwtest.c
 *	Wi-Fi -> MQTT -> dashboard connectivity test (make SENSOR_TEST=wifi).
 *	See wifi_link_hwtest.h and testing/integration/TEST_LOG.md (WIFI-H01..03).
 *
 *	The test never touches the network stack itself: the built-in
 *	cyw43_utk_task joins Wi-Fi and runs MQTT, exactly as in VEHICLE=1.
 *	This task only reads their status and uses the normal application
 *	APIs (dashboard_link, vehicle_telemetry), so a PASS here means the
 *	vehicle build will connect the same way.
 */

#include "../../app.h"
#include "wifi_link_hwtest.h"

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif

#if APP_SENSOR_TEST == 19

#include "cyw43_utk.h"
#include "wifi_secrets.h"
#include "wifi_service.h"
#include "wifi_mqtt_bridge.h"
#include "dashboard_link.h"
#include "vehicle_telemetry.h"

/* CYW43_LINK_* values (cyw43.h), mirrored in T_CYW43_UTK_STATUS.link_status. */
#define LINK_DOWN	0
#define LINK_JOIN	1
#define LINK_NOIP	2
#define LINK_UP		3
#define LINK_FAIL	(-1)
#define LINK_NONET	(-2)
#define LINK_BADAUTH	(-3)

#define TICK_MS			200U
#define WIFI_TIMEOUT_MS		30000U	/* boot -> IP address */
#define MQTT_TIMEOUT_MS		15000U	/* IP address -> MQTT session */
#define DASH_TIMEOUT_MS		20000U	/* first ping -> dashboard reply */
#define DASH_PING_PERIOD_MS	5000U	/* resend the test barcode */
#define REMINDER_PERIOD_MS	15000U	/* repeat a FAIL while still waiting */
#define STATUS_PERIOD_MS	10000U	/* status line once everything passed */
#define LINK_MAX_AGE_MS		1000U	/* bridge snapshot freshness */
#define PING_LETTER		'W'	/* barcode letter sent to the dashboard */

#define TAG "[HWTEST][WIFI] "

typedef enum { STAGE_WAIT, STAGE_PASS, STAGE_FAIL } stage_t;

static uint32_t now_ms(void)
{
	SYSTIM now;

	return (tk_get_otm(&now) == E_OK) ? (uint32_t)now.lo : 0U;
}

static void print_ip(uint32_t ip)
{
	tm_printf((UB*)"%d.%d.%d.%d", (int)(ip & 0xffU), (int)((ip >> 8) & 0xffU),
		  (int)((ip >> 16) & 0xffU), (int)((ip >> 24) & 0xffU));
}

static void print_seconds(uint32_t ms)
{
	tm_printf((UB*)"%u.%u s", (unsigned)(ms / 1000U), (unsigned)((ms % 1000U) / 100U));
}

static const char *link_name(int32_t status)
{
	switch (status) {
	case LINK_DOWN:    return "down (joining)";
	case LINK_JOIN:    return "joined AP, waiting for IP";
	case LINK_NOIP:    return "joined AP, no IP yet (DHCP)";
	case LINK_UP:      return "up";
	case LINK_FAIL:    return "association failed";
	case LINK_NONET:   return "SSID not found";
	case LINK_BADAUTH: return "wrong password / auth rejected";
	default:           return "unknown";
	}
}

static const char *turn_name(dashboard_turn_t turn)
{
	switch (turn) {
	case DASHBOARD_TURN_STRAIGHT: return "STRAIGHT";
	case DASHBOARD_TURN_LEFT:     return "LEFT";
	case DASHBOARD_TURN_RIGHT:    return "RIGHT";
	case DASHBOARD_TURN_UTURN:    return "U-TURN";
	default:                      return "?";
	}
}

static void wifi_fail_hint(int32_t status)
{
	tm_printf((UB*)TAG "FAIL  WIFI-H01 no IP address after %u s (last state: %s)\n",
		  (unsigned)(WIFI_TIMEOUT_MS / 1000U), link_name(status));
	switch (status) {
	case LINK_NONET:
		tm_printf((UB*)TAG "  hint: \"%s\" not seen. Check the name in config/wifi_secrets.h,\n"
			  TAG "        and that it is a 2.4 GHz network (the Pico W has no 5 GHz).\n",
			  WIFI_SSID);
		break;
	case LINK_BADAUTH:
		tm_printf((UB*)TAG "  hint: password rejected. Check WIFI_PASSWORD; WPA3-only networks\n"
			  TAG "        are not supported (use WPA2 or WPA2/WPA3 mixed mode).\n");
		break;
	case LINK_JOIN:
	case LINK_NOIP:
		tm_printf((UB*)TAG "  hint: joined the AP but DHCP gave no address. Phone hotspots\n"
			  TAG "        sometimes limit clients; check the router's DHCP pool.\n");
		break;
	default:
		tm_printf((UB*)TAG "  hint: check SSID/password in config/wifi_secrets.h and that the\n"
			  TAG "        AP is in range. The radio keeps retrying every 5 s.\n");
		break;
	}
}

static void mqtt_fail_hint(void)
{
	tm_printf((UB*)TAG "FAIL  WIFI-H02 no MQTT session with " MQTT_BROKER_IP ":%d after %u s\n",
		  (int)MQTT_BROKER_PORT, (unsigned)(MQTT_TIMEOUT_MS / 1000U));
	tm_printf((UB*)TAG "  hint: is the broker running on that PC (e.g. mosquitto -v)?\n"
		  TAG "        mosquitto 2.x only listens on localhost by default; add\n"
		  TAG "          listener %d 0.0.0.0\n"
		  TAG "          allow_anonymous true\n"
		  TAG "        to mosquitto.conf. Allow port %d through the PC firewall, and\n"
		  TAG "        check MQTT_BROKER_IP is the PC's address on the same network.\n",
		  (int)MQTT_BROKER_PORT, (int)MQTT_BROKER_PORT);
}

static void dash_fail_hint(void)
{
	tm_printf((UB*)TAG "FAIL  WIFI-H03 no reply from the dashboard after %u s\n",
		  (unsigned)(DASH_TIMEOUT_MS / 1000U));
	tm_printf((UB*)TAG "  hint: start it on the PC:  cd dashboard && MQTT_URL=mqtt://"
		  MQTT_BROKER_IP ":%d npm start\n"
		  TAG "        (PowerShell: $env:MQTT_URL=\"mqtt://" MQTT_BROKER_IP ":%d\"; npm start)\n"
		  TAG "        It must use the SAME broker as the robot and show\n"
		  TAG "        \"broker CONNECTED\". Still sending the test barcode every %u s.\n",
		  (int)MQTT_BROKER_PORT, (int)MQTT_BROKER_PORT,
		  (unsigned)(DASH_PING_PERIOD_MS / 1000U));
}

static void report_forever(const char *what, ER er)
{
	for (;;) {
		tm_printf((UB*)TAG "FAIL  %s: %d; test stopped\n", what, (int)er);
		tk_dly_tsk(5000);
	}
}

/* Idle snapshot so the dashboard sees robot/telemetry (its "car
 * connected" card is driven by telemetry, not by the heartbeat). */
static void send_idle_telemetry(uint32_t t)
{
	static const wifi_telemetry_data_t idle = {
		WIFI_TELEMETRY_VEHICLE_IDLE, WIFI_TELEMETRY_LINE_UNKNOWN,
		WIFI_TELEMETRY_OBSTACLE_UNKNOWN, WIFI_TELEMETRY_TERRAIN_UNKNOWN,
		0U, 0U
	};

	vehicle_telemetry_report(&idle, t);
}

void wifi_link_hwtest_run(void)
{
	T_CYW43_UTK_STATUS radio;
	stage_t wifi = STAGE_WAIT, mqtt = STAGE_WAIT, dash = STAGE_WAIT;
	int32_t last_link = 0x7fffffff;
	bool link_up = false, last_link_up = false;
	bool mqtt_ready = false, last_mqtt_ready = false;
	bool summary_done = false, radio_reported = false;
	uint32_t start, t, link_up_at = 0U, mqtt_up_at = 0U;
	uint32_t ping_first_at = 0U, ping_last_at = 0U, last_reminder = 0U;
	uint32_t last_status = 0U, commands_seen, pings = 0U;
	ER er;

	tm_printf((UB*)"\n" TAG "=== Wi-Fi / MQTT / dashboard link test ===\n");
	tm_printf((UB*)TAG "SSID \"%s\", broker " MQTT_BROKER_IP ":%d, client id \"%s\"\n",
		  WIFI_SSID, (int)MQTT_BROKER_PORT, MQTT_CLIENT_ID);
	tm_printf((UB*)TAG "no motors or sensors are used\n");

	/* Same start-up order as VEHICLE=1 (app_main.c vehicle_start()). */
	er = dashboard_link_start();
	if (er != E_OK) report_forever("dashboard_link_start", er);
	er = wifi_service_start_mqtt(100U, 1000U, 1000U, LINK_MAX_AGE_MS);
	if (er != E_OK) report_forever("wifi_service_start_mqtt", er);
	er = vehicle_telemetry_enable();
	if (er != E_OK) report_forever("vehicle_telemetry_enable", er);

	start = now_ms();
	commands_seen = dashboard_command_count();

	for (;;) {
		bool connected = false, ready = false;
		uint32_t commands;
		dashboard_run_command_t run;

		tk_dly_tsk(TICK_MS);
		t = now_ms();
		cyw43_utk_get_status(&radio);
		send_idle_telemetry(t);

		/* ---- WIFI-H01: radio, join, DHCP -------------------------------- */
		if (radio.init_complete && radio.init_result != 0) {
			if (!radio_reported || (t - last_reminder) >= REMINDER_PERIOD_MS) {
				tm_printf((UB*)TAG "FAIL  WIFI-H01 the CYW43 radio did not start (err %d).\n"
					  TAG "  hint: is this a Pico W? Power-cycle it; check the 3V3 supply.\n",
					  (int)radio.init_result);
				radio_reported = true;
				last_reminder = t;
			}
			continue;
		}
		if (!radio.init_complete) {
			continue;		/* radio still loading firmware */
		}
		if (radio.link_status != last_link) {
			last_link = radio.link_status;
			tm_printf((UB*)TAG "wifi: %s\n", link_name(radio.link_status));
		}
		link_up = (radio.link_status == LINK_UP);
		if (link_up && !last_link_up) {
			link_up_at = t;
			if (wifi != STAGE_PASS) {
				tm_printf((UB*)TAG "PASS  WIFI-H01 joined \"%s\", ip=", WIFI_SSID);
				print_ip(radio.ip_addr);
				tm_printf((UB*)" (");
				print_seconds(t - start);
				tm_printf((UB*)" after boot)\n");
				wifi = STAGE_PASS;
			} else {
				tm_printf((UB*)TAG "wifi link back up, ip=");
				print_ip(radio.ip_addr);
				tm_printf((UB*)"\n");
			}
		} else if (!link_up && last_link_up) {
			tm_printf((UB*)TAG "WARN  wifi link LOST (%s); radio is rejoining\n",
				  link_name(radio.link_status));
		}
		last_link_up = link_up;
		if (wifi != STAGE_PASS) {
			if ((t - start) >= WIFI_TIMEOUT_MS &&
			    (wifi == STAGE_WAIT || (t - last_reminder) >= REMINDER_PERIOD_MS)) {
				wifi_fail_hint(radio.link_status);
				wifi = STAGE_FAIL;
				last_reminder = t;
			}
			continue;
		}

		/* ---- WIFI-H02: MQTT session ------------------------------------ */
		if (wifi_mqtt_bridge_link(LINK_MAX_AGE_MS, &connected, &ready) != E_OK) {
			ready = false;
		}
		mqtt_ready = link_up && ready;
		if (mqtt_ready && !last_mqtt_ready) {
			mqtt_up_at = t;
			if (mqtt != STAGE_PASS) {
				tm_printf((UB*)TAG "PASS  WIFI-H02 MQTT connected to " MQTT_BROKER_IP ":%d (",
					  (int)MQTT_BROKER_PORT);
				print_seconds(t - link_up_at);
				tm_printf((UB*)" after IP)\n");
				mqtt = STAGE_PASS;
			} else {
				tm_printf((UB*)TAG "MQTT session back up\n");
			}
		} else if (!mqtt_ready && last_mqtt_ready) {
			tm_printf((UB*)TAG "WARN  MQTT session LOST; reconnecting\n");
		}
		last_mqtt_ready = mqtt_ready;
		if (mqtt != STAGE_PASS) {
			if (link_up && (t - link_up_at) >= MQTT_TIMEOUT_MS &&
			    (mqtt == STAGE_WAIT || (t - last_reminder) >= REMINDER_PERIOD_MS)) {
				mqtt_fail_hint();
				mqtt = STAGE_FAIL;
				last_reminder = t;
			}
			continue;
		}

		/* ---- Anything from the dashboard (robot/command) --------------- */
		commands = dashboard_command_count();
		if (commands != commands_seen) {
			commands_seen = commands;
			if (dash != STAGE_PASS) {
				tm_printf((UB*)TAG "PASS  WIFI-H03 dashboard replied on robot/command: "
					  "next junction %s (", turn_name(dashboard_next_junction()));
				print_seconds(t - ping_first_at);
				tm_printf((UB*)" after first test barcode)\n");
				dash = STAGE_PASS;
			} else {
				tm_printf((UB*)TAG "robot/command received (#%u), next junction %s\n",
					  (unsigned)commands, turn_name(dashboard_next_junction()));
			}
		}
		if (dashboard_take_run_command(&run)) {
			tm_printf((UB*)TAG "dashboard button: %s\n",
				  run == DASHBOARD_RUN_START ? "START" : "STOP");
		}

		/* ---- WIFI-H03: dashboard round trip ---------------------------- */
		if (dash != STAGE_PASS) {
			if (!mqtt_ready) continue;
			if (pings == 0U || (t - ping_last_at) >= DASH_PING_PERIOD_MS) {
				er = dashboard_report_barcode(PING_LETTER);
				if (pings == 0U) {
					ping_first_at = t;
					tm_printf((UB*)TAG "sent test barcode '%c' on robot/barcode; "
						  "waiting for the dashboard's reply...\n", PING_LETTER);
				}
				if (er != E_OK && er != E_TMOUT) {
					tm_printf((UB*)TAG "test barcode not queued: %d\n", (int)er);
				}
				ping_last_at = t;
				pings++;
			}
			if ((t - ping_first_at) >= DASH_TIMEOUT_MS &&
			    (dash == STAGE_WAIT || (t - last_reminder) >= REMINDER_PERIOD_MS)) {
				dash_fail_hint();
				dash = STAGE_FAIL;
				last_reminder = t;
			}
			continue;
		}

		/* ---- All three passed ------------------------------------------- */
		if (!summary_done) {
			tm_printf((UB*)TAG "==========================================\n");
			tm_printf((UB*)TAG "RESULT  WiFi PASS   MQTT PASS   Dashboard PASS\n");
			tm_printf((UB*)TAG "On the dashboard: the car card should be green (telemetry\n"
				  TAG "every 100 ms) and the log shows \"Decoded barcode %c\" from\n"
				  TAG "this test. Press Start / Stop to check the downlink again.\n",
				  PING_LETTER);
			tm_printf((UB*)TAG "==========================================\n");
			summary_done = true;
			last_status = t;
		} else if ((t - last_status) >= STATUS_PERIOD_MS) {
			tm_printf((UB*)TAG "status: wifi %s, ip=", link_up ? "UP" : "DOWN");
			print_ip(radio.ip_addr);
			tm_printf((UB*)", mqtt %s, commands received %u, up ",
				  mqtt_ready ? "READY" : "DOWN", (unsigned)commands_seen);
			print_seconds(t - mqtt_up_at);
			tm_printf((UB*)"\n");
			last_status = t;
		}
	}
}

#endif /* APP_SENSOR_TEST == 19 */
