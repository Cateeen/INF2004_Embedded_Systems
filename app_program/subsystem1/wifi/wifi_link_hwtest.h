#ifndef WIFI_LINK_HWTEST_H
#define WIFI_LINK_HWTEST_H

#include <tk/tkernel.h>

/* Wi-Fi / MQTT / dashboard link test (make SENSOR_TEST=wifi). Checks, in
 * order, and prints PASS/FAIL with a hint for each:
 *   WIFI-H01  the radio joins WIFI_SSID and gets an IP address
 *   WIFI-H02  the MQTT session to MQTT_BROKER_IP:MQTT_BROKER_PORT is up
 *   WIFI-H03  the dashboard answers: a test barcode goes out on
 *             robot/barcode and the dashboard's turn comes back on
 *             robot/command
 * It also publishes idle robot/telemetry so the dashboard's car card turns
 * green, and keeps reporting link state and any Start/Stop presses.
 * Body of the hardware sensor test task: never returns. No motors, no
 * sensors. */
void wifi_link_hwtest_run(void);

#endif
