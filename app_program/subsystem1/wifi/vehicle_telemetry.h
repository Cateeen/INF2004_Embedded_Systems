/*
 *	vehicle_telemetry.h
 *	Periodic 5-byte robot/telemetry snapshot (vehicle, line, obstacle,
 *	terrain, faults) for the dashboard.
 *
 *	The vehicle controller owns all of these states, so it calls
 *	vehicle_telemetry_report() once per control cycle. This module
 *	rate-limits to VEHICLE_TELEMETRY_PERIOD_MS, packs the snapshot and queues
 *	it for the Wi-Fi service. It never blocks and never affects control:
 *	every error is ignored apart from a rate-limited log line.
 */

#ifndef VEHICLE_TELEMETRY_H
#define VEHICLE_TELEMETRY_H

#include <tk/tkernel.h>
#include "wifi_telemetry.h"

#define VEHICLE_TELEMETRY_PERIOD_MS  100U
/* Must not exceed the Wi-Fi service's telemetry_max_age_ms (1000 ms). */
#define VEHICLE_TELEMETRY_MAX_AGE_MS 500U

/* Call once from the startup owner after wifi_service_start_mqtt(). Until
 * then vehicle_telemetry_report() does nothing, which keeps the start-up
 * unit tests' telemetry queue clean. */
ER vehicle_telemetry_enable(void);

/* Single caller: the vehicle controller task. data is a snapshot taken at
 * now_ms. Also updates the vehicle state carried in robot/heartbeat. */
void vehicle_telemetry_report(const wifi_telemetry_data_t *data, uint32_t now_ms);

#endif /* VEHICLE_TELEMETRY_H */
