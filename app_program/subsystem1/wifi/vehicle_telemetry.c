/*
 *	vehicle_telemetry.c
 *	Rate-limited telemetry snapshots from the vehicle controller to the
 *	Wi-Fi service (see vehicle_telemetry.h).
 */

#include "../../app.h"
#include "vehicle_telemetry.h"
#include "wifi_telemetry_producer.h"
#include "dashboard_link.h"

#define TELEMETRY_LOG_INTERVAL_MS 5000U

static wifi_telemetry_producer_t s_producer;
static int s_enabled;			/* written once by the startup owner */
static bool s_have_sent;		/* controller task only */
static uint32_t s_last_sent_ms;		/* controller task only */
static uint32_t s_last_log_ms;		/* controller task only */
static bool s_logged;			/* controller task only */

ER vehicle_telemetry_enable(void)
{
	ER result;

	if (__atomic_load_n(&s_enabled, __ATOMIC_ACQUIRE) != 0) {
		return E_OBJ;
	}
	result = wifi_telemetry_producer_init(&s_producer, 0U, VEHICLE_TELEMETRY_MAX_AGE_MS);
	if (result == E_OK) {
		__atomic_store_n(&s_enabled, 1, __ATOMIC_RELEASE);
	}
	return result;
}

void vehicle_telemetry_report(const wifi_telemetry_data_t *data, uint32_t now_ms)
{
	ER result;

	if ((data == NULL) || (__atomic_load_n(&s_enabled, __ATOMIC_ACQUIRE) == 0)) {
		return;
	}
	dashboard_set_vehicle_state(data->vehicle_state);
	if (s_have_sent && ((now_ms - s_last_sent_ms) < VEHICLE_TELEMETRY_PERIOD_MS)) {
		return;
	}
	/* E_TMOUT: the service is not draining (MQTT down). Skip this period;
	 * the sequence is not advanced, so the next snapshot follows on. */
	result = wifi_telemetry_producer_send(&s_producer, data, now_ms);
	s_last_sent_ms = now_ms;
	s_have_sent = true;
	if ((result != E_OK) && (result != E_TMOUT) &&
	    (!s_logged || ((now_ms - s_last_log_ms) >= TELEMETRY_LOG_INTERVAL_MS))) {
		tm_printf((UB*)"vehicle telemetry: snapshot not queued (%d)\n", (int)result);
		s_last_log_ms = now_ms;
		s_logged = true;
	}
}
