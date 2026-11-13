#include "../app.h"
#include "../vehicle_buffers.h"
#include "wifi_mqtt_bridge.h"
#include "wifi_mqtt_bridge_owner.h"
#include "wifi_telemetry.h"

#define BRIDGE_CAPACITY 4U
#define BRIDGE_WORDS (BRIDGE_CAPACITY * \
	(1U + ((sizeof(telemetry_message_t) + sizeof(INT) - 1U) / sizeof(INT))))

typedef struct {
	telemetry_message_t pending;
	uint32_t max_age_ms;
	uint32_t last_clock_ms;
	bool have_pending;
} bridge_state_t;

typedef struct {
	uint32_t sequence;
	uint32_t time;
	uint32_t flags;
} bridge_snapshot_t;

#if TM_MQTT
static INT s_storage[BRIDGE_WORDS];
#endif
static ID s_buffer;
static bridge_state_t s_bridge;
/* Published last by the single startup owner; only the network owner faults it. */
static int s_status;
static int s_stop_requested;
static uint32_t s_started_at;
/* One network-owner writer; the sequence prevents mixed cross-core snapshots. */
static bridge_snapshot_t s_snapshot;
static bool s_fault_sent;
static bool s_fault_delivery_reported;

static bool valid_packet(const telemetry_message_t *message)
{
	return message->header.type == VEHICLE_MESSAGE_TELEMETRY &&
	       message->header.version == VEHICLE_MESSAGE_VERSION &&
	       wifi_telemetry_packet_valid((const uint8_t *)message->payload,
					   message->payload_length);
}

static void snapshot_write(bridge_snapshot_t *snapshot, uint32_t now, uint32_t flags)
{
	uint32_t sequence = __atomic_load_n(&snapshot->sequence, __ATOMIC_SEQ_CST);
	sequence = sequence >= UINT32_MAX - 1U ? 2U : sequence + 2U;
	__atomic_store_n(&snapshot->sequence, sequence - 1U, __ATOMIC_SEQ_CST);
	__atomic_store_n(&snapshot->time, now, __ATOMIC_SEQ_CST);
	__atomic_store_n(&snapshot->flags, flags, __ATOMIC_SEQ_CST);
	__atomic_store_n(&snapshot->sequence, sequence, __ATOMIC_SEQ_CST);
}

static bool snapshot_read(const bridge_snapshot_t *snapshot, uint32_t started_at,
			  uint32_t *timestamp, uint32_t *flags)
{
	uint32_t before, after;
	unsigned i;

	for (i = 0; i < 3U; i++) {
		before = __atomic_load_n(&snapshot->sequence, __ATOMIC_SEQ_CST);
		if (before & 1U) continue;
		*timestamp = __atomic_load_n(&snapshot->time, __ATOMIC_SEQ_CST);
		*flags = __atomic_load_n(&snapshot->flags, __ATOMIC_SEQ_CST);
		after = __atomic_load_n(&snapshot->sequence, __ATOMIC_SEQ_CST);
		if (before != after) continue;
		if (before == 0U) {
			*timestamp = started_at;
			*flags = 0U;
		}
		return true;
	}
	return false;
}

static ER bridge_cycle(bridge_state_t *state, ID buffer, uint32_t now,
		       int connected, wifi_mqtt_bridge_publish_t publish)
{
	INT received;
	int result;

	if (!state->have_pending && connected) {
		received = tk_rcv_mbf(buffer, &state->pending, TMO_POL);
		if (received == E_TMOUT) return E_OK;
		if (received != (INT)sizeof(state->pending) || !valid_packet(&state->pending))
			return E_SYS;
		state->have_pending = true;
	}
	if (!state->have_pending) return E_OK;
	if (now - state->pending.header.timestamp_ms > state->max_age_ms)
		return E_SYS;
	if (!connected) return E_OK;
	result = publish((const uint8_t *)state->pending.payload);
	if (result == WIFI_MQTT_PUBLISH_BUSY) return E_OK;
	if (result != WIFI_MQTT_PUBLISH_OK) return E_SYS;
	state->have_pending = false;
	return E_OK;
}

ER wifi_mqtt_bridge_start(uint32_t max_age_ms)
{
#if TM_MQTT
	T_CMBF config = {0};
	SYSTIM now;
	ID buffer;
	ER result;

	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 0) return E_OBJ;
	if (max_age_ms == 0U || max_age_ms > INT32_MAX) return E_PAR;
	if (g_mbf_faults <= 0) return E_OBJ;
	result = tk_get_otm(&now);
	if (result != E_OK) return result < E_OK ? result : E_SYS;
	config.mbfatr = TA_TFIFO | TA_USERBUF;
	config.bufsz = (SZ)sizeof(s_storage);
	config.maxmsz = (INT)sizeof(telemetry_message_t);
	config.bufptr = s_storage;
	buffer = tk_cre_mbf(&config);
	if (buffer < E_OK) return buffer;
	s_buffer = buffer;
	s_bridge.max_age_ms = max_age_ms;
	s_bridge.last_clock_ms = now.lo;
	s_started_at = now.lo;
	__atomic_store_n(&s_status, 1, __ATOMIC_RELEASE);
	return E_OK;
#else
	(void)max_age_ms;
	return E_NOSPT;
#endif
}

ER wifi_mqtt_bridge_submit(const telemetry_message_t *message)
{
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 1) return E_OBJ;
	if (__atomic_load_n(&s_stop_requested, __ATOMIC_ACQUIRE)) return E_OBJ;
	if (message == NULL || !valid_packet(message)) return E_PAR;
	return tk_snd_mbf(s_buffer, message, (INT)sizeof(*message), TMO_POL);
}

ER wifi_mqtt_bridge_link(uint32_t max_age_ms, bool *connected, bool *mqtt_ready)
{
	uint32_t timestamp, flags;
	SYSTIM now;
	ER result;

	if (max_age_ms == 0U || max_age_ms > INT32_MAX ||
	    connected == NULL || mqtt_ready == NULL) return E_PAR;
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 1 ||
	    __atomic_load_n(&s_stop_requested, __ATOMIC_ACQUIRE)) return E_OBJ;
	if (!snapshot_read(&s_snapshot, s_started_at, &timestamp, &flags)) return E_TMOUT;
	result = tk_get_otm(&now);
	if (result != E_OK) return result < E_OK ? result : E_SYS;
	if (now.lo - timestamp > max_age_ms) return E_SYS;
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 1 ||
	    __atomic_load_n(&s_stop_requested, __ATOMIC_ACQUIRE)) return E_OBJ;
	*connected = (flags & 1U) != 0U;
	*mqtt_ready = (flags & 2U) != 0U;
	return E_OK;
}

ER wifi_mqtt_bridge_stop(void)
{
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) == 0) return E_OBJ;
	__atomic_store_n(&s_stop_requested, 1, __ATOMIC_RELEASE);
	return E_OK;
}

void wifi_mqtt_bridge_publish_failed(void)
{
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) == 1) {
		__atomic_store_n(&s_status, -1, __ATOMIC_RELEASE);
		tm_printf((UB*)"MQTT telemetry bridge stopped: publish failure\n");
	}
}

void wifi_mqtt_bridge_poll(wifi_mqtt_bridge_publish_t publish, int link_up, int mqtt_ready)
{
	int status = __atomic_load_n(&s_status, __ATOMIC_ACQUIRE);
	SYSTIM now;
	ER result;
	fault_message_t fault = {0};

	if (status == 0) return;
	if (status == 1 && __atomic_load_n(&s_stop_requested, __ATOMIC_ACQUIRE)) return;
	if (status == 1) {
		result = tk_get_otm(&now);
		if (result != E_OK || publish == NULL ||
		    (mqtt_ready && !link_up) ||
		    now.lo - s_bridge.last_clock_ms > INT32_MAX ||
		    bridge_cycle(&s_bridge, s_buffer, now.lo, mqtt_ready, publish) != E_OK) {
			__atomic_store_n(&s_status, -1, __ATOMIC_RELEASE);
			tm_printf((UB*)"MQTT telemetry bridge stopped: clock, message or publish error\n");
			status = -1;
		} else {
			s_bridge.last_clock_ms = now.lo;
			snapshot_write(&s_snapshot, now.lo,
				       (link_up ? 1U : 0U) | (mqtt_ready ? 2U : 0U));
		}
	}
	if (status == -1 && !s_fault_sent) {
		fault.header.type = VEHICLE_MESSAGE_FAULT;
		fault.header.version = VEHICLE_MESSAGE_VERSION;
		fault.header.sequence = 1U;
		if (tk_get_otm(&now) != E_OK) return;
		fault.header.timestamp_ms = now.lo;
		fault.source = VEHICLE_FAULT_COMMUNICATION;
		fault.code = VEHICLE_FAULT_CODE_WIFI_BACKEND;
		result = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
		if (result == E_OK) s_fault_sent = true;
		else if (!s_fault_delivery_reported) {
			tm_printf((UB*)"MQTT bridge fault delivery failed: %d\n", (int)result);
			s_fault_delivery_reported = true;
		}
	}
}

#if APP_TEST
#include "../test_report.h"

static int s_test_publish_result;
static unsigned s_test_calls;
static uint8_t s_test_payload[WIFI_TELEMETRY_PACKET_SIZE];

static int test_publish(const uint8_t *payload)
{
	unsigned i;
	s_test_calls++;
	if (s_test_publish_result == WIFI_MQTT_PUBLISH_OK) {
		for (i = 0; i < WIFI_TELEMETRY_PACKET_SIZE; i++) s_test_payload[i] = payload[i];
	}
	return s_test_publish_result;
}

BOOL wifi_mqtt_bridge_test(void)
{
	static INT storage[BRIDGE_WORDS];
	T_CMBF config = {0};
	bridge_state_t state = {0};
	telemetry_message_t message = {0};
	wifi_telemetry_data_t data = {0};
	ID buffer;
	BOOL ok;
	BOOL snapshot_ok;
	bridge_snapshot_t snapshot = {0};
	uint32_t timestamp, flags;
	unsigned i;
	TEST_INIT();

	config.mbfatr = TA_TFIFO | TA_USERBUF;
	config.bufsz = (SZ)sizeof(storage);
	config.maxmsz = (INT)sizeof(message);
	config.bufptr = storage;
	buffer = tk_cre_mbf(&config);
	if (buffer < E_OK) {
		TEST_CHECK("WIFI-U04", FALSE, "MQTT outbound bridge queue");
		TEST_SUMMARY("MQTT bridge");
		return FALSE;
	}
	state.max_age_ms = 20U;
	data.battery_pct = 100U;
	s_test_calls = 0U;
	s_test_publish_result = WIFI_MQTT_PUBLISH_BUSY;
	ok = wifi_telemetry_make(&data, 0U, UINT32_MAX - 5U, &message) &&
	     valid_packet(&message) &&
	     tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_OK;
	message.payload[3] = 0;
	ok = bridge_cycle(&state, buffer, UINT32_MAX - 4U, 0, test_publish) == E_OK &&
	     s_test_calls == 0U && ok;
	ok = bridge_cycle(&state, buffer, UINT32_MAX - 4U, 1, test_publish) == E_OK &&
	     state.have_pending && s_test_calls == 1U && ok;
	s_test_publish_result = WIFI_MQTT_PUBLISH_OK;
	ok = bridge_cycle(&state, buffer, 3U, 1, test_publish) == E_OK &&
	     !state.have_pending && s_test_calls == 2U && s_test_payload[3] == 100U && ok;
	message.header.timestamp_ms = 10U;
	for (i = 0; i < BRIDGE_CAPACITY; i++)
		ok = tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_OK && ok;
	ok = tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_TMOUT && ok;
	for (i = 0; i < BRIDGE_CAPACITY; i++)
		ok = bridge_cycle(&state, buffer, 10U, 1, test_publish) == E_OK && ok;
	message.payload[4] = 1;
	ok = !valid_packet(&message) && ok;
	message.payload[4] = 0;
	message.payload[0] = (char)0x20;
	ok = !valid_packet(&message) && ok;
	message.payload[0] = 0;
	message.payload_length = 4U;
	ok = !valid_packet(&message) && ok;
	message.payload_length = WIFI_TELEMETRY_PACKET_SIZE;
	ok = tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     bridge_cycle(&state, buffer, 31U, 1, test_publish) == E_SYS && ok;
	state.have_pending = false;
	message.header.timestamp_ms = 50U;
	s_test_publish_result = WIFI_MQTT_PUBLISH_BUSY;
	ok = tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     bridge_cycle(&state, buffer, 50U, 1, test_publish) == E_OK &&
	     bridge_cycle(&state, buffer, 71U, 0, test_publish) == E_SYS && ok;
	state.have_pending = false;
	message.header.timestamp_ms = 80U;
	ok = tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     bridge_cycle(&state, buffer, 79U, 1, test_publish) == E_SYS && ok;
	state.have_pending = false;
	s_test_publish_result = WIFI_MQTT_PUBLISH_ERROR;
	message.header.timestamp_ms = 40U;
	ok = tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     bridge_cycle(&state, buffer, 40U, 1, test_publish) == E_SYS &&
	     state.have_pending && ok;
	ok = tk_del_mbf(buffer) == E_OK && ok;
	snapshot_ok = snapshot_read(&snapshot, 19U, &timestamp, &flags) &&
		      timestamp == 19U && flags == 0U;
	snapshot_write(&snapshot, 21U, 1U);
	snapshot_ok = snapshot_read(&snapshot, 19U, &timestamp, &flags) &&
		      timestamp == 21U && flags == 1U && snapshot_ok;
	snapshot.sequence = UINT32_MAX - 1U;
	snapshot_write(&snapshot, 22U, 3U);
	snapshot_ok = snapshot_read(&snapshot, 19U, &timestamp, &flags) &&
		      timestamp == 22U && flags == 3U && snapshot_ok;
	snapshot.sequence = 1U;
	snapshot_ok = !snapshot_read(&snapshot, 19U, &timestamp, &flags) && snapshot_ok;
	TEST_CHECK("WIFI-U04", ok, "MQTT bridge copied queue, retry and failures");
	TEST_CHECK("WIFI-U05", snapshot_ok, "network owner link snapshot");
	TEST_SUMMARY("MQTT bridge");
	return ok && snapshot_ok;
}
#endif
