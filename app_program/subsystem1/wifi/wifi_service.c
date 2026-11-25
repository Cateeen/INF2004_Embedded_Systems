#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "wifi_service.h"
#include "wifi_mqtt_bridge.h"

#define WIFI_SERVICE_PRIORITY 12   /* below every vehicle task */
/* A communication problem is logged at most this often while it repeats. */
#define WIFI_FAULT_LOG_INTERVAL_MS 5000U
/* Equals the MQTT bridge queue depth; the bridge pushes back when full. */
#define WIFI_SERVICE_MAX_PUBLISH_PER_CYCLE 4U

typedef struct {
	wifi_service_link_t link;
	telemetry_message_t pending;
	uint32_t last_clock_ms;
	uint32_t last_status_ms;
	uint32_t status_sequence;
	uint32_t telemetry_sequence;
	uint32_t telemetry_timestamp_ms;
	ER callback_error;
	bool have_status;
	bool have_pending;
	bool have_telemetry;
} wifi_service_state_t;

static wifi_service_config_t s_config;
static wifi_service_state_t s_state;
static bool s_started;

static bool valid_interval(uint32_t interval)
{
	return interval != 0U && interval <= INT32_MAX;
}

static uint16_t get_clock(wifi_service_state_t *state, uint32_t *now)
{
	SYSTIM time;
	ER er = tk_get_otm(&time);
	if (er != E_OK || time.lo - state->last_clock_ms > INT32_MAX) {
		state->callback_error = er;
		return VEHICLE_FAULT_CODE_WIFI_CLOCK;
	}
	*now = time.lo;
	state->last_clock_ms = *now;
	return 0U;
}

static bool same_link(const wifi_service_link_t *a, const wifi_service_link_t *b)
{
	return a->connected == b->connected && a->mqtt_ready == b->mqtt_ready &&
	       a->retry_count == b->retry_count && a->error_code == b->error_code;
}

static uint16_t publish_status(wifi_service_state_t *state,
			       const wifi_service_config_t *config,
			       const wifi_service_link_t *link, uint32_t now)
{
	network_status_message_t status = {0};
	ER er;

	if (state->have_status && same_link(&state->link, link) &&
	    now - state->last_status_ms < config->status_period_ms)
		return 0U;
	status.header.type = VEHICLE_MESSAGE_NETWORK_STATUS;
	status.header.version = VEHICLE_MESSAGE_VERSION;
	status.header.sequence = state->status_sequence + 1U;
	status.header.timestamp_ms = now;
	status.connected = link->connected;
	status.mqtt_ready = link->mqtt_ready;
	status.retry_count = link->retry_count;
	status.error_code = link->error_code;
	er = tk_snd_mbf(g_mbf_network_status, &status, (INT)sizeof(status), TMO_POL);
	if (er == E_TMOUT) {
		/* Keep-latest: link status is a snapshot, so when no consumer has
		 * drained the queue the oldest entry is discarded, not reported as
		 * a fault. This service is the only producer of this buffer. */
		network_status_message_t oldest;
		(void)tk_rcv_mbf(g_mbf_network_status, &oldest, TMO_POL);
		er = tk_snd_mbf(g_mbf_network_status, &status, (INT)sizeof(status), TMO_POL);
	}
	if (er != E_OK) {
		state->callback_error = er;
		return VEHICLE_FAULT_CODE_WIFI_DELIVERY;
	}
	state->link = *link;
	state->status_sequence = status.header.sequence;
	state->last_status_ms = now;
	state->have_status = true;
	return 0U;
}

static uint16_t receive_telemetry(wifi_service_state_t *state,
				  const wifi_service_config_t *config, uint32_t now)
{
	telemetry_message_t message;
	INT received;

	if (state->have_pending) return 0U;
	received = tk_rcv_mbf(g_mbf_telemetry, &message, TMO_POL);
	if (received == E_TMOUT) return 0U;
	if (received != (INT)sizeof(message) ||
	    message.header.type != VEHICLE_MESSAGE_TELEMETRY ||
	    message.header.version != VEHICLE_MESSAGE_VERSION ||
	    message.payload_length > sizeof(message.payload) ||
	    now - message.header.timestamp_ms > config->telemetry_max_age_ms ||
	    (state->have_telemetry &&
	     (message.header.sequence - state->telemetry_sequence != 1U ||
	      message.header.timestamp_ms - state->telemetry_timestamp_ms > INT32_MAX)))
		return VEHICLE_FAULT_CODE_WIFI_TELEMETRY;
	state->pending = message;
	state->have_pending = true;
	return 0U;
}

static uint16_t service_cycle(wifi_service_state_t *state,
			      const wifi_service_config_t *config)
{
	wifi_service_link_t link = {0};
	uint32_t now;
	uint32_t sent;
	uint16_t fault;
	ER er = config->poll(config->context, &link);

	if (er != E_OK) {
		state->callback_error = er;
		return VEHICLE_FAULT_CODE_WIFI_BACKEND;
	}
	if (link.mqtt_ready && !link.connected)
		return VEHICLE_FAULT_CODE_WIFI_BACKEND;
	fault = get_clock(state, &now);
	if (fault != 0U) return fault;
	fault = publish_status(state, config, &link, now);
	if (fault != 0U) return fault;
	if (state->have_pending &&
	    now - state->pending.header.timestamp_ms > config->telemetry_max_age_ms)
		return VEHICLE_FAULT_CODE_WIFI_TELEMETRY;
	if (!link.mqtt_ready) return 0U;
	/* Forward several messages per cycle, in order, so a backlog left by an
	 * outage drains instead of leaving the dashboard permanently behind
	 * (the producer adds one snapshot per period). Stops at backpressure. */
	for (sent = 0U; sent < WIFI_SERVICE_MAX_PUBLISH_PER_CYCLE; sent++) {
		fault = receive_telemetry(state, config, now);
		if (fault != 0U || !state->have_pending) return fault;
		er = config->publish(config->context, &state->pending);
		if (er == E_TMOUT) return 0U;
		if (er != E_OK) {
			state->callback_error = er;
			return VEHICLE_FAULT_CODE_WIFI_BACKEND;
		}
		state->telemetry_sequence = state->pending.header.sequence;
		state->telemetry_timestamp_ms = state->pending.header.timestamp_ms;
		state->have_telemetry = true;
		state->have_pending = false;
	}
	return 0U;
}

/* Communication is best-effort: a Wi-Fi, MQTT or telemetry problem must
 * never stop the car. The fault is logged (rate-limited), the message in
 * hand is dropped and the service carries on, so telemetry resumes on its
 * own when the link recovers. Nothing is sent to g_mbf_faults, because the
 * vehicle controller latches a safe stop on any fault message. */
static void recover_from_fault(wifi_service_state_t *state, uint16_t code)
{
	static uint16_t s_last_code;
	static uint32_t s_last_log_ms;
	static uint32_t s_suppressed;

	state->have_pending = false;
	state->have_telemetry = false;	/* accept any sequence after recovery */
	if ((code != s_last_code) ||
	    ((state->last_clock_ms - s_last_log_ms) >= WIFI_FAULT_LOG_INTERVAL_MS)) {
		tm_printf((UB*)"WiFi service: communication fault %u (error %d), "
			  "continuing; %u repeats suppressed\n",
			  (unsigned)code, (int)state->callback_error, (unsigned)s_suppressed);
		s_last_code = code;
		s_last_log_ms = state->last_clock_ms;
		s_suppressed = 0U;
	} else {
		s_suppressed++;
	}
	state->callback_error = E_OK;
}

static void wifi_service_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	for (;;) {
		uint16_t fault = service_cycle(&s_state, &s_config);
		ER er;
		if (fault != 0U) recover_from_fault(&s_state, fault);
		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) {
			s_state.callback_error = er;
			recover_from_fault(&s_state, VEHICLE_FAULT_CODE_WIFI_SCHEDULE);
		}
	}
}

ER wifi_service_start(const wifi_service_config_t *config)
{
	T_CTSK task = {0};
	SYSTIM now;
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || !valid_interval(config->period_ms) ||
	    !valid_interval(config->status_period_ms) ||
	    !valid_interval(config->telemetry_max_age_ms) ||
	    config->period_ms > config->status_period_ms ||
	    config->period_ms > config->telemetry_max_age_ms ||
	    config->poll == NULL || config->publish == NULL || config->stop == NULL)
		return E_PAR;
	if (g_mbf_telemetry <= 0 || g_mbf_network_status <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	er = tk_get_otm(&now);
	if (er != E_OK) return er < E_OK ? er : E_SYS;
	s_config = *config;
	s_state = (wifi_service_state_t){0};
	s_state.last_clock_ms = now.lo;
	task.task = (FP)wifi_service_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = WIFI_SERVICE_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
	id = tk_cre_tsk(&task);
	if (id < E_OK) return id;
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK) tm_printf((UB*)"WiFi task cleanup failed: %d\n", (int)cleanup);
		else s_started = false;
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

static ER mqtt_backend_poll(void *context, wifi_service_link_t *link)
{
	const uint32_t *max_age_ms = context;
	return wifi_mqtt_bridge_link(*max_age_ms, &link->connected, &link->mqtt_ready);
}

static ER mqtt_backend_publish(void *context, const telemetry_message_t *message)
{
	(void)context;
	return wifi_mqtt_bridge_submit(message);
}

static ER mqtt_backend_stop(void *context)
{
	(void)context;
	return wifi_mqtt_bridge_stop();
}

ER wifi_service_start_mqtt(RELTIM period_ms, uint32_t status_period_ms,
			   uint32_t telemetry_max_age_ms, uint32_t owner_max_age_ms)
{
	static uint32_t s_owner_max_age_ms;
	wifi_service_config_t config;
	ER result;

	if (s_started) return E_OBJ;
	if (!valid_interval(period_ms) || !valid_interval(status_period_ms) ||
	    !valid_interval(telemetry_max_age_ms) || !valid_interval(owner_max_age_ms) ||
	    period_ms > status_period_ms || period_ms > telemetry_max_age_ms ||
	    period_ms > owner_max_age_ms) return E_PAR;
	if (g_mbf_telemetry <= 0 || g_mbf_network_status <= 0 || g_mbf_faults <= 0)
		return E_OBJ;
	result = wifi_mqtt_bridge_start(telemetry_max_age_ms);
	if (result != E_OK) return result;
	s_owner_max_age_ms = owner_max_age_ms;
	config = (wifi_service_config_t){period_ms, status_period_ms, telemetry_max_age_ms,
					 mqtt_backend_poll, mqtt_backend_publish,
					 mqtt_backend_stop, &s_owner_max_age_ms};
	result = wifi_service_start(&config);
	if (result != E_OK) {
		ER stopped = wifi_mqtt_bridge_stop();
		if (stopped != E_OK)
			tm_printf((UB*)"MQTT bridge stop request failed: %d\n", (int)stopped);
	}
	return result;
}

#if APP_TEST
#include "../../test_report.h"
#include "wifi_command.h"
#include "wifi_telemetry.h"
#include "wifi_telemetry_producer.h"

typedef struct {
	wifi_service_link_t link;
	telemetry_message_t published;
	ER poll_result;
	ER publish_result;
	unsigned publish_calls;
} wifi_test_backend_t;

static ER test_poll(void *context, wifi_service_link_t *link)
{
	wifi_test_backend_t *backend = context;
	*link = backend->link;
	return backend->poll_result;
}

static ER test_publish(void *context, const telemetry_message_t *message)
{
	wifi_test_backend_t *backend = context;
	backend->publish_calls++;
	if (backend->publish_result == E_OK) backend->published = *message;
	return backend->publish_result;
}

static ER test_stop(void *context)
{
	(void)context;
	return E_OK;
}

static BOOL drain_test_buffers(void)
{
	union {
		telemetry_message_t telemetry;
		network_status_message_t status;
	} message;
	INT received;
	BOOL ok;
	do {
		received = tk_rcv_mbf(g_mbf_telemetry, &message, TMO_POL);
	} while (received > 0);
	ok = received == E_TMOUT;
	do {
		received = tk_rcv_mbf(g_mbf_network_status, &message, TMO_POL);
	} while (received > 0);
	return received == E_TMOUT && ok;
}

static BOOL wifi_flow_test(void)
{
	wifi_test_backend_t backend = {0};
	wifi_service_config_t config = {10U, 1000U, 10000U, test_poll, test_publish, test_stop,
					 &backend};
	wifi_service_state_t state = {0};
	telemetry_message_t message = {0};
	network_status_message_t status;
	SYSTIM now;
	BOOL ok;

	if (tk_get_otm(&now) != E_OK) return FALSE;
	state.last_clock_ms = now.lo;
	message.header.type = VEHICLE_MESSAGE_TELEMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = UINT32_MAX;
	message.header.timestamp_ms = now.lo;
	message.payload_length = 2U;
	message.payload[0] = 'O';
	message.payload[1] = 'K';
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK;
	ok = service_cycle(&state, &config) == 0U && backend.publish_calls == 0U &&
	     tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == (INT)sizeof(status) &&
	     !status.connected && !status.mqtt_ready && status.header.sequence == 1U && ok;
	backend.link.connected = true;
	backend.link.mqtt_ready = true;
	backend.publish_result = E_TMOUT;
	ok = service_cycle(&state, &config) == 0U && state.have_pending &&
	     backend.publish_calls == 1U &&
	     tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == (INT)sizeof(status) &&
	     status.connected && status.mqtt_ready && status.header.sequence == 2U && ok;
	backend.publish_result = E_OK;
	ok = service_cycle(&state, &config) == 0U && !state.have_pending &&
	     backend.publish_calls == 2U && backend.published.header.sequence == UINT32_MAX &&
	     backend.published.payload[0] == 'O' && backend.published.payload[1] == 'K' && ok;
	message.header.sequence = 0U;
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     service_cycle(&state, &config) == 0U && backend.publish_calls == 3U &&
	     backend.published.header.sequence == 0U && ok;
	state.last_status_ms = UINT32_MAX - 100U;
	ok = publish_status(&state, &config, &backend.link, 898U) == 0U &&
	     tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == E_TMOUT && ok;
	ok = publish_status(&state, &config, &backend.link, 899U) == 0U &&
	     tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == (INT)sizeof(status) &&
	     status.header.sequence == 3U && status.header.timestamp_ms == 899U && ok;
	return drain_test_buffers() && ok;
}

static BOOL wifi_fault_test(void)
{
	wifi_test_backend_t backend = {0};
	wifi_service_config_t config = {10U, 1000U, 10000U, test_poll, test_publish, test_stop,
					 &backend};
	wifi_service_state_t state = {0};
	telemetry_message_t message = {0};
	network_status_message_t status = {0};
	SYSTIM now;
	BOOL ok;
	unsigned i;

	if (tk_get_otm(&now) != E_OK) return FALSE;
	state.last_clock_ms = now.lo;
	backend.link.mqtt_ready = true;
	ok = service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_BACKEND;
	backend.link.connected = true;
	message.header.type = VEHICLE_MESSAGE_TELEMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = 1U;
	message.header.timestamp_ms = now.lo;
	message.payload_length = VEHICLE_TELEMETRY_PAYLOAD_SIZE + 1U;
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_TELEMETRY && ok;
	ok = drain_test_buffers() && ok;
	state = (wifi_service_state_t){0};
	state.last_clock_ms = now.lo;
	message.payload_length = 0U;
	message.header.timestamp_ms = now.lo - config.telemetry_max_age_ms - 1U;
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_TELEMETRY && ok;
	ok = drain_test_buffers() && ok;
	state = (wifi_service_state_t){0};
	state.last_clock_ms = now.lo;
	/* A full status queue with no consumer keeps the latest status instead
	 * of faulting: the oldest entry is replaced. */
	for (i = 0U; i < VEHICLE_MB_NETWORK_CAPACITY; i++)
		ok = tk_snd_mbf(g_mbf_network_status, &status, (INT)sizeof(status), TMO_POL) ==
		     E_OK && ok;
	ok = service_cycle(&state, &config) == 0U && state.have_status &&
	     state.status_sequence == 1U && ok;
	for (i = 0U; i < VEHICLE_MB_NETWORK_CAPACITY; i++)
		ok = tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == (INT)sizeof(status) && ok;
	ok = status.header.sequence == 1U && status.connected && status.mqtt_ready &&
	     tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == E_TMOUT && ok;
	status = (network_status_message_t){0};
	ok = drain_test_buffers() && ok;
	state = (wifi_service_state_t){0};
	state.last_clock_ms = now.lo;
	message.header.timestamp_ms = now.lo;
	backend.publish_result = E_TMOUT;
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     service_cycle(&state, &config) == 0U && state.have_pending && ok;
	state.pending.header.timestamp_ms = now.lo - config.telemetry_max_age_ms - 1U;
	ok = service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_TELEMETRY &&
	     state.have_pending && ok;
	ok = drain_test_buffers() && ok;
	state = (wifi_service_state_t){0};
	state.last_clock_ms = now.lo;
	backend.publish_result = E_SYS;
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_BACKEND &&
	     state.have_pending && state.callback_error == E_SYS && ok;
	ok = drain_test_buffers() && ok;
	state = (wifi_service_state_t){0};
	state.last_clock_ms = now.lo;
	state.have_telemetry = true;
	state.telemetry_sequence = 5U;
	state.telemetry_timestamp_ms = now.lo;
	message.header.sequence = 7U;
	ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) == E_OK &&
	     service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_TELEMETRY && ok;
	ok = drain_test_buffers() && ok;
	state = (wifi_service_state_t){0};
	state.last_clock_ms = now.lo;
	backend.poll_result = E_SYS;
	ok = service_cycle(&state, &config) == VEHICLE_FAULT_CODE_WIFI_BACKEND &&
	     state.callback_error == E_SYS && ok;
	return drain_test_buffers() && ok;
}

static BOOL wifi_telemetry_test(void)
{
	wifi_telemetry_data_t data = {WIFI_TELEMETRY_VEHICLE_RUNNING,
		WIFI_TELEMETRY_LINE_FOUND, WIFI_TELEMETRY_OBSTACLE_DETECTED,
		WIFI_TELEMETRY_TERRAIN_FLAT,
		WIFI_TELEMETRY_FAULT_MOTOR_STALL | WIFI_TELEMETRY_FAULT_WIFI_LOSS, 100U};
	telemetry_message_t message;
	const uint8_t *packet = (const uint8_t *)message.payload;
	BOOL ok;
	unsigned i, j;

	ok = wifi_telemetry_make(&data, 128U, UINT32_MAX, &message) &&
	     message.header.type == VEHICLE_MESSAGE_TELEMETRY &&
	     message.header.version == VEHICLE_MESSAGE_VERSION &&
	     message.header.sequence == 128U && message.header.timestamp_ms == UINT32_MAX &&
	     message.payload_length == WIFI_TELEMETRY_PACKET_SIZE &&
	     packet[0] == 0x05U && packet[1] == 0x91U && packet[2] == 0x01U &&
	     packet[3] == 100U && packet[4] == 0U &&
	     wifi_telemetry_packet_valid(packet, message.payload_length);
	data.vehicle_state = WIFI_TELEMETRY_VEHICLE_COMPLETE;
	data.line_state = WIFI_TELEMETRY_LINE_LOST;
	data.obstacle_state = WIFI_TELEMETRY_OBSTACLE_DETECTED;
	data.terrain_state = WIFI_TELEMETRY_TERRAIN_HUMP;
	data.fault_flags = WIFI_TELEMETRY_FAULT_ALL;
	ok = wifi_telemetry_make(&data, 127U, 0U, &message) &&
	     packet[0] == 0x12U && packet[1] == 0xafU && packet[2] == 0xffU && ok;
	for (i = 0U; i < sizeof(message); i++) ((uint8_t *)&message)[i] = 0xa5U;
	for (i = 0U; i < 6U; i++) {
		wifi_telemetry_data_t invalid = data;
		switch (i) {
		case 0: invalid.vehicle_state = 5U; break;
		case 1: invalid.line_state = 3U; break;
		case 2: invalid.obstacle_state = 3U; break;
		case 3: invalid.terrain_state = 3U; break;
		case 4: invalid.fault_flags = 32U; break;
		default: invalid.battery_pct = 101U; break;
		}
		ok = !wifi_telemetry_make(&invalid, 0U, 0U, &message) && ok;
	}
	ok = !wifi_telemetry_make(NULL, 0U, 0U, &message) &&
	     !wifi_telemetry_make(&data, 0U, 0U, NULL) && ok;
	for (j = 0U; j < sizeof(message); j++)
		ok = ((const uint8_t *)&message)[j] == 0xa5U && ok;
	{
		uint8_t bytes[WIFI_TELEMETRY_PACKET_SIZE] = {0};
		ok = wifi_telemetry_packet_valid(bytes, sizeof(bytes)) &&
		     !wifi_telemetry_packet_valid(NULL, sizeof(bytes)) &&
		     !wifi_telemetry_packet_valid(bytes, sizeof(bytes) - 1U) && ok;
		for (i = 0U; i < 7U; i++) {
			bytes[0] = bytes[1] = bytes[3] = bytes[4] = 0U;
			switch (i) {
			case 0: bytes[0] = 0x20U; break;
			case 1: bytes[0] = 0x14U; break;
			case 2: bytes[0] = 0x03U; break;
			case 3: bytes[1] = 0xc0U; break;
			case 4: bytes[1] = 0x30U; break;
			case 5: bytes[3] = 101U; break;
			default: bytes[4] = 1U; break;
			}
			ok = !wifi_telemetry_packet_valid(bytes, sizeof(bytes)) && ok;
		}
	}
	return ok;
}

static BOOL wifi_producer_test(void)
{
	wifi_telemetry_producer_t producer = {0};
	wifi_telemetry_data_t data = {0};
	wifi_test_backend_t backend = {0};
	wifi_service_config_t config = {10U, 1000U, 10000U, test_poll, test_publish,
					 test_stop, &backend};
	wifi_service_state_t state = {0};
	telemetry_message_t message = {0};
	network_status_message_t status;
	SYSTIM now;
	BOOL ok;
	unsigned i;

	if (tk_get_otm(&now) != E_OK) return FALSE;
	ok = wifi_telemetry_producer_init(&producer, UINT32_MAX, 0U) == E_PAR &&
	     wifi_telemetry_producer_send(&producer, &data, now.lo) == E_OBJ &&
	     wifi_telemetry_producer_init(&producer, UINT32_MAX, 10000U) == E_OK &&
	     wifi_telemetry_producer_init(&producer, 1U, 10000U) == E_OBJ;
	data.battery_pct = 101U;
	ok = wifi_telemetry_producer_send(&producer, &data, now.lo) == E_PAR &&
	     producer.next_sequence == UINT32_MAX && ok;
	data.battery_pct = 80U;
	ok = wifi_telemetry_producer_send(&producer, &data, now.lo - 11000U) == E_PAR &&
	     wifi_telemetry_producer_send(&producer, &data, now.lo + 1000000U) == E_PAR && ok;
	for (i = 0U; i < VEHICLE_MB_TELEMETRY_CAPACITY; i++)
		ok = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message), TMO_POL) ==
		     E_OK && ok;
	ok = wifi_telemetry_producer_send(&producer, &data, now.lo) == E_TMOUT &&
	     producer.next_sequence == UINT32_MAX && !producer.have_sent && ok;
	ok = drain_test_buffers() && ok;
	backend.link.connected = true;
	backend.link.mqtt_ready = true;
	state.last_clock_ms = now.lo;
	ok = wifi_telemetry_producer_send(&producer, &data, now.lo) == E_OK &&
	     service_cycle(&state, &config) == 0U && backend.publish_calls == 1U &&
	     backend.published.header.sequence == UINT32_MAX &&
	     (uint8_t)backend.published.payload[2] == 0xfeU &&
	     (uint8_t)backend.published.payload[3] == 80U &&
	     tk_rcv_mbf(g_mbf_network_status, &status, TMO_POL) == (INT)sizeof(status) && ok;
	ok = wifi_telemetry_producer_send(&producer, &data, now.lo - 1U) == E_PAR &&
	     producer.next_sequence == 0U &&
	     wifi_telemetry_producer_send(&producer, &data, now.lo) == E_OK &&
	     service_cycle(&state, &config) == 0U && backend.publish_calls == 2U &&
	     backend.published.header.sequence == 0U &&
	     (uint8_t)backend.published.payload[2] == 0U && ok;
	return drain_test_buffers() && ok;
}

static BOOL wifi_command_test(void)
{
	wifi_command_t command = {99U, 88U, 77U};
	uint8_t packet[WIFI_COMMAND_PACKET_SIZE] = {0x80U, 44U};
	BOOL ok;
	unsigned type;

	ok = wifi_command_decode(packet, sizeof(packet), &command) &&
	     command.type == WIFI_COMMAND_STOP && command.argument == 0U &&
	     command.sequence == 44U;
	packet[0] = 0xc4U;
	packet[1] = 255U;
	ok = wifi_command_decode(packet, sizeof(packet), &command) &&
	     command.type == WIFI_COMMAND_JUNCTION && command.argument == 4U &&
	     command.sequence == 255U && ok;
	for (type = WIFI_COMMAND_START; type <= WIFI_COMMAND_JUNCTION; type++) {
		packet[0] = (uint8_t)((type << 5U) | 31U);
		packet[1] = 0U;
		ok = wifi_command_decode(packet, sizeof(packet), &command) &&
		     command.type == type && command.argument == 31U && command.sequence == 0U && ok;
	}
	packet[0] = 0xc4U;
	packet[1] = 255U;
	ok = wifi_command_decode(packet, sizeof(packet), &command) && ok;
	packet[0] = 0U;
	ok = !wifi_command_decode(packet, sizeof(packet), &command) &&
	     command.type == WIFI_COMMAND_JUNCTION && command.argument == 4U &&
	     command.sequence == 255U && ok;
	packet[0] = 0xe0U;
	ok = !wifi_command_decode(packet, sizeof(packet), &command) &&
	     !wifi_command_decode(packet, 1U, &command) &&
	     !wifi_command_decode(packet, 3U, &command) &&
	     !wifi_command_decode(NULL, sizeof(packet), &command) &&
	     !wifi_command_decode(packet, sizeof(packet), NULL) &&
	     command.type == WIFI_COMMAND_JUNCTION && command.argument == 4U &&
	     command.sequence == 255U && ok;
	return ok;
}

BOOL wifi_service_test(void)
{
	BOOL flow_ok;
	BOOL fault_ok;
	BOOL command_ok;
	BOOL telemetry_ok;
	BOOL producer_ok;
	TEST_INIT();

	flow_ok = wifi_flow_test();
	fault_ok = wifi_fault_test();
	command_ok = wifi_command_test();
	telemetry_ok = wifi_telemetry_test();
	producer_ok = wifi_producer_test();
	TEST_CHECK("WIFI-U01", flow_ok, "link status and telemetry backpressure");
	TEST_CHECK("WIFI-U02", fault_ok, "invalid telemetry and link failures; full status keeps latest");
	TEST_CHECK("WIFI-U03", command_ok, "legacy command decode and malformed packets");
	TEST_CHECK("WIFI-U06", telemetry_ok, "legacy telemetry encoding and validation");
	TEST_CHECK("WIFI-U07", producer_ok, "telemetry producer and service queue");
	TEST_SUMMARY("WiFi service");
	return flow_ok && fault_ok && command_ok && telemetry_ok && producer_ok;
}
#endif
