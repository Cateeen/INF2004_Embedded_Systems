/*
 *	dashboard_link.c
 *	Queue between application tasks and the MQTT network owner for the
 *	dashboard topics (see dashboard_link.h and dashboard_link_owner.h).
 *
 *	Concurrency
 *	  Producers: any application task, either core -> tk_snd_mbf(TMO_POL).
 *	  Consumer:  cyw43_utk_task only -> tk_rcv_mbf(TMO_POL), same pattern
 *	             as wifi_mqtt_bridge.c.
 *	  Junction turn, run command and vehicle state are single words read
 *	  and written with __atomic builtins, so no lock is needed across cores.
 */

#include "../../app.h"
#include "dashboard_link.h"
#include "dashboard_link_owner.h"
#include "wifi_command.h"
#include "wifi_mqtt_bridge_owner.h"
#include "wifi_telemetry.h"

#define PUBLISH_PER_POLL     2U
#define RUN_COMMAND_SHIFT    8U
#define RUN_COMMAND_MASK     0xffU

typedef struct {
	uint8_t topic;		/* dashboard_topic_t */
	uint8_t reserved;
	uint16_t length;
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
} link_message_t;

typedef struct {
	link_message_t pending;
	bool have_pending;
} link_owner_t;

/* Message-buffer storage: each entry also needs a length word. */
#define LINK_WORDS (DASHBOARD_LINK_QUEUE_CAPACITY * \
	(1U + ((sizeof(link_message_t) + sizeof(INT) - 1U) / sizeof(INT))))

#if defined(TM_MQTT) && TM_MQTT
static INT s_storage[LINK_WORDS];
#endif
static ID s_buffer;
static int s_status;			/* 0 not started, 1 running */
static link_owner_t s_owner;		/* network owner only */
static uint8_t s_heartbeat_sequence;	/* network owner only */
static uint8_t s_next_turn = (uint8_t)DASHBOARD_DEFAULT_TURN;
static uint8_t s_vehicle_state = (uint8_t)WIFI_TELEMETRY_VEHICLE_IDLE;
/* (sequence << 8) | command, so both change in one atomic store. */
static uint32_t s_run_command;
static uint32_t s_run_taken;		/* consumer only */
static uint32_t s_command_count;	/* written by the network owner only */

/* ---- Shared helpers -------------------------------------------------------- */

static bool text_equal(const char *a, const char *b)
{
	while ((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return *a == *b;
}

static ER submit_to(ID buffer, uint8_t topic, const char *payload, uint16_t length)
{
	link_message_t message;
	uint16_t i;

	if ((length == 0U) || (length > DASHBOARD_MSG_PAYLOAD_MAX)) {
		return E_PAR;
	}
	message.topic = topic;
	message.reserved = 0U;
	message.length = length;
	for (i = 0U; i < length; i++) {
		message.payload[i] = payload[i];
	}
	return tk_snd_mbf(buffer, &message, (INT)sizeof(message), TMO_POL);
}

static ER submit(dashboard_topic_t topic, const char *payload, uint16_t length)
{
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 1) {
		return E_OBJ;
	}
	return submit_to(s_buffer, (uint8_t)topic, payload, length);
}

/* Drains up to PUBLISH_PER_POLL messages. BUSY keeps the message for the
 * next poll; ERROR drops it so one bad publish cannot wedge the queue. */
static void owner_cycle(link_owner_t *owner, ID buffer, dashboard_publish_t publish)
{
	uint32_t sent;
	INT received;
	int result;

	for (sent = 0U; sent < PUBLISH_PER_POLL; sent++) {
		if (!owner->have_pending) {
			received = tk_rcv_mbf(buffer, &owner->pending, TMO_POL);
			if (received != (INT)sizeof(owner->pending)) {
				return;
			}
			owner->have_pending = true;
		}
		result = publish(dashboard_msg_topic(owner->pending.topic),
				 (const uint8_t *)owner->pending.payload,
				 owner->pending.length);
		if (result == WIFI_MQTT_PUBLISH_BUSY) {
			return;
		}
		owner->have_pending = false;
	}
}

/* ---- Startup ------------------------------------------------------------------ */

ER dashboard_link_start(void)
{
#if defined(TM_MQTT) && TM_MQTT
	T_CMBF config = {0};
	ID buffer;

	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 0) {
		return E_OBJ;
	}
	config.mbfatr = TA_TFIFO | TA_USERBUF;
	config.bufsz = (SZ)sizeof(s_storage);
	config.maxmsz = (INT)sizeof(link_message_t);
	config.bufptr = s_storage;
	buffer = tk_cre_mbf(&config);
	if (buffer < E_OK) {
		return buffer;
	}
	s_buffer = buffer;
	__atomic_store_n(&s_status, 1, __ATOMIC_RELEASE);
	return E_OK;
#else
	return E_NOSPT;
#endif
}

/* ---- Reporting API --------------------------------------------------------- */

ER dashboard_report_barcode_detected(void)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload,
		      dashboard_msg_simple_event(DASHBOARD_EVENT_BARCODE_DETECTED, payload));
}

ER dashboard_report_barcode(char letter)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_BARCODE, payload, dashboard_msg_barcode(letter, payload));
}

ER dashboard_report_action_done(dashboard_turn_t turn)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload, dashboard_msg_action_done(turn, payload));
}

ER dashboard_report_pickup(void)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload,
		      dashboard_msg_simple_event(DASHBOARD_EVENT_PICKUP_STOP, payload));
}

void dashboard_set_vehicle_state(uint8_t vehicle_state)
{
	if (vehicle_state <= (uint8_t)WIFI_TELEMETRY_VEHICLE_COMPLETE) {
		__atomic_store_n(&s_vehicle_state, vehicle_state, __ATOMIC_RELAXED);
	}
}

ER dashboard_report_hump_height(uint16_t height_mm)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_DETAIL, payload, dashboard_msg_hump_height(height_mm, payload));
}

ER dashboard_report_hump_peak(uint16_t peak_mm)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload, dashboard_msg_hump_peak(peak_mm, payload));
}

ER dashboard_report_obstacle(uint16_t distance_mm)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload, dashboard_msg_obstacle(distance_mm, payload));
}

ER dashboard_report_obstacle_profile(uint16_t width_mm, uint16_t clear_left_mm,
				     uint16_t clear_right_mm, dashboard_side_t bypass_side)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload,
		      dashboard_msg_obstacle_profile(width_mm, clear_left_mm, clear_right_mm,
						     bypass_side, payload));
}

ER dashboard_report_avoid_phase(dashboard_avoid_phase_t phase)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload, dashboard_msg_avoid_phase(phase, payload));
}

ER dashboard_report_line_rejoined(void)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload,
		      dashboard_msg_simple_event(DASHBOARD_EVENT_LINE_REJOINED, payload));
}

ER dashboard_report_line_event(dashboard_line_event_t event)
{
	char payload[DASHBOARD_MSG_PAYLOAD_MAX];
	return submit(DASHBOARD_TOPIC_EVENT, payload, dashboard_msg_line_event(event, payload));
}

/* ---- From the dashboard ----------------------------------------------------- */

dashboard_turn_t dashboard_next_junction(void)
{
	return (dashboard_turn_t)__atomic_load_n(&s_next_turn, __ATOMIC_ACQUIRE);
}

void dashboard_junction_reset(void)
{
	__atomic_store_n(&s_next_turn, (uint8_t)DASHBOARD_DEFAULT_TURN, __ATOMIC_RELEASE);
}

uint32_t dashboard_command_count(void)
{
	return __atomic_load_n(&s_command_count, __ATOMIC_ACQUIRE);
}

bool dashboard_take_run_command(dashboard_run_command_t *command)
{
	uint32_t latest = __atomic_load_n(&s_run_command, __ATOMIC_ACQUIRE);

	if ((command == NULL) || (latest == s_run_taken)) {
		return false;
	}
	s_run_taken = latest;
	*command = (dashboard_run_command_t)(latest & RUN_COMMAND_MASK);
	return true;
}

static void record_run_command(uint8_t type)
{
	uint32_t previous = __atomic_load_n(&s_run_command, __ATOMIC_RELAXED);
	uint32_t sequence = (previous >> RUN_COMMAND_SHIFT) + 1U;

	__atomic_store_n(&s_run_command, (sequence << RUN_COMMAND_SHIFT) | type,
			 __ATOMIC_RELEASE);
}

/* ---- Network-owner side ----------------------------------------------------- */

void dashboard_link_owner_poll(dashboard_publish_t publish, int mqtt_ready)
{
	if ((publish == NULL) || (mqtt_ready == 0) ||
	    (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 1)) {
		return;
	}
	owner_cycle(&s_owner, s_buffer, publish);
}

void dashboard_link_owner_on_message(const char *topic, const uint8_t *data,
				     uint16_t length)
{
	wifi_command_t command;

	if ((topic == NULL) || !text_equal(topic, DASHBOARD_COMMAND_TOPIC) ||
	    !wifi_command_decode(data, length, &command)) {
		return;
	}
	__atomic_store_n(&s_command_count,
			 __atomic_load_n(&s_command_count, __ATOMIC_RELAXED) + 1U,
			 __ATOMIC_RELEASE);
	switch (command.type) {
	case WIFI_COMMAND_JUNCTION:
		if (dashboard_msg_turn_valid(command.argument)) {
			__atomic_store_n(&s_next_turn, command.argument, __ATOMIC_RELEASE);
		}
		break;
	case WIFI_COMMAND_START:
		/* Matches the dashboard: Start begins a fresh run at the default. */
		dashboard_junction_reset();
		record_run_command((uint8_t)DASHBOARD_RUN_START);
		break;
	case WIFI_COMMAND_STOP:
		/* The run is over: the last barcode's turn does not carry over. */
		dashboard_junction_reset();
		record_run_command((uint8_t)DASHBOARD_RUN_STOP);
		break;
	default:
		break;
	}
}

bool dashboard_link_owner_heartbeat(uint8_t out[2])
{
	if (__atomic_load_n(&s_status, __ATOMIC_ACQUIRE) != 1) {
		return false;
	}
	s_heartbeat_sequence++;
	return dashboard_msg_heartbeat(__atomic_load_n(&s_vehicle_state, __ATOMIC_RELAXED),
				       s_heartbeat_sequence, out);
}

/* ---- On-target unit test ------------------------------------------------------ */

#if APP_TEST
#include "../../test_report.h"

static unsigned s_test_calls;
static int s_test_result;
static char s_test_topic[24];

static int test_publish(const char *topic, const uint8_t *payload, uint16_t length)
{
	unsigned i;

	(void)payload;
	(void)length;
	s_test_calls++;
	for (i = 0U; (i < sizeof(s_test_topic) - 1U) && (topic[i] != '\0'); i++) {
		s_test_topic[i] = topic[i];
	}
	s_test_topic[i] = '\0';
	return s_test_result;
}

BOOL dashboard_link_test(void)
{
	static INT storage[LINK_WORDS];
	T_CMBF config = {0};
	link_owner_t owner = {0};
	char text[DASHBOARD_MSG_PAYLOAD_MAX];
	uint8_t packet[WIFI_COMMAND_PACKET_SIZE];
	uint8_t heartbeat[DASHBOARD_MSG_HEARTBEAT_LEN];
	dashboard_run_command_t run;
	ID buffer;
	BOOL format_ok, command_ok, queue_ok;
	TEST_INIT();

	/* Formats the dashboard parses. */
	format_ok = dashboard_msg_hump_height(32U, text) == 20U &&	/* {"humpHeightCm":3.2} */
		    dashboard_msg_barcode('b', text) == 1U && text[0] == 'B' &&
		    dashboard_msg_barcode('1', text) == 0U &&
		    dashboard_msg_obstacle_profile(65535U, 65535U, 65535U,
						   DASHBOARD_SIDE_RIGHT, text) > 0U &&
		    dashboard_msg_heartbeat(1U, 200U, heartbeat) &&
		    heartbeat[0] == 0x24U && heartbeat[1] == 72U &&
		    !dashboard_msg_heartbeat(5U, 0U, heartbeat) &&
		    dashboard_msg_line_event(DASHBOARD_LINE_SEARCH_CENTRED, text) == 40U &&
		    text[0] == '{' && text[39] == '}' &&	/* {"type":"LINE","event":"SEARCH_CENTRED"} */
		    dashboard_msg_line_event((dashboard_line_event_t)9, text) == 0U;

	/* Junction default, update, reset; Start resets; run command once. */
	command_ok = dashboard_next_junction() == DASHBOARD_TURN_LEFT;
	packet[0] = (uint8_t)((WIFI_COMMAND_JUNCTION << 5) | DASHBOARD_TURN_UTURN);
	packet[1] = 1U;
	dashboard_link_owner_on_message("robot/command", packet, 2U);
	command_ok = dashboard_next_junction() == DASHBOARD_TURN_UTURN && command_ok;
	packet[0] = (uint8_t)((WIFI_COMMAND_JUNCTION << 5) | 1U);	/* reverse: rejected */
	dashboard_link_owner_on_message("robot/command", packet, 2U);
	command_ok = dashboard_next_junction() == DASHBOARD_TURN_UTURN && command_ok;
	dashboard_link_owner_on_message("robot/other", packet, 2U);
	packet[0] = (uint8_t)(WIFI_COMMAND_START << 5);
	dashboard_link_owner_on_message("robot/command", packet, 2U);
	command_ok = dashboard_next_junction() == DASHBOARD_TURN_LEFT &&
		     dashboard_take_run_command(&run) && run == DASHBOARD_RUN_START &&
		     !dashboard_take_run_command(&run) && command_ok;

	/* Queue: BUSY keeps the message, OK drains it, full queue drops. */
	config.mbfatr = TA_TFIFO | TA_USERBUF;
	config.bufsz = (SZ)sizeof(storage);
	config.maxmsz = (INT)sizeof(link_message_t);
	config.bufptr = storage;
	buffer = tk_cre_mbf(&config);
	queue_ok = buffer >= E_OK;
	if (queue_ok) {
		unsigned i;
		queue_ok = submit_to(buffer, DASHBOARD_TOPIC_BARCODE, "A", 1U) == E_OK;
		s_test_calls = 0U;
		s_test_result = WIFI_MQTT_PUBLISH_BUSY;
		owner_cycle(&owner, buffer, test_publish);
		queue_ok = owner.have_pending && s_test_calls == 1U && queue_ok;
		s_test_result = WIFI_MQTT_PUBLISH_OK;
		owner_cycle(&owner, buffer, test_publish);
		queue_ok = !owner.have_pending && s_test_calls == 2U &&
			   text_equal(s_test_topic, "robot/barcode") && queue_ok;
		for (i = 0U; i < DASHBOARD_LINK_QUEUE_CAPACITY; i++) {
			queue_ok = submit_to(buffer, DASHBOARD_TOPIC_EVENT, "{}", 2U) == E_OK && queue_ok;
		}
		queue_ok = submit_to(buffer, DASHBOARD_TOPIC_EVENT, "{}", 2U) == E_TMOUT &&
			   submit_to(buffer, DASHBOARD_TOPIC_EVENT, "{}", 0U) == E_PAR && queue_ok;
		for (i = 0U; i < DASHBOARD_LINK_QUEUE_CAPACITY; i++) {
			owner_cycle(&owner, buffer, test_publish);
		}
		queue_ok = tk_rcv_mbf(buffer, &owner.pending, TMO_POL) == E_TMOUT && queue_ok;
		queue_ok = tk_del_mbf(buffer) == E_OK && queue_ok;
	}

	TEST_CHECK("WIFI-U08", format_ok, "dashboard message formats");
	TEST_CHECK("WIFI-U09", command_ok, "dashboard junction and run commands");
	TEST_CHECK("WIFI-U10", queue_ok, "dashboard queue retry, drain and overflow");
	TEST_SUMMARY("dashboard link");
	return format_ok && command_ok && queue_ok;
}
#endif
