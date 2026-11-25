/*
 *	dashboard_msg.c
 *	Bounded formatters for the dashboard wire formats (see dashboard_msg.h).
 */

#include "dashboard_msg.h"

#include <stddef.h>

#define HEARTBEAT_MESSAGE_TYPE  1U
#define HEARTBEAT_TYPE_SHIFT    5U
#define HEARTBEAT_STATE_SHIFT   2U
#define HEARTBEAT_STATE_MAX     4U
#define HEARTBEAT_SEQUENCE_MASK 0x7fU
#define DECIMAL_BASE            10U
#define U16_DIGITS_MAX          5U

/* Append-only writer that refuses to run past DASHBOARD_MSG_PAYLOAD_MAX.
 * Once an append fails, every later append also fails and the message is
 * reported as length 0, so a truncated JSON object is never published. */
typedef struct {
	char *out;
	uint16_t length;
	bool failed;
} msg_writer_t;

static const char *const s_topics[DASHBOARD_TOPIC_COUNT] = {
	"robot/event",
	"robot/telemetry/detail",
	"robot/barcode"
};

static void put_char(msg_writer_t *writer, char c)
{
	if (writer->failed || (writer->length >= DASHBOARD_MSG_PAYLOAD_MAX)) {
		writer->failed = true;
		return;
	}
	writer->out[writer->length] = c;
	writer->length++;
}

static void put_text(msg_writer_t *writer, const char *text)
{
	while (*text != '\0') {
		put_char(writer, *text);
		text++;
	}
}

static void put_u16(msg_writer_t *writer, uint16_t value)
{
	char digits[U16_DIGITS_MAX];
	uint8_t count = 0U;

	do {
		digits[count] = (char)('0' + (value % DECIMAL_BASE));
		value = (uint16_t)(value / DECIMAL_BASE);
		count++;
	} while ((value != 0U) && (count < U16_DIGITS_MAX));

	while (count > 0U) {
		count--;
		put_char(writer, digits[count]);
	}
}

static uint16_t finish(const msg_writer_t *writer)
{
	return writer->failed ? 0U : writer->length;
}

static msg_writer_t writer_begin(char *out)
{
	msg_writer_t writer;

	writer.out = out;
	writer.length = 0U;
	writer.failed = (out == NULL);
	return writer;
}

static const char *turn_name(dashboard_turn_t turn)
{
	const char *name = NULL;

	switch (turn) {
	case DASHBOARD_TURN_STRAIGHT:
		name = "forward";
		break;
	case DASHBOARD_TURN_LEFT:
		name = "left";
		break;
	case DASHBOARD_TURN_RIGHT:
		name = "right";
		break;
	case DASHBOARD_TURN_UTURN:
		name = "uturn";
		break;
	default:
		break;
	}
	return name;
}

const char *dashboard_msg_topic(uint8_t topic)
{
	return (topic < (uint8_t)DASHBOARD_TOPIC_COUNT) ? s_topics[topic] : NULL;
}

bool dashboard_msg_turn_valid(uint8_t argument)
{
	return turn_name((dashboard_turn_t)argument) != NULL;
}

uint16_t dashboard_msg_simple_event(dashboard_simple_event_t event, char *out)
{
	msg_writer_t writer = writer_begin(out);
	const char *type = NULL;

	switch (event) {
	case DASHBOARD_EVENT_BARCODE_DETECTED:
		type = "BARCODE_DETECTED";
		break;
	case DASHBOARD_EVENT_LINE_REJOINED:
		type = "LINE_REJOINED";
		break;
	case DASHBOARD_EVENT_PICKUP_STOP:
		type = "PICKUP_STOP";
		break;
	default:
		break;
	}
	if (type == NULL) {
		return 0U;
	}
	put_text(&writer, "{\"type\":\"");
	put_text(&writer, type);
	put_text(&writer, "\"}");
	return finish(&writer);
}

uint16_t dashboard_msg_action_done(dashboard_turn_t turn, char *out)
{
	msg_writer_t writer = writer_begin(out);
	const char *name = turn_name(turn);

	if (name == NULL) {
		return 0U;
	}
	put_text(&writer, "{\"type\":\"ACTION_DONE\",\"action\":\"");
	put_text(&writer, name);
	put_text(&writer, "\"}");
	return finish(&writer);
}

uint16_t dashboard_msg_hump_peak(uint16_t peak_mm, char *out)
{
	msg_writer_t writer = writer_begin(out);

	put_text(&writer, "{\"type\":\"HUMP\",\"peakMm\":");
	put_u16(&writer, peak_mm);
	put_char(&writer, '}');
	return finish(&writer);
}

/* The dashboard reads humpHeightCm as a number; mm is sent as cm with one
 * decimal place, e.g. 32 mm -> 3.2. */
uint16_t dashboard_msg_hump_height(uint16_t height_mm, char *out)
{
	msg_writer_t writer = writer_begin(out);

	put_text(&writer, "{\"humpHeightCm\":");
	put_u16(&writer, (uint16_t)(height_mm / DECIMAL_BASE));
	put_char(&writer, '.');
	put_char(&writer, (char)('0' + (height_mm % DECIMAL_BASE)));
	put_char(&writer, '}');
	return finish(&writer);
}

uint16_t dashboard_msg_obstacle(uint16_t distance_mm, char *out)
{
	msg_writer_t writer = writer_begin(out);

	put_text(&writer, "{\"type\":\"OBSTACLE\",\"distMm\":");
	put_u16(&writer, distance_mm);
	put_char(&writer, '}');
	return finish(&writer);
}

uint16_t dashboard_msg_obstacle_profile(uint16_t width_mm, uint16_t clear_left_mm,
					uint16_t clear_right_mm, dashboard_side_t side,
					char *out)
{
	msg_writer_t writer = writer_begin(out);

	if ((side != DASHBOARD_SIDE_LEFT) && (side != DASHBOARD_SIDE_RIGHT)) {
		return 0U;
	}
	put_text(&writer, "{\"type\":\"OBSTACLE_PROFILE\",\"widthMm\":");
	put_u16(&writer, width_mm);
	put_text(&writer, ",\"clearLeftMm\":");
	put_u16(&writer, clear_left_mm);
	put_text(&writer, ",\"clearRightMm\":");
	put_u16(&writer, clear_right_mm);
	put_text(&writer, (side == DASHBOARD_SIDE_LEFT) ? ",\"side\":\"LEFT\"}"
							 : ",\"side\":\"RIGHT\"}");
	return finish(&writer);
}

uint16_t dashboard_msg_avoid_phase(dashboard_avoid_phase_t phase, char *out)
{
	msg_writer_t writer = writer_begin(out);
	const char *name = NULL;

	switch (phase) {
	case DASHBOARD_AVOID_SCANNING:
		name = "SCANNING";
		break;
	case DASHBOARD_AVOID_BYPASS:
		name = "BYPASS";
		break;
	case DASHBOARD_AVOID_SEARCH:
		name = "SEARCH";
		break;
	case DASHBOARD_AVOID_REJOINED:
		name = "REJOINED";
		break;
	default:
		break;
	}
	if (name == NULL) {
		return 0U;
	}
	put_text(&writer, "{\"type\":\"AVOID\",\"phase\":\"");
	put_text(&writer, name);
	put_text(&writer, "\"}");
	return finish(&writer);
}

uint16_t dashboard_msg_line_event(dashboard_line_event_t event, char *out)
{
	msg_writer_t writer = writer_begin(out);
	const char *name = NULL;

	switch (event) {
	case DASHBOARD_LINE_PIVOT:
		name = "PIVOT";
		break;
	case DASHBOARD_LINE_SEARCH_FOUND:
		name = "SEARCH_FOUND";
		break;
	case DASHBOARD_LINE_SEARCH_CENTRED:
		name = "SEARCH_CENTRED";
		break;
	case DASHBOARD_LINE_LOST:
		name = "LOST";
		break;
	default:
		break;
	}
	if (name == NULL) {
		return 0U;
	}
	put_text(&writer, "{\"type\":\"LINE\",\"event\":\"");
	put_text(&writer, name);
	put_text(&writer, "\"}");
	return finish(&writer);
}

uint16_t dashboard_msg_barcode(char letter, char *out)
{
	msg_writer_t writer = writer_begin(out);

	if ((letter >= 'a') && (letter <= 'z')) {
		letter = (char)(letter - 'a' + 'A');
	}
	if ((letter < 'A') || (letter > 'Z')) {
		return 0U;
	}
	put_char(&writer, letter);
	return finish(&writer);
}

bool dashboard_msg_heartbeat(uint8_t vehicle_state, uint8_t sequence,
			     uint8_t out[DASHBOARD_MSG_HEARTBEAT_LEN])
{
	if ((out == NULL) || (vehicle_state > HEARTBEAT_STATE_MAX)) {
		return false;
	}
	out[0] = (uint8_t)((HEARTBEAT_MESSAGE_TYPE << HEARTBEAT_TYPE_SHIFT) |
			   ((uint32_t)vehicle_state << HEARTBEAT_STATE_SHIFT));
	out[1] = (uint8_t)(sequence & HEARTBEAT_SEQUENCE_MASK);
	return true;
}
