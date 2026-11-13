#include "../app.h"
#include "barcode_scanner_test.h"

#if APP_TEST
#include "../test_report.h"
#include "barcode_scanner.h"

static barcode_scanner_t scanner;
static const barcode_scanner_config_t config = {10U, 0U, 0U, 0U, 0U};

static bool feed_character(const char *pattern, int32_t *count, int direction)
{
	unsigned i;
	bool black = true;

	*count += direction * 10;
	if (!barcode_scanner_update(&scanner, black, *count)) return false;
	for (i = 0U; i < BARCODE_ELEMENTS; i++) {
		unsigned index = direction > 0 ? i : BARCODE_ELEMENTS - 1U - i;
		*count += direction * (pattern[index] == 'w' ? 20 : 10);
		black = !black;
		if (!barcode_scanner_update(&scanner, black, *count)) return false;
	}
	return true;
}

static bool frame_test(int direction)
{
	const char *star = "nwnnwnwnn";
	const char *a = "wnnnnwnnw";
	const char *b = "nnwnnwnnw";
	int32_t count = 0;
	uint8_t result[2];
	uint8_t length = 0;

	if (!barcode_scanner_init(&scanner, &config, false, count) ||
	    !feed_character(star, &count, direction) ||
	    !feed_character(direction > 0 ? a : b, &count, direction) ||
	    !feed_character(direction > 0 ? b : a, &count, direction) ||
	    !feed_character(star, &count, direction)) return false;
	if (barcode_scanner_take_result(&scanner, result, 1U, &length)) return false;
	if (!barcode_scanner_take_result(&scanner, result, sizeof(result), &length)) return false;
	return length == 2U && result[0] == 'A' && result[1] == 'B' &&
	       !barcode_scanner_take_result(&scanner, result, sizeof(result), &length);
}

static bool rejection_test(void)
{
	barcode_char_info_t event;
	barcode_scanner_config_t invalid = {0};
	int32_t count = 0;
	const char *star = "nwnnwnwnn";

	if (barcode_scanner_init(&scanner, &invalid, false, count)) return false;
	if (!barcode_scanner_init(&scanner, &config, false, count) ||
	    !feed_character(star, &count, 1) ||
	    !barcode_scanner_update(&scanner, true, count += 10) ||
	    !barcode_scanner_update(&scanner, false, count += 1) ||
	    !barcode_scanner_take_event(&scanner, &event) ||
	    event.reject != BARCODE_REJECT_GLITCH || !event.aborted_frame) return false;

	count = 0;
	if (!barcode_scanner_init(&scanner, &config, false, count) ||
	    !feed_character(star, &count, 1) ||
	    !barcode_scanner_update(&scanner, true, count += 41) ||
	    !barcode_scanner_take_event(&scanner, &event) ||
	    event.reject != BARCODE_REJECT_GAP || !event.aborted_frame) return false;

	count = 0;
	if (!barcode_scanner_init(&scanner, &config, false, count) ||
	    !feed_character(star, &count, 1) ||
	    !barcode_scanner_update(&scanner, true, count -= 10) ||
	    !barcode_scanner_take_event(&scanner, &event) ||
	    event.reject != BARCODE_REJECT_DIRECTION || !event.aborted_frame) return false;

	count = 0;
	if (!barcode_scanner_init(&scanner, &config, false, count) ||
	    !feed_character(star, &count, 1) ||
	    !barcode_scanner_update(&scanner, true, count += 10) ||
	    !barcode_scanner_update(&scanner, false, count += 10) ||
	    !barcode_scanner_update(&scanner, true, count -= 10) ||
	    !barcode_scanner_take_event(&scanner, &event) ||
	    event.reject != BARCODE_REJECT_DIRECTION || !event.aborted_frame) return false;

	count = 0;
	return barcode_scanner_init(&scanner, &config, false, count) &&
	       feed_character("nnnnnnnnn", &count, 1) &&
	       barcode_scanner_take_event(&scanner, &event) &&
	       event.reject == BARCODE_REJECT_SEPARATION;
}

BOOL barcode_scanner_test(void)
{
	BOOL forward_ok;
	BOOL reverse_ok;
	BOOL rejection_ok;
	TEST_INIT();

	forward_ok = frame_test(1);
	reverse_ok = frame_test(-1);
	rejection_ok = rejection_test();
	TEST_CHECK("BAR-U01", forward_ok, "forward frame and result capacity");
	TEST_CHECK("BAR-U02", reverse_ok, "reverse frame and payload order");
	TEST_CHECK("BAR-U03", rejection_ok, "invalid config and bad scan rejection");
	TEST_SUMMARY("barcode");
	return forward_ok && reverse_ok && rejection_ok;
}
#endif
