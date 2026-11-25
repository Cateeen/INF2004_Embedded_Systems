#include "../../app.h"
#include "barcode_scanner_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "barcode_scanner.h"

static barcode_scanner_t scanner;
static const barcode_scanner_config_t config = {10U, 0U, 0U, 0U, 0U};

static bool feed_elements(const char *pattern, int32_t *count, int direction, bool mirrored)
{
	unsigned i;
	bool black = true;

	*count += direction * 10;
	if (!barcode_scanner_update(&scanner, black, *count)) return false;
	for (i = 0U; i < BARCODE_ELEMENTS; i++) {
		unsigned index = mirrored ? BARCODE_ELEMENTS - 1U - i : i;
		*count += direction * (pattern[index] == 'w' ? 20 : 10);
		black = !black;
		if (!barcode_scanner_update(&scanner, black, *count)) return false;
	}
	return true;
}

static bool feed_character(const char *pattern, int32_t *count, int direction)
{
	return feed_elements(pattern, count, direction, direction < 0);
}

/* A label laid backwards: forward travel sees it mirrored, last character
 * first; reverse travel sees it unmirrored, first character first. */
static bool reversed_label_test(int direction)
{
	const char *star = "nwnnwnwnn";
	const char *a = "wnnnnwnnw";
	const char *b = "nnwnnwnnw";
	bool mirrored = direction > 0;
	int32_t count = 0;
	uint8_t result[2];
	uint8_t length = 0;
	barcode_char_info_t event;

	if (!barcode_scanner_init(&scanner, &config, false, count) ||
	    !feed_elements(star, &count, direction, mirrored) ||
	    !barcode_scanner_take_event(&scanner, &event) ||
	    event.symbol != '*' || !event.label_reversed ||
	    !feed_elements(mirrored ? b : a, &count, direction, mirrored) ||
	    !feed_elements(mirrored ? a : b, &count, direction, mirrored) ||
	    !feed_elements(star, &count, direction, mirrored)) return false;
	return barcode_scanner_take_result(&scanner, result, sizeof(result), &length) &&
	       length == 2U && result[0] == 'A' && result[1] == 'B';
}

static bool feed_widths(const uint32_t *widths, int32_t *count, int direction,
			bool mirrored)
{
	unsigned i;
	bool black = true;

	*count += direction * 35;
	if (!barcode_scanner_update(&scanner, black, *count)) return false;
	for (i = 0U; i < BARCODE_ELEMENTS; i++) {
		unsigned index = mirrored ? BARCODE_ELEMENTS - 1U - i : i;
		*count += direction * (int32_t)widths[index];
		black = !black;
		if (!barcode_scanner_update(&scanner, black, *count)) return false;
	}
	return true;
}

static bool calibrated_y_test(int direction, bool mirrored)
{
	static const uint32_t start[] = {55, 104, 66, 33, 109, 39, 110, 41, 26};
	static const uint32_t y[] = {121, 124, 35, 35, 128, 36, 21, 54, 27};
	static const uint32_t stop[] = {39, 128, 56, 27, 117, 29, 128, 22, 45};
	const barcode_scanner_config_t calibrated = {35U, 25U, 0U, 0U, 0U};
	int32_t count = 0;
	uint8_t result[1];
	uint8_t length = 0;

	return barcode_scanner_init(&scanner, &calibrated, false, count) &&
	       feed_widths(mirrored ? stop : start, &count, direction, mirrored) &&
	       feed_widths(y, &count, direction, mirrored) &&
	       feed_widths(mirrored ? start : stop, &count, direction, mirrored) &&
	       barcode_scanner_take_result(&scanner, result, sizeof(result), &length) &&
	       length == 1U && result[0] == 'Y';
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

/* One character at any scale: a white gap, then the nine elements. */
static bool feed_scaled(const char *pattern, int32_t *count, int32_t gap,
			int32_t narrow, int32_t wide)
{
	unsigned i;
	bool black = true;

	*count += gap;
	if (!barcode_scanner_update(&scanner, black, *count)) return false;
	for (i = 0U; i < BARCODE_ELEMENTS; i++) {
		*count += pattern[i] == 'w' ? wide : narrow;
		black = !black;
		if (!barcode_scanner_update(&scanner, black, *count)) return false;
	}
	return true;
}

/*
 * Config says a 10-count module, but labels are printed bigger: the module
 * is learned from the frame, so a 5x label (gap 50 > the old fixed 40-count
 * limit) decodes, a smaller 0.8x one decodes, and a gap that is too big for
 * the measured size is still rejected.
 */
static bool scaled_label_test(void)
{
	const char *star = "nwnnwnwnn";
	const char *a = "wnnnnwnnw";
	static const int32_t scale_x10[] = {8, 10, 25, 50, 120};
	barcode_char_info_t event;
	uint8_t result[1];
	uint8_t length = 0;
	int32_t count;
	unsigned s;

	for (s = 0U; s < sizeof(scale_x10) / sizeof(scale_x10[0]); s++) {
		int32_t narrow = scale_x10[s];
		int32_t wide = narrow * 5 / 2;
		count = 0;
		if (!barcode_scanner_init(&scanner, &config, false, count) ||
		    !feed_scaled(star, &count, 500, narrow, wide) ||
		    !feed_scaled(a, &count, narrow, narrow, wide) ||
		    !feed_scaled(star, &count, narrow, narrow, wide) ||
		    !barcode_scanner_take_result(&scanner, result, sizeof(result), &length) ||
		    length != 1U || result[0] != 'A')
			return false;
	}

	count = 0;
	return barcode_scanner_init(&scanner, &config, false, count) &&
	       feed_scaled(star, &count, 500, 50, 125) &&
	       barcode_scanner_module_counts(&scanner) == 50U &&
	       barcode_scanner_update(&scanner, true, count += 201) &&
	       barcode_scanner_take_event(&scanner, &event) &&
	       event.reject == BARCODE_REJECT_GAP && event.aborted_frame;
}

BOOL barcode_scanner_test(void)
{
	BOOL forward_ok;
	BOOL reverse_ok;
	BOOL rejection_ok;
	BOOL reversed_label_ok;
	BOOL calibrated_ok;
	BOOL scaled_ok;
	TEST_INIT();

	forward_ok = frame_test(1);
	reverse_ok = frame_test(-1);
	rejection_ok = rejection_test();
	reversed_label_ok = reversed_label_test(1) && reversed_label_test(-1);
	calibrated_ok = calibrated_y_test(1, false) && calibrated_y_test(1, true) &&
			calibrated_y_test(-1, false) && calibrated_y_test(-1, true);
	scaled_ok = scaled_label_test();
	TEST_CHECK("BAR-U01", forward_ok, "forward frame and result capacity");
	TEST_CHECK("BAR-U02", reverse_ok, "reverse frame and payload order");
	TEST_CHECK("BAR-U03", rejection_ok, "invalid config and bad scan rejection");
	TEST_CHECK("BAR-U04", reversed_label_ok, "backwards label in both travel directions");
	TEST_CHECK("BAR-U05", calibrated_ok, "calibrated Y scan widths in both orientations");
	TEST_CHECK("BAR-U06", scaled_ok, "barcode size learned per frame (0.8x to 12x)");
	TEST_SUMMARY("barcode");
	return forward_ok && reverse_ok && rejection_ok && reversed_label_ok && calibrated_ok &&
	       scaled_ok;
}
#endif
