#include "../../app.h"
#include "straddle_steering.h"

#if APP_TEST
#include "../../test_report.h"
#include "../../subsystem3/line_barcode/line_observation.h"

/* Speed 10, turn 2, overshoot 250, pivot 5/4, blind 1000, pivot max 4000,
 * counter 50 % up to 300 ms. */
static const straddle_config_t s_cfg = { 10, 2, 250U, 5, 4, 1000U, 4000U, 50U, 300U };

static straddle_t s_st;
static uint32_t s_t;

/* n updates of 50 ms with the same position; the last command. */
static line_steering_command_t steps(uint8_t position, unsigned n)
{
	line_steering_command_t c = { 0, 0, true };

	for (unsigned i = 0U; i < n; i++) {
		s_t += 50U;
		c = straddle_update(&s_st, position, s_t);
	}
	return c;
}

static BOOL is_cmd(line_steering_command_t c, int16_t linear, int16_t turn)
{
	return !c.stop_latched && c.linear_cm_s == linear && c.turn_cm_s == turn;
}

/* Tape in the gap: straight; one sensor: gentle turn towards it; tape back
 * in the gap: counter-steer half the correction, then straight. */
static BOOL straight_correct_counter_test(void)
{
	line_steering_command_t c;
	BOOL ok = straddle_init(&s_st, &s_cfg);

	s_t = 1000U;
	c = steps(POS_LOST, 40U);       /* 2 s: no sighting yet, so no search */
	ok = is_cmd(c, 10, 0) && s_st.state == STRADDLE_STRAIGHT && ok;
	c = steps(POS_LEFT, 3U);
	ok = is_cmd(c, 10, 2) && s_st.state == STRADDLE_CORRECT && ok;
	c = steps(POS_LOST, 1U);        /* correction lasted 150 ms */
	ok = is_cmd(c, 10, -2) && s_st.state == STRADDLE_COUNTER && s_st.counter_ms == 75U && ok;
	c = steps(POS_LOST, 1U);
	ok = s_st.state == STRADDLE_COUNTER && ok;
	c = steps(POS_LOST, 1U);
	ok = is_cmd(c, 10, 0) && s_st.state == STRADDLE_STRAIGHT && ok;
	/* A sensor during the counter-steer starts a new correction. */
	c = steps(POS_RIGHT, 1U);
	c = steps(POS_LOST, 1U);
	ok = s_st.state == STRADDLE_COUNTER && c.turn_cm_s == 2 && ok;
	c = steps(POS_LEFT, 1U);
	ok = is_cmd(c, 10, 2) && s_st.state == STRADDLE_CORRECT && ok;
	/* Both black (crossing tape): straight. */
	c = steps(POS_JUNCTION, 1U);
	ok = is_cmd(c, 10, 0) && s_st.state == STRADDLE_STRAIGHT && ok;
	return ok;
}

/* Held past overshoot_ms: pivot; no counter-steer after it; both white for
 * blind_ms: search towards the last side; found on the other sensor counts
 * as a false search; a pivot past pivot_max_ms latches until reset. */
static BOOL pivot_search_test(void)
{
	line_steering_command_t c;
	BOOL ok = straddle_init(&s_st, &s_cfg);

	s_t = 5000U;
	c = steps(POS_RIGHT, 6U);
	ok = is_cmd(c, 5, -4) && s_st.state == STRADDLE_RECOVER && s_st.recoveries == 1U && ok;
	c = steps(POS_JUNCTION, 1U);
	ok = s_st.state == STRADDLE_RECOVER && ok;
	c = steps(POS_RIGHT, 4U);
	ok = is_cmd(c, 5, -4) && ok;
	c = steps(POS_LOST, 1U);
	ok = is_cmd(c, 10, 0) && s_st.state == STRADDLE_STRAIGHT && ok;
	c = steps(POS_LOST, 20U);       /* 1000 ms both white */
	ok = is_cmd(c, 5, -4) && s_st.state == STRADDLE_SEARCH && s_st.searches == 1U && ok;
	c = steps(POS_LEFT, 1U);
	ok = is_cmd(c, 10, 2) && s_st.false_searches == 1U && ok;
	c = steps(POS_LOST, 1U);
	c = steps(POS_LOST, 21U);
	ok = s_st.state == STRADDLE_SEARCH && c.turn_cm_s == 4 && ok;
	c = steps(POS_LOST, 80U);
	ok = c.stop_latched && s_st.stop_latched && ok;
	c = steps(POS_LEFT, 1U);
	ok = c.stop_latched && ok;
	straddle_reset(&s_st);
	c = steps(POS_LOST, 1U);
	ok = is_cmd(c, 10, 0) && s_st.last_side == 0 && s_st.searches == 0U && ok;
	return ok;
}

/* On a hump the blind search never starts (the tape is often out of sight
 * over the crest); off the hump it works again. Reset keeps the hump flag. */
static BOOL hump_test(void)
{
	line_steering_command_t c;
	BOOL ok = straddle_init(&s_st, &s_cfg);

	s_t = 9000U;
	(void)steps(POS_LEFT, 1U);
	straddle_set_hump(&s_st, true);
	c = steps(POS_LOST, 60U);
	ok = is_cmd(c, 10, 0) && s_st.state == STRADDLE_STRAIGHT && s_st.searches == 0U && ok;
	straddle_reset(&s_st);
	ok = s_st.hump && ok;
	(void)steps(POS_LEFT, 1U);
	straddle_set_hump(&s_st, false);
	c = steps(POS_LOST, 22U);
	ok = s_st.state == STRADDLE_SEARCH && ok;
	return ok;
}

/* Bad configs are refused and leave the steering stopped; unknown positions
 * latch the stop. */
static BOOL config_test(void)
{
	straddle_config_t bad = s_cfg;
	BOOL ok;

	bad.turn_cm_s = bad.speed_cm_s;
	ok = !straddle_init(&s_st, &bad) && straddle_update(&s_st, POS_LOST, 1U).stop_latched;
	bad = s_cfg;
	bad.overshoot_ms = 10U;
	ok = !straddle_init(&s_st, &bad) && ok;
	ok = straddle_init(&s_st, &s_cfg) && ok;
	ok = straddle_update(&s_st, 0xFFU, 2U).stop_latched &&
	     straddle_update(&s_st, POS_LOST, 3U).stop_latched && ok;
	ok = straddle_state_name(STRADDLE_COUNTER)[0] == 'C' && straddle_state_name(9U)[0] == '?' && ok;
	return ok;
}

/* Slower over a barcode: straight and correcting speed change at once, the
 * turn and the pivot speed do not; a speed not above the turn is refused. */
static BOOL set_speed_test(void)
{
	line_steering_command_t c;
	BOOL ok = straddle_init(&s_st, &s_cfg);

	s_t = 1000U;
	ok = straddle_set_speed(&s_st, 6) && ok;
	c = steps(POS_LOST, 1U);
	ok = is_cmd(c, 6, 0) && ok;
	c = steps(POS_LEFT, 1U);
	ok = is_cmd(c, 6, 2) && ok;
	c = steps(POS_LEFT, 6U);                /* 350 ms: pivot, own speed */
	ok = is_cmd(c, 5, 4) && s_st.state == STRADDLE_RECOVER && ok;
	ok = !straddle_set_speed(&s_st, 2) && !straddle_set_speed(NULL, 5) &&
	     s_st.config.speed_cm_s == 6 && ok;
	ok = straddle_set_speed(&s_st, 10) && ok;
	c = steps(POS_LOST, 1U);
	ok = !c.stop_latched && c.linear_cm_s == 10 && ok;
	return ok;
}

BOOL straddle_steering_test(void)
{
	BOOL basic_ok, pivot_ok, hump_ok, config_ok, speed_ok;
	TEST_INIT();

	basic_ok = straight_correct_counter_test();
	pivot_ok = pivot_search_test();
	hump_ok = hump_test();
	config_ok = config_test();
	speed_ok = set_speed_test();
	TEST_CHECK("VC-U31", basic_ok,
		   "straddle: gap = straight, one sensor = gentle turn, then counter-steer half the correction");
	TEST_CHECK("VC-U32", pivot_ok,
		   "straddle: held too long pivots, blind search towards the last side, pivot timeout latches until reset");
	TEST_CHECK("VC-U33", hump_ok, "straddle: no blind search on a hump");
	TEST_CHECK("VC-U34", config_ok, "straddle: bad config and unknown positions stop");
	TEST_CHECK("VC-U36", speed_ok, "straddle: speed changes on the fly (slow over a barcode), turn below it");
	TEST_SUMMARY("straddle steering");
	return basic_ok && pivot_ok && hump_ok && config_ok && speed_ok;
}
#endif
