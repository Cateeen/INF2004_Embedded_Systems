#include "../../app.h"
#include "straddle_junction.h"

#if APP_TEST
#include "../../test_report.h"

static bool in(float d, float from, float to) { return d >= from && d < to; }

typedef bool (*both_fn_t)(float d);

/* Drive from..to mm in 1 mm steps; count detections and narrow events. */
static unsigned run(straddle_junction_t *j, float from, float to, both_fn_t both,
		    unsigned *narrow)
{
	unsigned detected = 0U;

	*narrow = 0U;
	for (float d = from; d <= to; d += 1.0f) {
		straddle_junction_event_t e = straddle_junction_update(j, both(d), both(d), d);

		if (e == STRADDLE_JUNCTION_DETECTED) detected++;
		if (e == STRADDLE_JUNCTION_NARROW) (*narrow)++;
	}
	return detected;
}

static bool cross(float d) { return in(d, 100, 112); }
static bool cross_flicker(float d) { return in(d, 100, 112) && !in(d, 105, 107); }
static bool three(float d) { return in(d, 100, 112) || in(d, 125, 137) || in(d, 200, 212); }
static bool blip(float d) { return in(d, 50, 52) || in(d, 100, 112); }

BOOL straddle_junction_test(void)
{
	static const straddle_junction_config_t config = STRADDLE_JUNCTION_DEFAULT_CONFIG;
	straddle_junction_config_t bad = config;
	straddle_junction_t j;
	unsigned narrow, detected;
	BOOL cross_ok, rearm_ok, edge_ok, config_ok;
	TEST_INIT();

	/* A crossing: one junction; a 2 mm flicker inside it is bridged. */
	cross_ok = straddle_junction_init(&j, &config);
	detected = run(&j, 0, 200, cross, &narrow);
	cross_ok = detected == 1U && narrow == 0U && cross_ok;
	straddle_junction_reset(&j);
	detected = run(&j, 0, 200, cross_flicker, &narrow);
	cross_ok = detected == 1U && narrow == 0U && cross_ok;

	/* A second crossing 13 mm after the first is not re-armed yet; one
	 * 88 mm later is. */
	straddle_junction_reset(&j);
	detected = run(&j, 0, 300, three, &narrow);
	rearm_ok = detected == 2U && narrow == 0U;

	/* A 2 mm blip is narrow and leaves it armed; one sensor at a time
	 * (ordinary corrections) never; reverse travel adds nothing. */
	straddle_junction_reset(&j);
	detected = run(&j, 0, 200, blip, &narrow);
	edge_ok = detected == 1U && narrow == 1U;
	straddle_junction_reset(&j);
	detected = 0U;
	for (float d = 0.0f; d < 200.0f; d += 1.0f) {
		bool centre = ((int)d / 20) % 2 == 0;

		if (straddle_junction_update(&j, centre, !centre, d) != STRADDLE_JUNCTION_NONE)
			detected++;
	}
	edge_ok = detected == 0U && edge_ok;
	straddle_junction_reset(&j);
	for (float d = 100.0f; d > 0.0f; d -= 1.0f)
		if (straddle_junction_update(&j, true, true, d) == STRADDLE_JUNCTION_DETECTED)
			detected++;
	edge_ok = detected == 0U && edge_ok;

	bad.both_min_mm = 0.0f;
	config_ok = !straddle_junction_init(&j, &bad);
	bad = config;
	bad.rearm_mm = __builtin_nanf("");
	config_ok = !straddle_junction_init(&j, &bad) &&
		    straddle_junction_update(&j, true, true, 10.0f) == STRADDLE_JUNCTION_NONE &&
		    config_ok;

	TEST_CHECK("JN-U06", cross_ok, "straddle junction: both line sensors black 5 mm is one junction; flicker bridged");
	TEST_CHECK("JN-U07", rearm_ok, "straddle junction: re-armed only after 30 mm without both black");
	TEST_CHECK("JN-U08", edge_ok, "straddle junction: blips are narrow, single-sensor corrections and reverse travel never count");
	TEST_CHECK("JN-U09", config_ok, "straddle junction: invalid configuration rejected");
	TEST_SUMMARY("straddle junction");
	return cross_ok && rearm_ok && edge_ok && config_ok;
}
#endif
