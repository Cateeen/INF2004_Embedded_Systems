#include "../../app.h"
#include "junction_detector.h"

/* NULL comes from the kernel headers via app.h (<stddef.h> clashes with them). */

/* Built-ins: <math.h> clashes with the kernel's size_t once app.h is in. */
#define isfinite(x) __builtin_isfinite(x)
#define fmaxf(a, b) __builtin_fmaxf((a), (b))

static bool valid_config(const junction_config_t *c)
{
	return c != NULL && isfinite(c->left_min_mm) && isfinite(c->left_max_mm) &&
	       isfinite(c->offset_mm) && isfinite(c->tol_mm) && isfinite(c->both_min_mm) &&
	       c->left_min_mm > 0.0f && c->left_min_mm <= c->left_max_mm &&
	       c->offset_mm >= 0.0f && c->tol_mm >= 0.0f && c->both_min_mm > 0.0f;
}

bool junction_detector_init(junction_detector_t *detector, const junction_config_t *config)
{
	if (detector == NULL) return false;
	*detector = (junction_detector_t){0};
	if (!valid_config(config)) return false;
	detector->config = *config;
	detector->initialized = true;
	return true;
}

void junction_detector_reset(junction_detector_t *detector)
{
	junction_config_t config;

	if (detector == NULL || !detector->initialized) return;
	config = detector->config;
	(void)junction_detector_init(detector, &config);
}

void junction_detector_last(const junction_detector_t *detector, float *left_mm,
			    float *both_mm)
{
	if (detector == NULL) return;
	if (left_mm != NULL) *left_mm = detector->left_mm;
	if (both_mm != NULL) *both_mm = detector->both_mm;
}

junction_event_t junction_detector_update(junction_detector_t *detector, bool left_black,
					  bool centre_black, bool right_black,
					  float distance_mm)
{
	const junction_config_t *c;
	float step = 0.0f, lo, hi;

	if (detector == NULL || !detector->initialized || !isfinite(distance_mm))
		return JUNCTION_NONE;
	c = &detector->config;
	if (detector->have_last && distance_mm > detector->last_mm)
		step = distance_mm - detector->last_mm;
	detector->last_mm = distance_mm;
	detector->have_last = true;

	/* After a too-wide stretch, wait for the left sensor to see white. */
	if (detector->wait_white) {
		if (left_black) return JUNCTION_NONE;
		detector->wait_white = false;
	}

	/* A new left-black stretch starts a new candidate (an undecided older one,
	 * e.g. the previous barcode bar, is dropped). */
	if (left_black && !(detector->active && detector->left_on)) {
		detector->active = true;
		detector->left_on = true;
		detector->start_mm = distance_mm;
		detector->left_mm = 0.0f;
		detector->both_mm = 0.0f;
	}
	if (!detector->active) return JUNCTION_NONE;

	if (detector->left_on) {
		if (left_black)
			detector->left_mm = fmaxf(detector->left_mm,
						  distance_mm - detector->start_mm);
		else
			detector->left_on = false;
	}

	/* Where the crossing should be under the line sensors. */
	lo = detector->start_mm + c->offset_mm - c->tol_mm;
	hi = detector->start_mm + c->offset_mm + detector->left_mm + c->tol_mm;
	if (centre_black && right_black && distance_mm >= lo && distance_mm <= hi)
		detector->both_mm += step;

	if (detector->left_mm > c->left_max_mm) {
		detector->active = false;
		detector->wait_white = detector->left_on;
		return JUNCTION_REJECT_WIDE;
	}
	if (detector->left_on) return JUNCTION_NONE;
	if (detector->left_mm < c->left_min_mm) {
		detector->active = false;
		return JUNCTION_REJECT_NARROW;
	}
	if (detector->both_mm >= c->both_min_mm) {
		detector->active = false;
		return JUNCTION_DETECTED;
	}
	if (distance_mm > hi) {
		detector->active = false;
		return JUNCTION_REJECT_UNCONFIRMED;
	}
	return JUNCTION_NONE;
}

#if APP_TEST
#include "../../test_report.h"

typedef struct {
	float left_from, left_to;       /* left sensor black on [from, to) mm */
	float both_from, both_to;       /* centre and right black on [from, to) */
	float centre_from, centre_to;   /* centre alone black on [from, to) */
} jn_track_t;

static bool in(float d, float from, float to) { return d >= from && d < to; }

/* Drive 0..200 mm in 1 mm steps over a track; count each event kind and
 * return the distance of the first DETECTED (or -1). */
static float jn_run(const jn_track_t *t, unsigned counts[5])
{
	const junction_config_t config = JUNCTION_DEFAULT_CONFIG;
	junction_detector_t det;
	float detected_at = -1.0f;
	unsigned i;

	for (i = 0U; i < 5U; i++) counts[i] = 0U;
	if (!junction_detector_init(&det, &config)) return -2.0f;
	for (float d = 0.0f; d <= 200.0f; d += 1.0f) {
		bool both = in(d, t->both_from, t->both_to);
		junction_event_t e = junction_detector_update(
			&det, in(d, t->left_from, t->left_to),
			both || in(d, t->centre_from, t->centre_to), both, d);

		counts[e]++;
		if (e == JUNCTION_DETECTED && detected_at < 0.0f) detected_at = d;
	}
	return detected_at;
}

/* Barcode under the left sensor, bars as the sensor MEASURES them (~6 mm
 * narrower than printed, like the tape: 2/5/2/5 mm) while the robot rides the
 * edge (centre+right both black now and then). */
static BOOL jn_barcode_test(void)
{
	const junction_config_t config = JUNCTION_DEFAULT_CONFIG;
	static const float bars[][2] = {{100, 103}, {106, 112}, {116, 119}, {123, 129}};
	junction_detector_t det;
	unsigned narrow = 0U, other = 0U;

	if (!junction_detector_init(&det, &config)) return FALSE;
	for (float d = 0.0f; d <= 200.0f; d += 1.0f) {
		bool left = false;
		junction_event_t e;

		for (unsigned b = 0U; b < 4U; b++)
			left = left || in(d, bars[b][0], bars[b][1]);
		e = junction_detector_update(&det, left, true, in(d, 110, 160), d);
		if (e == JUNCTION_REJECT_NARROW) narrow++;
		else if (e != JUNCTION_NONE) other++;
	}
	return narrow == 4U && other == 0U;
}

BOOL junction_detector_test(void)
{
	unsigned n[5];
	float at;
	BOOL cross_ok, wide_tape_ok, centred_ok, reject_ok, barcode_ok, config_ok;
	junction_detector_t det;
	junction_config_t bad = JUNCTION_DEFAULT_CONFIG;
	TEST_INIT();

	/* As measured on the robot over an 18 mm tape: left black ~12 mm
	 * (100-112), line sensors black ~7 mm later for ~12 mm (107-119). */
	at = jn_run(&(jn_track_t){100, 112, 107, 119, 0, 0}, n);
	cross_ok = at >= 110.0f && at <= 115.0f && n[JUNCTION_DETECTED] == 1U &&
		   n[JUNCTION_REJECT_NARROW] + n[JUNCTION_REJECT_WIDE] +
		   n[JUNCTION_REJECT_UNCONFIRMED] == 0U;
	/* Full-width readings with a later line-sensor crossing also pass. */
	at = jn_run(&(jn_track_t){100, 118, 115, 133, 0, 0}, n);
	cross_ok = at >= 118.0f && at <= 125.0f && n[JUNCTION_DETECTED] == 1U && cross_ok;
	/* 30 mm tape: line sensors black while the left still is; decided when
	 * the left stretch ends. */
	at = jn_run(&(jn_track_t){100, 130, 120, 150, 0, 0}, n);
	wide_tape_ok = at >= 130.0f && at <= 132.0f && n[JUNCTION_DETECTED] == 1U;
	/* Already riding CENTRE (both black throughout) when the crossing comes. */
	at = jn_run(&(jn_track_t){100, 118, 0, 201, 0, 0}, n);
	centred_ok = at > 0.0f && n[JUNCTION_DETECTED] == 1U;
	TEST_CHECK("JN-U01", cross_ok && wide_tape_ok && centred_ok,
		   "tape across the path: left black then centre+right black ~8 mm later is a junction");

	/* Left 18 mm but only the centre sensor black after it: unconfirmed. */
	(void)jn_run(&(jn_track_t){100, 118, 0, 0, 110, 160}, n);
	reject_ok = n[JUNCTION_REJECT_UNCONFIRMED] == 1U && n[JUNCTION_DETECTED] == 0U;
	/* Both black only after the window (window ends 100+8+17+12 = 137). */
	(void)jn_run(&(jn_track_t){100, 118, 140, 170, 0, 0}, n);
	reject_ok = n[JUNCTION_REJECT_UNCONFIRMED] == 1U && n[JUNCTION_DETECTED] == 0U &&
		    reject_ok;
	/* Edge riding with the left sensor white: nothing at all. */
	(void)jn_run(&(jn_track_t){0, 0, 30, 90, 0, 201}, n);
	reject_ok = n[JUNCTION_NONE] == 201U && reject_ok;
	/* A 100 mm left patch: rejected once as too wide, not re-armed inside it. */
	(void)jn_run(&(jn_track_t){50, 150, 0, 201, 0, 0}, n);
	reject_ok = n[JUNCTION_REJECT_WIDE] == 1U && n[JUNCTION_DETECTED] == 0U && reject_ok;
	TEST_CHECK("JN-U02", reject_ok,
		   "unconfirmed, mistimed, left-white and too-wide patterns are not junctions");

	barcode_ok = jn_barcode_test();
	TEST_CHECK("JN-U03", barcode_ok, "barcode bars under the left sensor are rejected as narrow");

	/* Going backwards adds no distance; reset forgets the candidate. */
	config_ok = junction_detector_init(&det, &bad);
	(void)junction_detector_update(&det, true, true, true, 100.0f);
	(void)junction_detector_update(&det, true, true, true, 90.0f);
	config_ok = det.left_mm == 0.0f && det.both_mm == 0.0f && config_ok;
	junction_detector_reset(&det);
	config_ok = !det.active && det.initialized && config_ok;
	TEST_CHECK("JN-U04", config_ok, "reverse travel adds nothing; reset clears the candidate");

	bad.left_min_mm = 0.0f;
	config_ok = !junction_detector_init(&det, &bad);
	bad = (junction_config_t)JUNCTION_DEFAULT_CONFIG;
	bad.left_max_mm = 5.0f;
	config_ok = !junction_detector_init(&det, &bad) && config_ok;
	bad = (junction_config_t)JUNCTION_DEFAULT_CONFIG;
	bad.tol_mm = __builtin_nanf("");
	config_ok = !junction_detector_init(&det, &bad) && config_ok &&
		    junction_detector_update(&det, true, true, true, 1.0f) == JUNCTION_NONE;
	TEST_CHECK("JN-U05", config_ok, "invalid configuration rejected");

	TEST_SUMMARY("junction detector");
	return cross_ok && wide_tape_ok && centred_ok && reject_ok && barcode_ok && config_ok;
}
#endif
