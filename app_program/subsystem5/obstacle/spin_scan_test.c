#include "../../app.h"
#include "spin_scan_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "spin_scan.h"
#include "obs_profile_config.h"


#define DEG_TO_RAD (3.14159265f / 180.0f)
#define N_FULL     41U        /* R60..L60 in 3 deg steps */

static const spin_scan_config_t s_cfg = {
	70.0f,   /* sensor offset */
	60.0f,   /* cluster */
	15.0f,   /* trim = the model's beam half-angle */
	20.0f,   /* centre zone */
	60.0f,   /* half */
	200.0f,  /* lane */
	400.0f,  /* beyond */
	450.0f   /* max obstacle distance */
};

static float absf(float v) { return v < 0.0f ? -v : v; }

/* Model HC-SR04 reading at heading h (deg, + = left): a flat face face_mm ahead
 * of the turning point spanning x_min..x_max (mm, + = left of the line),
 * heard when any point of it lies within beam_deg of the beam axis; the
 * nearest such point is the echo. */
static uint16_t model(float h, float face_mm, float x_min, float x_max, float beam_deg)
{
	float sx = 70.0f * __builtin_sinf(h * DEG_TO_RAD), sy = 70.0f * __builtin_cosf(h * DEG_TO_RAD);
	float ux = __builtin_sinf(h * DEG_TO_RAD), uy = __builtin_cosf(h * DEG_TO_RAD);
	float c_beam = __builtin_cosf(beam_deg * DEG_TO_RAD), best = 1e9f, x;

	for (x = x_min; x <= x_max + 0.01f; x += 2.0f) {
		float dx = x - sx, dy = face_mm - sy;
		float d = __builtin_sqrtf(dx * dx + dy * dy);

		if ((dx * ux + dy * uy) / d >= c_beam && d < best) best = d;
	}
	return best < 1e8f ? (uint16_t)(best + 0.5f) : (uint16_t)OBS_DIST_NONE;
}

static void model_scan(spin_scan_sample_t *s, float x_min, float x_max)
{
	unsigned i;

	for (i = 0U; i < N_FULL; i++) {
		s[i].deg = -60.0f + 3.0f * (float)i;
		s[i].mm = model(s[i].deg, 370.0f, x_min, x_max, 15.0f);
	}
}

/* OBS-U10: width and extents of a 90 mm face 300 mm from the sensor. */
static BOOL width_test(void)
{
	spin_scan_sample_t s[N_FULL];
	spin_scan_result_t r;
	BOOL ok = true;

	model_scan(s, -45.0f, 45.0f);                 /* centred */
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.found && r.left_seen && r.right_seen && ok;
	ok = absf(r.width_mm - 90.0f) <= 20.0f && ok;
	ok = absf(r.left_mm - r.right_mm) <= 20.0f && ok;
	ok = absf(r.closest_mm - 300.0f) <= 2.0f && ok;
	ok = r.left_free.passable && r.right_free.passable &&
	     !r.left_free.object_seen && !r.right_free.object_seen && ok;

	model_scan(s, -95.0f, -5.0f);                 /* 5..95 mm right of the line */
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.found && r.left_seen && r.right_seen && ok;
	ok = absf(r.width_mm - 90.0f) <= 25.0f && ok;
	ok = r.right_mm - r.left_mm >= 60.0f && ok;   /* mostly right */
	return ok;
}

/* OBS-U11: dropped echo bridged, hidden edge, side object, nothing ahead. */
static BOOL robustness_test(void)
{
	spin_scan_sample_t s[N_FULL];
	spin_scan_result_t clean, r;
	BOOL ok = true;
	unsigned i, edge;

	model_scan(s, -45.0f, 45.0f);
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &clean) && ok;

	s[22].mm = OBS_DIST_NONE;                     /* one dropout at L6 */
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = absf(r.width_mm - clean.width_mm) < 0.1f && r.left_seen && ok;

	s[23].mm = OBS_DIST_NONE;                     /* two in a row: edge at L3 */
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.left_index == 21U && r.width_mm < clean.width_mm && ok;

	model_scan(s, -45.0f, 45.0f);                 /* nearer object takes over */
	edge = clean.left_index + 1U;
	s[edge].mm = 200U;
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.left_hidden && !r.left_seen && r.right_seen && ok;
	ok = r.left_free.object_seen && r.left_free.gap_mm < s_cfg.lane_mm &&
	     !r.left_free.passable && ok;
	{
		float trim;
		ok = !spin_scan_calibrate_trim(&r, &s_cfg, 90.0f, &trim) && ok;
	}

	model_scan(s, -45.0f, 45.0f);                 /* farther box out to the left */
	for (i = 33U; i < N_FULL; i++) s[i].mm = 420U;
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.left_seen && r.left_free.object_seen && ok;
	ok = absf(r.left_free.gap_mm - (r.left_free.object_mm - r.left_mm)) < 0.01f && ok;
	ok = r.left_free.passable == (r.left_free.gap_mm >= s_cfg.lane_mm) && ok;
	ok = absf(r.width_mm - clean.width_mm) < 0.1f && ok;

	/* An object far behind the obstacle (L21..L30 at 950 mm, about 890 mm
	 * ahead of the turning point; limit about 370 + 400) is past the bypass. */
	model_scan(s, -45.0f, 45.0f);
	for (i = 27U; i <= 30U; i++) s[i].mm = 950U;
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.left_seen && !r.left_free.object_seen && r.left_free.ignored == 4U &&
	     r.left_free.passable && r.right_free.ignored == 0U && ok;
	ok = absf(r.left_free.ahead_limit_mm - 770.0f) < 5.0f && ok;

	/* Box 5..95 mm right of the line not heard straight ahead: the wall
	 * behind (900 mm) fills the readings around 0 deg. The obstacle is the
	 * closest echo near the line, not the wall at 0 deg. */
	model_scan(s, -95.0f, -5.0f);
	for (i = 0U; i < N_FULL; i++)
		if (s[i].mm == OBS_DIST_NONE) s[i].mm = 900U;
	s[20].mm = 900U;                              /* 0 deg: wall */
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;
	ok = r.closest_mm < 400.0f && r.closest_deg < 0.0f && r.right_mm > r.left_mm && ok;
	for (i = 0U; i < N_FULL; i++) s[i].mm = 900U; /* only background */
	ok = !spin_scan_analyse(s, N_FULL, &s_cfg, &r) && ok;

	for (i = 0U; i < N_FULL; i++) s[i].mm = OBS_DIST_NONE;   /* nothing ahead */
	s[0].mm = 300U;                               /* only far off to the right */
	ok = !spin_scan_analyse(s, N_FULL, &s_cfg, &r) && !r.found && ok;
	ok = !spin_scan_analyse(s, 0U, &s_cfg, &r) && ok;
	return ok;
}

/* OBS-U12: live edge tracking, room angle, sweep start and stop. */
static BOOL tracker_test(void)
{
	spin_scan_edge_t e;
	BOOL ok = true;
	float room, expect;

	spin_scan_edge_start(&e, 300U, 0.0f);
	ok = !spin_scan_edge_update(&e, &s_cfg, -3.0f, 302U) && ok;
	ok = !spin_scan_edge_update(&e, &s_cfg, -6.0f, OBS_DIST_NONE) && e.misses == 1U && ok;
	ok = !spin_scan_edge_update(&e, &s_cfg, -9.0f, 305U) && e.misses == 0U && ok;
	ok = !spin_scan_edge_update(&e, &s_cfg, -12.0f, OBS_DIST_NONE) && ok;
	ok = spin_scan_edge_update(&e, &s_cfg, -15.0f, 500U) && e.lost && !e.hidden && ok;
	ok = absf(spin_scan_edge_deg(&e) + 10.5f) < 0.01f && ok;
	ok = spin_scan_edge_update(&e, &s_cfg, -18.0f, 300U) && ok;   /* stays lost */

	expect = __builtin_atan2f(70.0f * __builtin_sinf(10.5f * DEG_TO_RAD) +
			305.0f * __builtin_sinf((10.5f - 15.0f) * DEG_TO_RAD) + 200.0f,
			305.0f + 70.0f) / DEG_TO_RAD + SPIN_SCAN_ROOM_EXTRA_DEG;
	room = spin_scan_room_deg(&s_cfg, -10.5f, 305.0f);
	ok = absf(room - expect) < 0.01f && room > 10.5f && ok;
	ok = absf(spin_scan_sweep_start_deg(&s_cfg, &e, 3.0f) + room) < 0.01f && ok;

	{
		spin_scan_edge_t near;
		spin_scan_edge_start(&near, 300U, 0.0f);
		ok = !spin_scan_edge_update(&near, &s_cfg, -3.0f, 150U) && ok;
		ok = spin_scan_edge_update(&near, &s_cfg, -6.0f, OBS_DIST_NONE) && near.hidden && ok;
		/* An edge close to 0 deg: the start is SPIN_SCAN_LOST_MISSES fine
		 * steps past it unless the room needs more; never past half_deg. */
		ok = spin_scan_sweep_start_deg(&s_cfg, &near, 3.0f) <= -(1.5f + 6.0f) && ok;
	}

	spin_scan_edge_start(&e, 300U, 0.0f);         /* right edge never found */
	ok = absf(spin_scan_sweep_start_deg(&s_cfg, &e, 3.0f) + 60.0f) < 0.01f && ok;
	ok = absf(spin_scan_sweep_start_deg(&s_cfg, NULL, 3.0f) + 60.0f) < 0.01f && ok;

	/* Left edge: not lost -> keep going until half_deg. */
	ok = !spin_scan_sweep_done(&s_cfg, &e, 30.0f) && spin_scan_sweep_done(&s_cfg, &e, 60.0f) && ok;
	(void)spin_scan_edge_update(&e, &s_cfg, 9.0f, 305U);
	(void)spin_scan_edge_update(&e, &s_cfg, 12.0f, OBS_DIST_NONE);
	(void)spin_scan_edge_update(&e, &s_cfg, 15.0f, OBS_DIST_NONE);
	room = spin_scan_room_deg(&s_cfg, spin_scan_edge_deg(&e), 305.0f);
	ok = e.lost && !spin_scan_sweep_done(&s_cfg, &e, room - 0.5f) &&
	     spin_scan_sweep_done(&s_cfg, &e, room + 0.5f) && ok;
	return ok;
}

/* OBS-U13: sort by heading; calibrated trim reproduces the true width. */
static BOOL sort_calibrate_test(void)
{
	spin_scan_sample_t s[N_FULL], shuffled[N_FULL];
	spin_scan_result_t r, again;
	spin_scan_config_t cfg = s_cfg;
	BOOL ok = true;
	float trim;
	unsigned i;

	model_scan(s, -45.0f, 45.0f);
	for (i = 0U; i < N_FULL; i++) shuffled[i] = s[(i * 17U) % N_FULL];   /* 17 coprime to 41 */
	spin_scan_sort(shuffled, N_FULL);
	for (i = 0U; i < N_FULL; i++)
		ok = shuffled[i].deg == s[i].deg && shuffled[i].mm == s[i].mm && ok;

	cfg.trim_deg = 0.0f;
	ok = spin_scan_analyse(s, N_FULL, &cfg, &r) && ok;
	ok = spin_scan_calibrate_trim(&r, &cfg, 90.0f, &trim) && ok;
	ok = trim > 5.0f && trim < 25.0f && ok;
	cfg.trim_deg = trim;
	ok = spin_scan_analyse(s, N_FULL, &cfg, &again) && ok;
	ok = absf(again.width_mm - 90.0f) < 0.5f && ok;
	ok = absf(r.raw_width_mm - again.raw_width_mm) < 0.01f && ok;   /* raw ignores trim */
	return ok;
}

/* OBS-U16: bypass side and target (car 140, clearance 50, margin 80, tie 10). */
static BOOL bypass_plan_test(void)
{
	spin_scan_sample_t s[N_FULL];
	spin_scan_result_t r, m;
	spin_scan_bypass_t p;
	BOOL ok = true;
	const float out = 80.0f + 50.0f + 70.0f;

	/* Centred box from the model: a tie, so RIGHT. */
	model_scan(s, -45.0f, 45.0f);
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &m) && ok;
	ok = spin_scan_plan_bypass(&m, 140.0f, 50.0f, 80.0f, 10.0f, &p) && ok;
	ok = p.ok && p.preferred && p.side == SPIN_SCAN_BYPASS_RIGHT && ok;
	ok = absf(p.target_x - (m.right_mm + out)) < 0.01f && ok;
	/* Box 5..95 mm RIGHT of the line (model x + = left): sticks out less on
	 * the left, so LEFT, with the target left of its left edge. */
	model_scan(s, -95.0f, -5.0f);
	ok = spin_scan_analyse(s, N_FULL, &s_cfg, &m) && ok;
	ok = spin_scan_plan_bypass(&m, 140.0f, 50.0f, 80.0f, 10.0f, &p) && ok;
	ok = p.side == SPIN_SCAN_BYPASS_LEFT && p.preferred && ok;
	ok = absf(p.edge_x + m.left_mm) < 0.01f && absf(p.target_x - (p.edge_x - out)) < 0.01f && ok;
	ok = p.target_x < -150.0f && p.target_x > -250.0f && ok;

	/* Hand-made results. */
	r = (spin_scan_result_t){0};
	r.found = r.left_seen = r.right_seen = true;
	r.left_free.passable = r.right_free.passable = true;
	r.left_mm = 40.0f;
	r.right_mm = 48.0f;                       /* within the tie: RIGHT */
	ok = spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && p.side == SPIN_SCAN_BYPASS_RIGHT && ok;
	r.right_mm = 51.0f;                       /* beyond the tie: LEFT */
	ok = spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && p.side == SPIN_SCAN_BYPASS_LEFT && ok;
	ok = absf(p.target_x - (-40.0f - out)) < 0.01f && ok;
	/* Preferred side blocked: the other side, not preferred. */
	r.right_mm = 30.0f;
	r.right_free.passable = false;
	ok = spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && ok;
	ok = p.side == SPIN_SCAN_BYPASS_LEFT && !p.preferred && !p.right_usable && p.left_usable && ok;
	/* Right edge not seen (beyond the scan or hidden): not usable either. */
	r.right_free.passable = true;
	r.right_seen = false;
	ok = spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && p.side == SPIN_SCAN_BYPASS_LEFT && ok;
	/* Neither side usable, or nothing found: no plan. */
	r.left_free.passable = false;
	ok = !spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && !p.ok && ok;
	r = (spin_scan_result_t){0};
	ok = !spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && ok;
	/* Obstacle well left of the line: right is already clear, never move
	 * towards it. */
	r.found = r.left_seen = r.right_seen = true;
	r.left_free.passable = r.right_free.passable = true;
	r.left_mm = 400.0f;
	r.right_mm = -300.0f;
	ok = spin_scan_plan_bypass(&r, 140.0f, 50.0f, 80.0f, 10.0f, &p) && ok;
	ok = p.side == SPIN_SCAN_BYPASS_RIGHT && p.target_x == 0.0f && ok;
	return ok;
}

BOOL spin_scan_test(void)
{
	BOOL width_ok, robust_ok, tracker_ok, calibrate_ok, bypass_ok;
	TEST_INIT();

	width_ok = width_test();
	robust_ok = robustness_test();
	tracker_ok = tracker_test();
	calibrate_ok = sort_calibrate_test();
	bypass_ok = bypass_plan_test();
	TEST_CHECK("OBS-U10", width_ok, "spin scan width and extents, centred and off-centre");
	TEST_CHECK("OBS-U11", robust_ok, "spin scan dropout, hidden edge, side object, nothing ahead");
	TEST_CHECK("OBS-U12", tracker_ok, "spin scan edge tracking, room angle, sweep start/stop");
	TEST_CHECK("OBS-U13", calibrate_ok, "spin scan sort and trim calibration");
	TEST_CHECK("OBS-U16", bypass_ok, "spin scan bypass side (right by default, else shorter side) and target");
	TEST_SUMMARY("spin scan");
	return width_ok && robust_ok && tracker_ok && calibrate_ok && bypass_ok;
}
#endif
