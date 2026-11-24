#include "../../app.h"
#include "diag_pass_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "diag_pass.h"
#include "obs_profile_config.h"

static float absf(float v) { return v < 0.0f ? -v : v; }

/* Flat face at y = 370 spanning x0..x1: heard while the sensor is in front of
 * it (the bench result), else the wall at 900 mm or nothing. */
static uint16_t face(float x, float y, float x0, float x1, bool wall)
{
	if (x >= x0 && x <= x1) return (uint16_t)(370.0f - y + 0.5f);
	return wall ? (uint16_t)(900.0f - y) : (uint16_t)OBS_DIST_NONE;
}

/* Sweep the sensor from xa to xb in 3 mm steps, moving along y by dy per mm
 * sideways (a real 20 deg pass is 2.75 mm per mm; smaller here keeps the
 * synthetic distances in range). */
static void sweep(diag_pass_t *p, float xa, float xb, float y0, float dy_per_mm,
		  float x0, float x1, bool wall, unsigned *entered, unsigned *exited)
{
	float step = xb > xa ? 3.0f : -3.0f, x = xa, y = y0;

	for (; (step > 0.0f) ? x <= xb : x >= xb; x += step) {
		diag_event_t ev = diag_pass_feed(p, x, y, face(x, y, x0, x1, wall));

		if (ev == DIAG_EVENT_ENTERED) (*entered)++;
		if (ev == DIAG_EVENT_EXITED) (*exited)++;
		y += dy_per_mm * absf(step);
	}
}

/* OBS-U14: edges and width across a face, with and without a wall behind,
 * one dropped echo, and a start in front of the face. */
static BOOL pass_test(void)
{
	diag_pass_t p;
	unsigned en = 0U, ex = 0U;
	BOOL ok = true;

	/* Forward crossing left to right over a 90 mm face (-45..45). */
	diag_pass_start(&p, 450.0f, 60.0f, false);
	sweep(&p, -100.0f, 100.0f, 0.0f, 1.0f, -45.0f, 45.0f, false, &en, &ex);
	ok = en == 1U && ex == 1U && p.entry_known && ok;
	ok = absf(p.entry_x + 45.0f) <= 3.0f && absf(p.exit_x - 45.0f) <= 3.0f && ok;
	ok = absf((p.exit_x - p.entry_x) - 90.0f) <= 4.0f && ok;

	/* Same with the wall behind: the far echo is not the face. */
	en = ex = 0U;
	diag_pass_start(&p, 450.0f, 60.0f, false);
	sweep(&p, -100.0f, 100.0f, 0.0f, 1.0f, -45.0f, 45.0f, true, &en, &ex);
	ok = en == 1U && ex == 1U && absf((p.exit_x - p.entry_x) - 90.0f) <= 4.0f && ok;

	/* Reverse leg from in front of the face moving left and away: entry
	 * unknown, exit at the left edge; then the forward leg keeps the face. */
	en = ex = 0U;
	diag_pass_start(&p, 450.0f, 60.0f, false);
	sweep(&p, 20.0f, -120.0f, 0.0f, -2.75f, -45.0f, 45.0f, true, &en, &ex);
	ok = en == 1U && ex == 1U && !p.entry_known && absf(p.exit_x + 45.0f) <= 3.0f &&
	     p.have_face && ok;
	diag_pass_start(&p, 450.0f, 60.0f, true);
	ok = p.have_face && !p.on_face && ok;

	/* One dropped echo in the middle does not end the face. */
	{
		float x, y = 0.0f;
		unsigned dropped = 0U;

		en = ex = 0U;
		diag_pass_start(&p, 450.0f, 60.0f, false);
		for (x = -100.0f; x <= 100.0f; x += 3.0f) {
			uint16_t mm = face(x, y, -45.0f, 45.0f, false);
			diag_event_t ev;

			if (x > -2.0f && x < 1.0f) {          /* the sample at x = -1 */
				mm = OBS_DIST_NONE;
				dropped++;
			}
			ev = diag_pass_feed(&p, x, y, mm);
			if (ev == DIAG_EVENT_ENTERED) en++;
			if (ev == DIAG_EVENT_EXITED) ex++;
			y += 1.0f * 3.0f;
		}
		ok = dropped == 1U && en == 1U && ex == 1U &&
		     absf((p.exit_x - p.entry_x) - 90.0f) <= 4.0f && ok;
	}

	/* Nothing within face_max: never entered. */
	en = ex = 0U;
	diag_pass_start(&p, 450.0f, 60.0f, false);
	sweep(&p, -100.0f, 100.0f, 0.0f, 0.0f, 500.0f, 600.0f, true, &en, &ex);
	ok = en == 0U && ex == 0U && !p.have_face && ok;
	return ok;
}

/* OBS-U15: bypass side and clearing target. */
static BOOL plan_test(void)
{
	diag_plan_t plan;
	BOOL ok = true;

	/* Centred: tie, so RIGHT; car centre at edge + 70 + 50. */
	ok = diag_pass_plan(-45.0f, 45.0f, 140.0f, 50.0f, 10.0f, &plan) && ok;
	ok = plan.side == DIAG_SIDE_RIGHT && absf(plan.target_x - 165.0f) < 0.01f &&
	     absf(plan.width_mm - 90.0f) < 0.01f && ok;
	/* Sticks out 2 cm more on the right: LEFT. */
	ok = diag_pass_plan(-35.0f, 55.0f, 140.0f, 50.0f, 10.0f, &plan) && ok;
	ok = plan.side == DIAG_SIDE_LEFT && absf(plan.target_x + 155.0f) < 0.01f && ok;
	/* Within the tie margin: RIGHT. */
	ok = diag_pass_plan(-40.0f, 48.0f, 140.0f, 50.0f, 10.0f, &plan) && ok;
	ok = plan.side == DIAG_SIDE_RIGHT && ok;
	/* Box entirely right of the line: LEFT, and its left extent is negative. */
	ok = diag_pass_plan(5.0f, 95.0f, 140.0f, 50.0f, 10.0f, &plan) && ok;
	ok = plan.side == DIAG_SIDE_LEFT && plan.left_extent_mm < 0.0f &&
	     absf(plan.target_x + 115.0f) < 0.01f && ok;
	/* Invalid. */
	ok = !diag_pass_plan(45.0f, -45.0f, 140.0f, 50.0f, 10.0f, &plan) && ok;
	return ok;
}

BOOL diag_pass_test(void)
{
	BOOL pass_ok, plan_ok;
	TEST_INIT();

	pass_ok = pass_test();
	plan_ok = plan_test();
	TEST_CHECK("OBS-U14", pass_ok, "diagonal pass edges and width, wall behind, dropout, start on face");
	TEST_CHECK("OBS-U15", plan_ok, "diagonal pass bypass side and clearing target");
	TEST_SUMMARY("diagonal pass");
	return pass_ok && plan_ok;
}
#endif
