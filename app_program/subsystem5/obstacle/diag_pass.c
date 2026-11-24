/*
 * diag_pass.c - obstacle width from a diagonal pass. See diag_pass.h.
 */
#include "diag_pass.h"
#include "obs_profile_config.h"

#include <stddef.h>

static float absf(float v) { return v < 0.0f ? -v : v; }

static bool valid_mm(uint16_t mm)
{
	return mm != OBS_DIST_NONE && mm >= OBS_MIN_RANGE_MM && mm <= OBS_MAX_RANGE_MM;
}

void diag_pass_start(diag_pass_t *p, float face_max_mm, float cluster_mm, bool keep_face)
{
	bool have_face = keep_face && p->have_face;
	float face_y = p->face_y;

	*p = (diag_pass_t){0};
	p->face_max_mm = face_max_mm;
	p->cluster_mm = cluster_mm;
	p->have_face = have_face;
	p->face_y = have_face ? face_y : 0.0f;
}

bool diag_pass_expected_mm(const diag_pass_t *p, float sensor_y, float *mm)
{
	if (!p->have_face) return false;
	*mm = p->face_y - sensor_y;
	return true;
}

/* The face: before its position is known, any echo within face_max_mm (the
 * robot stopped about 300 mm from it; the background is much farther). Then
 * an echo within cluster_mm of where the face should be. */
static bool is_hit(const diag_pass_t *p, float y, uint16_t mm)
{
	float face_y;

	if (!valid_mm(mm)) return false;
	if (p->have_face) face_y = p->face_y;
	else if (p->hit_run > 0U) face_y = p->pending_face_y;   /* confirming a first hit */
	else return (float)mm <= p->face_max_mm;
	return absf((float)mm - (face_y - y)) <= p->cluster_mm;
}

diag_event_t diag_pass_feed(diag_pass_t *p, float x, float y, uint16_t mm)
{
	bool hit = is_hit(p, y, mm);

	if (!p->on_face) {
		if (!hit) {
			p->hit_run = 0U;
			p->have_miss = true;
			p->last_miss_x = x;
			return DIAG_EVENT_NONE;
		}
		if (p->hit_run == 0U) {
			/* Edge half way between the last miss and this first hit. */
			p->pending_entry_known = p->have_miss;
			p->pending_entry_x = p->have_miss ? 0.5f * (p->last_miss_x + x) : x;
			p->pending_face_y = y + (float)mm;
		}
		p->hit_run++;
		p->last_hit_x = x;
		if (p->hit_run < DIAG_PASS_CONFIRM) return DIAG_EVENT_NONE;
		p->on_face = true;
		p->miss_run = 0U;
		p->entry_known = p->pending_entry_known;
		p->entry_x = p->pending_entry_x;
		if (!p->have_face) {
			p->have_face = true;
			p->face_y = 0.5f * (p->pending_face_y + y + (float)mm);
		}
		return DIAG_EVENT_ENTERED;
	}

	if (hit) {
		p->miss_run = 0U;
		p->last_hit_x = x;
		/* Follow the face slowly (it may not be exactly square). */
		p->face_y += 0.2f * ((y + (float)mm) - p->face_y);
		return DIAG_EVENT_NONE;
	}
	if (p->miss_run == 0U) p->pending_exit_x = 0.5f * (p->last_hit_x + x);
	p->miss_run++;
	if (p->miss_run < DIAG_PASS_LOST) return DIAG_EVENT_NONE;
	p->on_face = false;
	p->hit_run = 0U;
	p->have_miss = true;
	p->last_miss_x = x;
	p->exit_x = p->pending_exit_x;
	return DIAG_EVENT_EXITED;
}

bool diag_pass_plan(float left_x, float right_x, float car_width_mm, float clearance_mm,
		    float tie_mm, diag_plan_t *plan)
{
	float half = 0.5f * car_width_mm;

	if (plan == NULL || right_x < left_x || car_width_mm <= 0.0f || clearance_mm < 0.0f)
		return false;
	plan->left_x = left_x;
	plan->right_x = right_x;
	plan->width_mm = right_x - left_x;
	plan->left_extent_mm = -left_x;
	plan->right_extent_mm = right_x;
	if (plan->right_extent_mm <= plan->left_extent_mm + tie_mm) {
		plan->side = DIAG_SIDE_RIGHT;
		plan->target_x = right_x + half + clearance_mm;
	} else {
		plan->side = DIAG_SIDE_LEFT;
		plan->target_x = left_x - half - clearance_mm;
	}
	return true;
}
