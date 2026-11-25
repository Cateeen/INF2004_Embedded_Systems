/*
 * spin_scan.c - obstacle width from a spin-in-place ultrasonic scan.
 * See spin_scan.h for the scan sequence.
 *
 * Geometry: the robot turns about the midpoint between the wheels and the
 * sensor is sensor_offset_mm ahead of it. The beam spreads from the SENSOR,
 * so an edge heard at heading a with sensor distance d lies
 *     offset * sin(a) + d * sin(a - trim)
 * sideways from the line. The beam keeps hearing an object past its real
 * edge; trim pulls each edge back in (calibrate it with the spin_width test).
 */
#include "spin_scan.h"
#include "obs_profile_config.h"

#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD (3.14159265f / 180.0f)

static float absf(float v) { return v < 0.0f ? -v : v; }

bool spin_scan_valid_mm(uint16_t mm)
{
	return mm != OBS_DIST_NONE && mm >= OBS_MIN_RANGE_MM && mm <= OBS_MAX_RANGE_MM;
}

float spin_scan_extent(const spin_scan_config_t *config, float d_mm, float a_abs, float trim_deg)
{
	return config->sensor_offset_mm * sinf(a_abs * DEG_TO_RAD) +
	       d_mm * sinf((a_abs - trim_deg) * DEG_TO_RAD);
}

void spin_scan_edge_start(spin_scan_edge_t *edge, uint16_t ref_mm, float deg)
{
	edge->ref_mm = (float)ref_mm;
	edge->last_hit_deg = deg;
	edge->last_hit_mm = (float)ref_mm;
	edge->first_miss_deg = deg;
	edge->misses = 0U;
	edge->lost = false;
	edge->hidden = false;
}

bool spin_scan_edge_update(spin_scan_edge_t *edge, const spin_scan_config_t *config,
			   float deg, uint16_t mm)
{
	bool valid = spin_scan_valid_mm(mm);

	if (edge->lost) return true;
	if (valid && absf((float)mm - edge->ref_mm) <= config->cluster_mm) {
		edge->last_hit_deg = deg;
		edge->last_hit_mm = (float)mm;
		edge->misses = 0U;
		return false;
	}
	/* Not the obstacle: no echo, farther (past the edge) or nearer (another
	 * object takes over and hides the edge). */
	if (edge->misses == 0U) {
		edge->first_miss_deg = deg;
		edge->hidden = valid && (float)mm < edge->ref_mm;
	}
	edge->misses++;
	if (edge->misses >= SPIN_SCAN_LOST_MISSES) edge->lost = true;
	return edge->lost;
}

float spin_scan_edge_deg(const spin_scan_edge_t *edge)
{
	return 0.5f * (edge->last_hit_deg + edge->first_miss_deg);
}

float spin_scan_room_deg(const spin_scan_config_t *config, float edge_deg, float edge_mm)
{
	float a = absf(edge_deg);
	float lateral = spin_scan_extent(config, edge_mm, a, config->trim_deg) + config->lane_mm;
	float forward = edge_mm + config->sensor_offset_mm;
	float room;

	if (lateral < 0.0f) lateral = 0.0f;
	/* Plus a little: the final edges can come out a step from the live estimate. */
	room = atan2f(lateral, forward) / DEG_TO_RAD + SPIN_SCAN_ROOM_EXTRA_DEG;
	return room > config->half_deg ? config->half_deg : room;
}

float spin_scan_sweep_start_deg(const spin_scan_config_t *config, const spin_scan_edge_t *right_edge,
				float fine_step_deg)
{
	float start, room;

	if (right_edge == NULL || !right_edge->lost) return -config->half_deg;
	/* Far enough past the edge for SPIN_SCAN_LOST_MISSES fine readings that
	 * miss, and for the room beside it. */
	start = absf(spin_scan_edge_deg(right_edge)) + (float)SPIN_SCAN_LOST_MISSES * fine_step_deg;
	room = spin_scan_room_deg(config, spin_scan_edge_deg(right_edge), right_edge->last_hit_mm);
	if (room > start) start = room;
	if (start > config->half_deg) start = config->half_deg;
	return -start;
}

bool spin_scan_sweep_done(const spin_scan_config_t *config, const spin_scan_edge_t *left_edge,
			  float deg)
{
	if (deg >= config->half_deg) return true;
	if (!left_edge->lost) return false;
	return deg >= spin_scan_room_deg(config, spin_scan_edge_deg(left_edge), left_edge->last_hit_mm);
}

void spin_scan_sort(spin_scan_sample_t *samples, unsigned n)
{
	unsigned i, j;

	for (i = 1U; i < n; i++) {
		spin_scan_sample_t s = samples[i];

		for (j = i; j > 0U && samples[j - 1U].deg > s.deg; j--)
			samples[j] = samples[j - 1U];
		samples[j] = s;
	}
}

/* Free space beside the obstacle on one side: the nearest other echo among
 * samples [from, to), minus the obstacle's trimmed extent on that side. The
 * beam hears a side object before pointing at it, so its direction is pushed
 * out by the trim, as the obstacle's edges are pulled in. */
static void free_side(const spin_scan_sample_t *s, unsigned from, unsigned to, bool left,
		      const spin_scan_config_t *config, float obstacle_extent, float forward_mm,
		      float ahead_limit_mm, float max_abs_deg, spin_scan_side_t *out)
{
	float best = 1e9f, best_fwd = 0.0f;
	unsigned k;

	if (max_abs_deg < 0.0f) max_abs_deg = 0.0f;
	if (max_abs_deg > 80.0f) max_abs_deg = 80.0f;
	out->ignored = 0U;
	out->ahead_limit_mm = ahead_limit_mm;
	for (k = from; k < to; k++) {
		float a, lat, fwd;

		if (!spin_scan_valid_mm(s[k].mm)) continue;
		a = left ? s[k].deg : -s[k].deg;
		lat = config->sensor_offset_mm * sinf(a * DEG_TO_RAD) +
		      (float)s[k].mm * sinf((a + config->trim_deg) * DEG_TO_RAD);
		if (lat <= 0.0f) continue;
		/* Farther ahead than the bypass drives: not in its path. */
		fwd = ((float)s[k].mm + config->sensor_offset_mm) * cosf(s[k].deg * DEG_TO_RAD);
		if (fwd > ahead_limit_mm) {
			out->ignored++;
			continue;
		}
		if (lat < best) {
			best = lat;
			best_fwd = fwd;
		}
	}
	out->object_seen = best < 1e8f;
	out->object_mm = out->object_seen ? best : 0.0f;
	out->object_fwd_mm = best_fwd;
	out->gap_mm = out->object_seen ? best - obstacle_extent : 0.0f;
	/* On the beam's axis (a ray from the turning point) at the obstacle's
	 * distance: what spin_scan_room_deg() aims for. */
	out->reach_mm = forward_mm * tanf(max_abs_deg * DEG_TO_RAD);
	out->passable = out->object_seen ? out->gap_mm >= config->lane_mm
					 : out->reach_mm + 0.5f >= obstacle_extent + config->lane_mm;
}

bool spin_scan_analyse(const spin_scan_sample_t *s, unsigned n,
		       const spin_scan_config_t *config, spin_scan_result_t *out)
{
	unsigned i, centre = n, lh, rh;
	float best_deg = 1e9f, r_min = 1e9f, r_ref;

	*out = (spin_scan_result_t){0};
	if (s == NULL || n == 0U) return false;

	/* The obstacle is the closest echo within centre_zone_deg of the line and
	 * within max_obstacle_mm. Not simply the echo nearest 0 deg: a flat face
	 * beside the line may not echo straight ahead, and the reading there is
	 * then the background behind it. Objects farther off to the side are
	 * outside the zone. Ties go to the one nearer 0 deg. */
	{
		float best_mm = 1e9f;

		for (i = 0U; i < n; i++) {
			float mm = (float)s[i].mm, a = absf(s[i].deg);

			if (!spin_scan_valid_mm(s[i].mm) || a > config->centre_zone_deg ||
			    mm > config->max_obstacle_mm) continue;
			if (mm < best_mm || (mm == best_mm && a < best_deg)) {
				best_mm = mm;
				best_deg = a;
				centre = i;
			}
		}
	}
	if (centre == n) return false;

	/* Run of readings at the same distance as that echo, within the cluster
	 * tolerance either way: a nearer or farther object ends the run. */
	r_ref = (float)s[centre].mm;
#define HIT(k) (spin_scan_valid_mm(s[k].mm) && absf((float)s[k].mm - r_ref) <= config->cluster_mm)
#define CLOSER(k) (spin_scan_valid_mm(s[k].mm) && (float)s[k].mm < r_ref - config->cluster_mm)
	/* One missed reading (dropped echo) with the obstacle heard again on the
	 * next one is bridged, as the live tracker does; a nearer object is not. */
	lh = rh = centre;
	for (;;) {
		if (lh + 1U < n && HIT(lh + 1U)) lh++;
		else if (lh + 2U < n && !CLOSER(lh + 1U) && HIT(lh + 2U)) lh += 2U;
		else break;
	}
	for (;;) {
		if (rh > 0U && HIT(rh - 1U)) rh--;
		else if (rh > 1U && !CLOSER(rh - 1U) && HIT(rh - 2U)) rh -= 2U;
		else break;
	}
#undef CLOSER
#undef HIT
	for (i = rh; i <= lh; i++) {
		if (spin_scan_valid_mm(s[i].mm) && (float)s[i].mm < r_min) {
			r_min = (float)s[i].mm;
			centre = i;
		}
	}

	/* An edge is real only where the next reading stops hearing the obstacle
	 * (farther or no echo). If a NEARER object takes over, it hides the edge:
	 * the HC-SR04 reports the first echo, so the obstacle may continue behind
	 * it and the width is only a lower bound. */
#define NEARER(k) (spin_scan_valid_mm(s[k].mm) && (float)s[k].mm < r_min)
	out->left_hidden = lh + 1U < n && NEARER(lh + 1U);
	out->right_hidden = rh > 0U && NEARER(rh - 1U);
#undef NEARER
	out->left_seen = lh + 1U < n && !out->left_hidden;
	out->right_seen = rh > 0U && !out->right_hidden;

	/* Edge half way to the first reading that no longer hears the obstacle;
	 * a hidden or out-of-scan edge is at least as far as the last reading. */
	out->left_deg = out->left_seen ? 0.5f * (s[lh].deg + s[lh + 1U].deg) : s[lh].deg;
	out->right_deg = out->right_seen ? 0.5f * (s[rh].deg + s[rh - 1U].deg) : s[rh].deg;
	out->left_edge_mm = (float)s[lh].mm;
	out->right_edge_mm = (float)s[rh].mm;

	out->found = true;
	out->closest_index = centre;
	out->left_index = lh;
	out->right_index = rh;
	out->closest_mm = r_min;
	out->closest_deg = s[centre].deg;
	out->raw_left_mm = spin_scan_extent(config, out->left_edge_mm, out->left_deg, 0.0f);
	out->raw_right_mm = spin_scan_extent(config, out->right_edge_mm, -out->right_deg, 0.0f);
	out->left_mm = spin_scan_extent(config, out->left_edge_mm, out->left_deg, config->trim_deg);
	out->right_mm = spin_scan_extent(config, out->right_edge_mm, -out->right_deg, config->trim_deg);
	out->raw_width_mm = out->raw_left_mm + out->raw_right_mm;
	out->width_mm = out->left_mm + out->right_mm;
	out->mean_step_deg = n > 1U ? (s[n - 1U].deg - s[0].deg) / (float)(n - 1U) : 0.0f;

	{
		/* The obstacle's front, ahead of the turning point, plus how far the
		 * bypass drives beyond it. */
		float limit = (r_min + config->sensor_offset_mm) * cosf(out->closest_deg * DEG_TO_RAD) +
			      config->beyond_mm;

		free_side(s, lh + 1U, n, true, config, out->left_mm,
			  out->left_edge_mm + config->sensor_offset_mm, limit, s[n - 1U].deg,
			  &out->left_free);
		free_side(s, 0U, rh, false, config, out->right_mm,
			  out->right_edge_mm + config->sensor_offset_mm, limit, -s[0].deg,
			  &out->right_free);
	}
	return true;
}

bool spin_scan_calibrate_trim(const spin_scan_result_t *r, const spin_scan_config_t *config,
			      float true_width_mm, float *trim_deg)
{
	float lo = -30.0f, hi = 45.0f;
	unsigned i;

	if (!r->found || !r->left_seen || !r->right_seen) return false;
	/* Width falls as the trim rises: bisect for the true width. */
	for (i = 0U; i < 40U; i++) {
		float mid = 0.5f * (lo + hi);
		float w = spin_scan_extent(config, r->left_edge_mm, r->left_deg, mid) +
			  spin_scan_extent(config, r->right_edge_mm, -r->right_deg, mid);

		if (w > true_width_mm) lo = mid; else hi = mid;
	}
	*trim_deg = 0.5f * (lo + hi);
	return true;
}

bool spin_scan_plan_bypass(const spin_scan_result_t *r, float car_width_mm,
			   float clearance_mm, float margin_mm, float tie_mm,
			   spin_scan_bypass_t *plan)
{
	float out = margin_mm + clearance_mm + 0.5f * car_width_mm;
	bool want_right;

	if (plan == NULL) return false;
	*plan = (spin_scan_bypass_t){0};
	if (r == NULL || !r->found || car_width_mm <= 0.0f || clearance_mm < 0.0f || margin_mm < 0.0f)
		return false;
	plan->right_usable = r->right_seen && r->right_free.passable;
	plan->left_usable = r->left_seen && r->left_free.passable;
	want_right = r->right_mm <= r->left_mm + tie_mm;
	if (want_right ? plan->right_usable : plan->left_usable) {
		plan->preferred = true;
		plan->side = want_right ? SPIN_SCAN_BYPASS_RIGHT : SPIN_SCAN_BYPASS_LEFT;
	} else if (want_right ? plan->left_usable : plan->right_usable) {
		plan->side = want_right ? SPIN_SCAN_BYPASS_LEFT : SPIN_SCAN_BYPASS_RIGHT;
	} else {
		return false;
	}
	if (plan->side == SPIN_SCAN_BYPASS_RIGHT) {
		plan->edge_x = r->right_mm;
		plan->target_x = r->right_mm + out;
		if (plan->target_x < 0.0f) plan->target_x = 0.0f;
	} else {
		plan->edge_x = -r->left_mm;
		plan->target_x = -r->left_mm - out;
		if (plan->target_x > 0.0f) plan->target_x = 0.0f;
	}
	plan->ok = true;
	return true;
}
