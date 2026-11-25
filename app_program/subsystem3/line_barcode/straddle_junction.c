#include "../../app.h"
#include "straddle_junction.h"

/* NULL comes from the kernel headers via app.h (<stddef.h> clashes with them). */

static bool valid_config(const straddle_junction_config_t *c)
{
	return c != NULL && __builtin_isfinite(c->both_min_mm) &&
	       __builtin_isfinite(c->gap_mm) && __builtin_isfinite(c->rearm_mm) &&
	       c->both_min_mm > 0.0f && c->gap_mm >= 0.0f && c->rearm_mm > 0.0f;
}

bool straddle_junction_init(straddle_junction_t *detector,
			    const straddle_junction_config_t *config)
{
	if (detector == NULL) return false;
	*detector = (straddle_junction_t){0};
	if (!valid_config(config)) return false;
	detector->config = *config;
	detector->armed = true;
	detector->initialized = true;
	return true;
}

void straddle_junction_reset(straddle_junction_t *detector)
{
	straddle_junction_config_t config;

	if (detector == NULL || !detector->initialized) return;
	config = detector->config;
	(void)straddle_junction_init(detector, &config);
}

straddle_junction_event_t straddle_junction_update(straddle_junction_t *j,
						   bool centre_black, bool right_black,
						   float distance_mm)
{
	const bool both = centre_black && right_black;
	float step = 0.0f;

	if (j == NULL || !j->initialized || !__builtin_isfinite(distance_mm))
		return STRADDLE_JUNCTION_NONE;
	if (j->have_last && distance_mm > j->last_mm) step = distance_mm - j->last_mm;
	j->last_mm = distance_mm;
	j->have_last = true;

	if (both) {
		if (!j->in_stretch && j->armed) {
			j->in_stretch = true;
			j->fired = false;
			j->both_mm = 0.0f;
		}
		if (!j->in_stretch) {
			j->clear_mm = 0.0f;     /* black again before re-arming */
			return STRADDLE_JUNCTION_NONE;
		}
		j->both_mm += step;
		j->gap_mm = 0.0f;
		if (!j->fired && j->both_mm >= j->config.both_min_mm) {
			j->fired = true;
			return STRADDLE_JUNCTION_DETECTED;
		}
		return STRADDLE_JUNCTION_NONE;
	}
	if (j->in_stretch) {
		j->gap_mm += step;
		if (j->gap_mm < j->config.gap_mm) return STRADDLE_JUNCTION_NONE;
		j->in_stretch = false;
		j->armed = !j->fired;           /* a narrow blip leaves it armed */
		j->clear_mm = j->gap_mm;
		if (!j->fired && j->both_mm > 0.0f) return STRADDLE_JUNCTION_NARROW;
		return STRADDLE_JUNCTION_NONE;
	}
	if (!j->armed) {
		j->clear_mm += step;
		if (j->clear_mm >= j->config.rearm_mm) j->armed = true;
	}
	return STRADDLE_JUNCTION_NONE;
}
