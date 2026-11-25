#include "../../app.h"
#include "straddle_steering.h"
#include "../../subsystem3/line_barcode/line_observation.h"

/* NULL comes from the kernel headers via app.h (<stddef.h> clashes with them). */

static bool valid_config(const straddle_config_t *c)
{
	return c != NULL && c->speed_cm_s >= 1 && c->turn_cm_s >= 1 &&
	       c->turn_cm_s < c->speed_cm_s && c->pivot_speed_cm_s >= 1 &&
	       c->pivot_turn_cm_s >= 1 && c->overshoot_ms >= 50U &&
	       c->pivot_max_ms >= 50U && c->counter_pct <= 150U;
}

bool straddle_init(straddle_t *steering, const straddle_config_t *config)
{
	if (steering == NULL) return false;
	*steering = (straddle_t){0};
	if (!valid_config(config)) {
		steering->stop_latched = true;
		return false;
	}
	steering->config = *config;
	steering->initialized = true;
	return true;
}

void straddle_reset(straddle_t *steering)
{
	straddle_config_t config;
	bool hump;

	if (steering == NULL || !steering->initialized) return;
	config = steering->config;
	hump = steering->hump;
	(void)straddle_init(steering, &config);
	steering->hump = hump;
}

void straddle_set_hump(straddle_t *steering, bool hump)
{
	if (steering != NULL) steering->hump = hump;
}

bool straddle_set_speed(straddle_t *steering, int16_t speed_cm_s)
{
	straddle_config_t config;

	if (steering == NULL || !steering->initialized) return false;
	config = steering->config;
	config.speed_cm_s = speed_cm_s;
	if (!valid_config(&config)) return false;
	steering->config.speed_cm_s = speed_cm_s;
	return true;
}

const char *straddle_state_name(unsigned state)
{
	static const char *const names[5] = {
		"STRAIGHT", "CORRECT", "RECOVER", "SEARCH", "COUNTER"
	};
	return state < 5U ? names[state] : "?";
}

static void enter(straddle_t *s, straddle_state_t state, int8_t side, uint32_t now)
{
	s->state = state;
	s->side = side;
	s->since = now;
}

line_steering_command_t straddle_update(straddle_t *s, uint8_t position, uint32_t now)
{
	const line_steering_command_t stop = { 0, 0, true };
	line_steering_command_t cmd;
	const straddle_config_t *c;
	int8_t seen;

	if (s == NULL || !s->initialized || s->stop_latched) return stop;
	c = &s->config;
	cmd = (line_steering_command_t){ c->speed_cm_s, 0, false };
	switch (position) {
	case POS_LEFT:     seen = 1; break;    /* centre (left) sensor on the tape */
	case POS_RIGHT:    seen = -1; break;
	case POS_JUNCTION:
	case POS_LOST:     seen = 0; break;
	default:
		s->stop_latched = true;
		return stop;
	}

	if (seen != 0) {
		s->clear = false;
		s->last_side = seen;
		if (s->state == STRADDLE_SEARCH) {
			/* Found it. Same sensor: it had escaped outside; the other
			 * one: it was in the gap all along. Steer to it either way. */
			if (seen != s->side) s->false_searches++;
			enter(s, STRADDLE_CORRECT, seen, now);
			s->corrections++;
		} else if (s->state == STRADDLE_RECOVER && seen == s->side) {
			/* still under the sensor: keep pivoting */
		} else if (s->state != STRADDLE_CORRECT || seen != s->side) {
			enter(s, STRADDLE_CORRECT, seen, now);
			s->corrections++;
		}
		if (s->state == STRADDLE_CORRECT &&
		    (uint32_t)(now - s->since) >= c->overshoot_ms) {
			/* Held too long: running past a bend. Pivot back to it. */
			enter(s, STRADDLE_RECOVER, seen, now);
			s->recoveries++;
		}
	} else if (position == POS_JUNCTION) {
		/* Both black: a crossing tape. Straight, unless mid-pivot. */
		s->clear = false;
		if (s->state != STRADDLE_RECOVER && s->state != STRADDLE_SEARCH)
			enter(s, STRADDLE_STRAIGHT, 0, now);
	} else {
		/* Both white: the tape is in the gap (or escaped outside). */
		if (!s->clear || s->hump) {
			s->clear = true;
			s->clear_since = now;       /* on a hump: never search */
		}
		if (s->state == STRADDLE_CORRECT) {
			/* Correction over: take back part of the heading it added. */
			uint32_t ms = (uint32_t)(now - s->since) * (uint32_t)c->counter_pct / 100U;

			if (ms > c->counter_max_ms) ms = c->counter_max_ms;
			if (ms > 0U) {
				enter(s, STRADDLE_COUNTER, (int8_t)-s->side, now);
				s->counter_ms = ms;
				s->counters++;
			} else {
				enter(s, STRADDLE_STRAIGHT, 0, now);
			}
		} else if (s->state == STRADDLE_RECOVER) {
			enter(s, STRADDLE_STRAIGHT, 0, now);
		} else if (s->state == STRADDLE_COUNTER &&
			   (uint32_t)(now - s->since) >= s->counter_ms) {
			enter(s, STRADDLE_STRAIGHT, 0, now);
		}
		if (s->state == STRADDLE_STRAIGHT && c->blind_ms > 0U && s->last_side != 0 &&
		    (uint32_t)(now - s->clear_since) >= c->blind_ms) {
			enter(s, STRADDLE_SEARCH, s->last_side, now);
			s->searches++;
		}
	}

	if ((s->state == STRADDLE_RECOVER || s->state == STRADDLE_SEARCH) &&
	    (uint32_t)(now - s->since) >= c->pivot_max_ms) {
		s->stop_latched = true;
		return stop;
	}
	if (s->state == STRADDLE_CORRECT && now - s->since > s->longest_correct_ms)
		s->longest_correct_ms = now - s->since;
	if (s->state == STRADDLE_RECOVER && now - s->since > s->longest_recover_ms)
		s->longest_recover_ms = now - s->since;

	switch (s->state) {
	case STRADDLE_CORRECT:
	case STRADDLE_COUNTER:      /* side already points the other way */
		cmd.turn_cm_s = (int16_t)(s->side * c->turn_cm_s);
		break;
	case STRADDLE_RECOVER:
	case STRADDLE_SEARCH:
		cmd.linear_cm_s = c->pivot_speed_cm_s;
		cmd.turn_cm_s = (int16_t)(s->side * c->pivot_turn_cm_s);
		break;
	default:
		break;
	}
	return cmd;
}
