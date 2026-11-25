#include "line_steering.h"
#include "../../subsystem3/line_barcode/line_observation.h"

#include <stddef.h>

#define LOST_CONTINUE_MS 300U
/* Longest gap one update may add to the ramp (the controller runs every
 * 50 ms); a stall or the first update after a pause cannot jump to full. */
#define RAMP_STEP_MAX_MS 100U

const line_steering_profile_t line_steering_normal = {
	LINE_STEERING_SPEED_CM_S, LINE_STEERING_TURN_CM_S,
	LINE_STEERING_TURN_MIN_CM_S, LINE_STEERING_RAMP_MS
};

const line_steering_profile_t line_steering_slope = {
	LINE_STEERING_SLOPE_SPEED_CM_S, LINE_STEERING_SLOPE_TURN_CM_S,
	LINE_STEERING_SLOPE_TURN_MIN_CM_S, LINE_STEERING_SLOPE_RAMP_MS
};

void line_steering_reset(line_steering_t *steering)
{
	if (steering == NULL) return;
	*steering = (line_steering_t){0};
}

void line_steering_set_slope(line_steering_t *steering, bool slope)
{
	if (steering != NULL) steering->slope = slope;
}

const line_steering_profile_t *line_steering_profile(const line_steering_t *steering)
{
	return (steering != NULL && steering->slope) ? &line_steering_slope :
						       &line_steering_normal;
}

int16_t line_steering_ramp_turn(const line_steering_profile_t *profile, uint32_t ramp_ms)
{
	uint32_t span;

	if (profile == NULL) profile = &line_steering_normal;
	span = (uint32_t)(profile->turn_cm_s - profile->turn_min_cm_s);
	if (ramp_ms > profile->ramp_ms) ramp_ms = profile->ramp_ms;
	return (int16_t)(profile->turn_min_cm_s +
			 (int32_t)((span * ramp_ms + profile->ramp_ms / 2U) / profile->ramp_ms));
}

line_steering_command_t line_steering_update(
	line_steering_t *steering, uint8_t position, uint32_t now_ms)
{
	const line_steering_profile_t *profile;
	line_steering_command_t stop = {0, 0, true};
	line_steering_command_t command;
	uint32_t dt;
	int8_t side;

	if (steering == NULL || steering->stop_latched) return stop;
	profile = line_steering_profile(steering);
	command = (line_steering_command_t){profile->speed_cm_s, 0, false};

	dt = steering->have_last ? (uint32_t)(now_ms - steering->last_ms) : 0U;
	if (dt > RAMP_STEP_MAX_MS) dt = RAMP_STEP_MAX_MS;
	steering->last_ms = now_ms;
	steering->have_last = true;

	if (position == POS_LOST) {
		if (!steering->line_lost) {
			steering->line_lost = true;
			steering->lost_since_ms = now_ms;
			steering->lost_ramp_ms = steering->ramp_ms;  /* turning, or not */
		}
		if (steering->slope) {
			/* On a hump (a straight): drive straight blind, no stop,
			 * capped; the normal 300 ms counts from the hump's end. */
			if (!steering->slope_lost) {
				steering->slope_lost = true;
				steering->slope_lost_since_ms = now_ms;
			}
			if ((uint32_t)(now_ms - steering->slope_lost_since_ms) >=
			    (uint32_t)LINE_STEERING_SLOPE_LOST_MAX_MS) {
				steering->stop_latched = true;
				return stop;
			}
			steering->lost_since_ms = now_ms;
			steering->last_linear_cm_s = profile->speed_cm_s;
			steering->last_turn_cm_s = 0;
			command.linear_cm_s = profile->speed_cm_s;
			command.turn_cm_s = 0;
			return command;
		}
		if ((uint32_t)(now_ms - steering->lost_since_ms) >= LOST_CONTINUE_MS) {
			steering->stop_latched = true;
			return stop;
		}
		/* Keep turning towards the side the line was last seen on, and
		 * keep BUILDING the turn as if that side were still seen (right-
		 * edge following: on a left curve the tape escapes left past the
		 * centre sensor, often while the build-up is still small, and
		 * holding that weak turn ran wide). A brief loss on a straight
		 * barely changes the turn (no swing); on a curve the build-up is
		 * already high and carries on to full. Lost on a hump keeps
		 * going straight (slope_lost). */
		command.linear_cm_s = steering->last_linear_cm_s;
		command.turn_cm_s = steering->last_turn_cm_s;
		if (!steering->slope_lost && steering->side != 0) {
			steering->ramp_ms += dt;
			if (steering->ramp_ms > profile->ramp_ms)
				steering->ramp_ms = profile->ramp_ms;
			command.turn_cm_s = (int16_t)(steering->side *
				line_steering_ramp_turn(profile, steering->ramp_ms));
		}
		return command;
	}

	steering->line_lost = false;
	steering->slope_lost = false;
	switch (position) {
	case POS_JUNCTION:
		/* Centred: drive straight, and bleed the build-up off so a brief
		 * CENTRE mid-curve resumes near the turn it had. */
		steering->ramp_ms = steering->ramp_ms > dt ? steering->ramp_ms - dt : 0U;
		break;
	case POS_LEFT:
	case POS_RIGHT:
		side = position == POS_LEFT ? 1 : -1;
		if (side != steering->side) {
			steering->side = side;
			steering->ramp_ms = 0U;
		} else {
			steering->ramp_ms += dt;
			if (steering->ramp_ms > profile->ramp_ms)
				steering->ramp_ms = profile->ramp_ms;
		}
		command.turn_cm_s = (int16_t)(side * line_steering_ramp_turn(profile,
									      steering->ramp_ms));
		break;
	default:
		steering->stop_latched = true;
		return stop;
	}

	steering->last_linear_cm_s = command.linear_cm_s;
	steering->last_turn_cm_s = command.turn_cm_s;
	return command;
}
