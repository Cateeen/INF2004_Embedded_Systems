#ifndef LINE_STEERING_H
#define LINE_STEERING_H

#include <stdbool.h>
#include <stdint.h>

/* Closed-loop line following: the result is a VELOCITY command for the
 * motion task (wheel targets left = speed - turn, right = speed + turn).
 * Speed + turn must stay within motion_task's max_linear_speed_cm_s.
 *
 * Ramped steering: the first correction on a side is TURN_MIN, and it grows
 * linearly to TURN (the full turn) while that side keeps being seen for
 * RAMP_MS. Straights flip side every ~150-250 ms, so their corrections stay
 * small (less wiggle); a curve holds one side, so the turn builds to full.
 * Changing side restarts at TURN_MIN; CENTRE between two same-side sightings
 * (riding the edge mid-curve) bleeds the build-up off at the same rate
 * instead of dropping it. TURN_MIN == TURN is the old fixed bang-bang.
 *
 * Two profiles: NORMAL, and SLOPE for while the IMU reports the robot on a
 * ramp (the controller calls line_steering_set_slope). Coming down a ramp
 * straight into a full turn at 12 cm/s made the robot lurch (outer wheel
 * 13..26 cm/s against 19, inner on ~8 % PWM), so SLOPE is slower with a
 * smaller full turn -- 9/5 still steers a ~10 cm radius, as tight as NORMAL,
 * for a bend right after the ramp.
 *
 * SENSOR_TEST=line_follow may override all of these (LF_SPEED, LF_TURN,
 * LF_TURN_MIN, LF_RAMP_MS; LF_SLOPE_*; LF_SLOPE=1 runs SLOPE throughout, as
 * the test has no IMU) to tune; copy what works here. */
#ifndef LINE_STEERING_SPEED_CM_S
#define LINE_STEERING_SPEED_CM_S    12
#endif
#ifndef LINE_STEERING_TURN_CM_S
#define LINE_STEERING_TURN_CM_S     7       /* full turn, reached after RAMP_MS */
#endif
#ifndef LINE_STEERING_TURN_MIN_CM_S
#define LINE_STEERING_TURN_MIN_CM_S 2       /* first correction on a side */
#endif
#ifndef LINE_STEERING_RAMP_MS
#define LINE_STEERING_RAMP_MS       400     /* TURN_MIN -> TURN on one side */
#endif

/* SLOPE profile (on a ramp, from the IMU). */
#ifndef LINE_STEERING_SLOPE_SPEED_CM_S
#define LINE_STEERING_SLOPE_SPEED_CM_S    9
#endif
#ifndef LINE_STEERING_SLOPE_TURN_CM_S
#define LINE_STEERING_SLOPE_TURN_CM_S     5
#endif
#ifndef LINE_STEERING_SLOPE_TURN_MIN_CM_S
#define LINE_STEERING_SLOPE_TURN_MIN_CM_S 2
#endif
#ifndef LINE_STEERING_SLOPE_RAMP_MS
#define LINE_STEERING_SLOPE_RAMP_MS       400
#endif

_Static_assert(LINE_STEERING_TURN_MIN_CM_S >= 1 &&
	       LINE_STEERING_TURN_MIN_CM_S <= LINE_STEERING_TURN_CM_S &&
	       LINE_STEERING_TURN_CM_S < LINE_STEERING_SPEED_CM_S,
	       "need 1 <= LINE_STEERING_TURN_MIN_CM_S <= LINE_STEERING_TURN_CM_S < SPEED");
_Static_assert(LINE_STEERING_RAMP_MS >= 50 && LINE_STEERING_RAMP_MS <= 5000,
	       "need 50 <= LINE_STEERING_RAMP_MS <= 5000");
_Static_assert(LINE_STEERING_SLOPE_TURN_MIN_CM_S >= 1 &&
	       LINE_STEERING_SLOPE_TURN_MIN_CM_S <= LINE_STEERING_SLOPE_TURN_CM_S &&
	       LINE_STEERING_SLOPE_TURN_CM_S < LINE_STEERING_SLOPE_SPEED_CM_S,
	       "need 1 <= SLOPE_TURN_MIN <= SLOPE_TURN < SLOPE_SPEED");
_Static_assert(LINE_STEERING_SLOPE_RAMP_MS >= 50 && LINE_STEERING_SLOPE_RAMP_MS <= 5000,
	       "need 50 <= LINE_STEERING_SLOPE_RAMP_MS <= 5000");

typedef struct {
	int16_t speed_cm_s;     /* forward speed */
	int16_t turn_cm_s;      /* full turn */
	int16_t turn_min_cm_s;  /* first correction on a side */
	uint16_t ramp_ms;       /* turn_min -> turn while one side is held */
} line_steering_profile_t;

extern const line_steering_profile_t line_steering_normal;
extern const line_steering_profile_t line_steering_slope;

typedef struct {
	int16_t linear_cm_s;    /* forward speed */
	int16_t turn_cm_s;      /* + turns left (left wheel slower) */
	bool stop_latched;
} line_steering_command_t;

typedef struct {
	uint32_t lost_since_ms;
	uint32_t last_ms;       /* previous update, for the ramp's elapsed time */
	uint32_t ramp_ms;       /* build-up on the current side */
	int16_t last_linear_cm_s;
	int16_t last_turn_cm_s;
	int8_t side;            /* +1 left, -1 right, 0 none yet */
	bool have_last;
	bool slope;             /* SLOPE profile active */
	bool line_lost;
	bool stop_latched;
} line_steering_t;

/* Clears everything, including SLOPE (back to NORMAL). */
void line_steering_reset(line_steering_t *steering);
line_steering_command_t line_steering_update(
	line_steering_t *steering, uint8_t position, uint32_t now_ms);

/* Select SLOPE (true) or NORMAL; takes effect at the next update. The
 * build-up and side are kept, so a turn in progress carries on (scaled to
 * the new profile) instead of restarting. */
void line_steering_set_slope(line_steering_t *steering, bool slope);

/* The profile the next update uses. */
const line_steering_profile_t *line_steering_profile(const line_steering_t *steering);

/* Turn magnitude after `ramp_ms` of build-up in `profile` (turn_min..turn). */
int16_t line_steering_ramp_turn(const line_steering_profile_t *profile, uint32_t ramp_ms);

#endif
