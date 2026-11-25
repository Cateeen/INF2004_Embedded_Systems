#ifndef STRADDLE_STEERING_H
#define STRADDLE_STEERING_H

#include <stdbool.h>
#include <stdint.h>
#include "line_steering.h"      /* line_steering_command_t */

/*
 * Straddle line following.
 *
 * Sensor layout: the two line sensors (centre = the left one, and right) sit
 * FURTHER APART than the tape is wide, so the tape normally runs in the gap
 * between them (both white: POS_LOST from line_following_position). The
 * left/barcode sensor sits behind and to the left and takes no part.
 *
 *   STRAIGHT  both white: tape in the gap, drive straight (no wiggle).
 *   CORRECT   one sensor black: gentle turn (turn_cm_s) towards it.
 *   COUNTER   a correction just ended (tape back in the gap). It turned the
 *             robot past parallel, so left alone it would drive across the
 *             gap into the other sensor (the wiggle). Steer the other way
 *             for counter_pct % of the correction's length (capped at
 *             counter_max_ms), then STRAIGHT. counter_pct 0 = off.
 *   RECOVER   the same sensor black for over overshoot_ms: the gentle turn
 *             is not keeping up, the robot is running past a bend. Pivot
 *             towards it (pivot_speed -/+ pivot_turn) until the tape slides
 *             back into the gap. No counter-steer after a pivot.
 *   SEARCH    both white for over blind_ms after a sighting: on a sharp bend
 *             the tape crosses a sensor in a blip and ends up OUTSIDE both,
 *             which also reads both white. Pivot towards the sensor that saw
 *             it last; if it was really in the gap the other sensor finds it
 *             after a small swing (a "false search"). blind_ms 0 = off. Not
 *             while straddle_set_hump(true): over a hump's crest the tape is
 *             often out of sight and straight on is right.
 *   A pivot or search longer than pivot_max_ms latches a stop (line lost)
 *   until straddle_reset. Both black (a crossing tape) drives straight.
 *
 * Pure logic, called once per controller cycle with the latest position.
 * The command is the motion task's VELOCITY pair (left = linear - turn,
 * right = linear + turn; + turn = left).
 */

typedef struct {
	int16_t speed_cm_s;         /* straight and correcting speed */
	int16_t turn_cm_s;          /* gentle correction */
	uint16_t overshoot_ms;      /* one sensor on the tape this long = past a bend */
	int16_t pivot_speed_cm_s;   /* RECOVER / SEARCH */
	int16_t pivot_turn_cm_s;
	uint16_t blind_ms;          /* both white this long -> SEARCH; 0 = off */
	uint16_t pivot_max_ms;      /* a pivot / search longer than this = lost */
	uint8_t counter_pct;        /* counter-steer, % of the correction; 0 = off */
	uint16_t counter_max_ms;
} straddle_config_t;

typedef enum {
	STRADDLE_STRAIGHT = 0,
	STRADDLE_CORRECT,
	STRADDLE_RECOVER,
	STRADDLE_SEARCH,
	STRADDLE_COUNTER
} straddle_state_t;

typedef struct {
	straddle_config_t config;
	straddle_state_t state;
	int8_t side;            /* +1 left/centre sensor, -1 right: the way it steers */
	int8_t last_side;       /* sensor that saw the tape last; 0 = none yet */
	uint32_t since;         /* when the current state began */
	uint32_t clear_since;   /* both white since */
	uint32_t counter_ms;    /* length of the current COUNTER */
	bool clear;
	bool hump;              /* no SEARCH while set */
	bool stop_latched;
	bool initialized;
	/* Statistics (since the last reset). */
	unsigned corrections;
	unsigned recoveries;
	unsigned searches;
	unsigned false_searches;        /* found on the OTHER sensor: was in the gap */
	unsigned counters;
	uint32_t longest_correct_ms;
	uint32_t longest_recover_ms;
} straddle_t;

/* Validates the config (speeds and turns >= 1, turn < speed, overshoot and
 * pivot_max >= 50 ms, counter_pct <= 150); false leaves the steering latched
 * stopped. */
bool straddle_init(straddle_t *steering, const straddle_config_t *config);

/* Back to STRAIGHT with no side seen and the stop latch cleared; keeps the
 * config and the hump flag. */
void straddle_reset(straddle_t *steering);

/* On a hump (from the IMU): no SEARCH while set. */
void straddle_set_hump(straddle_t *steering, bool hump);

/* Change the straight/correcting speed on the fly (e.g. slower over a
 * barcode), keeping the state; pivots keep their own speed. Refused (false,
 * unchanged) unless turn_cm_s < speed_cm_s. */
bool straddle_set_speed(straddle_t *steering, int16_t speed_cm_s);

/* One update with the latest POS_* (line_following_position). Unknown
 * positions latch the stop. */
line_steering_command_t straddle_update(straddle_t *steering, uint8_t position,
					uint32_t now_ms);

const char *straddle_state_name(unsigned state);

#if APP_TEST
#include <tk/tkernel.h>
BOOL straddle_steering_test(void);
#endif

#endif
