#ifndef STRADDLE_HWTEST_H
#define STRADDLE_HWTEST_H

/*
 * Straddle line following and junction detection for the line_follow and
 * junction hardware tests (LF_STRADDLE=1): this test's settings, fed to the
 * vehicle's own straddle_steering.c / straddle_junction.c.
 *
 * Sensor layout: the two line sensors (centre = left one, right) are set
 * FURTHER APART than the tape is wide, so the tape normally runs in the gap
 * between them (both white). The left/barcode sensor sits behind and to the
 * left and takes no part in steering or junctions.
 *
 * STEERING
 *   STRAIGHT  both white: tape in the gap, drive straight (no wiggle).
 *   CORRECT   one sensor black: gentle turn (LF_SL_TURN) towards it.
 *   RECOVER   the same sensor black for over LF_SL_OVERSHOOT_MS: the gentle
 *             turn is not keeping up, the robot is running past a bend.
 *             Pivot hard towards it (LF_SL_REC_SPEED -/+ LF_SL_REC_TURN,
 *             inner wheel ~stopped) until the tape slides into the gap.
 *   SEARCH    both white for over LF_SL_BLIND_MS after a sighting: on a sharp
 *             bend the tape crosses a sensor in a blip and ends up OUTSIDE
 *             both, which also reads both white. Pivot towards the sensor
 *             that saw it last. If the tape was really in the gap the OTHER
 *             sensor finds it after a small swing (a "false search").
 *             LF_SL_BLIND_MS=0 turns this off.
 *   COUNTER   a correction just ended (tape back in the gap). The correction
 *             turned the robot past parallel, so left alone it drives
 *             straight across the gap into the OTHER sensor and the cycle
 *             repeats (the wiggle). Steer the other way for
 *             LF_SL_COUNTER_PCT % of the correction's length (capped at
 *             LF_SL_COUNTER_MAX_MS) to take that extra heading back out,
 *             then STRAIGHT. LF_SL_COUNTER_PCT=0 turns it off.
 *   A pivot/search longer than LF_SL_PIVOT_MAX_MS latches a stop (lost).
 *   Both black (a crossing tape) drives straight.
 *
 * SENSOR HYSTERESIS
 *   A sensor half over the tape edge reads close to the reader's single
 *   black threshold and flickers. In straddle mode a line sensor turns black
 *   at >= LF_SL_BLACK_ADC and white again only at <= LF_SL_WHITE_ADC
 *   (sl_hysteresis); LF_SL_BLACK_ADC=0 keeps the reader's own threshold.
 *
 * JUNCTION
 *   With the sensors straddling the tape, BOTH can only be black at once when
 *   a tape crosses the path (a "+", or the top of a "T" from its stem). A
 *   junction is both black for JN_BOTH_MIN_MM of travel (encoder distance,
 *   so speed does not matter). Dropouts shorter than JN_GAP_MM inside the
 *   stretch are bridged (edge flicker); a stretch that ends before the
 *   minimum is reported "narrow" and leaves it armed. After a junction, both
 *   must stay not-black together for JN_REARM_MM before a new one can count,
 *   so one crossing is never counted twice.
 *   Not seen: a side branch (one sensor only, e.g. a "|-" from the stem) --
 *   that looks like an ordinary correction.
 */
#include <stdbool.h>
#include <stdint.h>
#include "../../subsystem3/line_barcode/line_observation.h"
#include "line_steering.h"

#ifndef LF_STRADDLE
#define LF_STRADDLE 0
#endif
/*
 * Steering settings default to the MAIN ROBOT's (vehicle_controller_task.h),
 * so the tests drive exactly as the vehicle does; a value given on the make
 * command line (LF_SPEED, LF_SL_*) overrides it for that build only.
 */
#include "vehicle_controller_task.h"
#ifndef LF_SL_SPEED
#define LF_SL_SPEED VEHICLE_LINE_SPEED_CM_S       /* straight / correcting speed */
#endif
#ifndef LF_SL_TURN
#define LF_SL_TURN VEHICLE_LINE_TURN_CM_S         /* gentle correction */
#endif
#ifndef LF_SL_COUNTER_PCT
#define LF_SL_COUNTER_PCT VEHICLE_LINE_COUNTER_PCT /* % of the correction; 0 = off */
#endif
#ifndef LF_SL_COUNTER_MAX_MS
#define LF_SL_COUNTER_MAX_MS VEHICLE_LINE_COUNTER_MAX_MS
#endif
#ifndef LF_SL_BLACK_ADC
#define LF_SL_BLACK_ADC 1500     /* line sensor black at/above this; 0 = reader's threshold */
#endif
#ifndef LF_SL_WHITE_ADC
#define LF_SL_WHITE_ADC 900      /* ... and white again only at/below this */
#endif
#ifndef LF_SL_OVERSHOOT_MS
#define LF_SL_OVERSHOOT_MS VEHICLE_LINE_OVERSHOOT_MS /* one sensor this long = past a bend */
#endif
#ifndef LF_SL_REC_SPEED
#define LF_SL_REC_SPEED VEHICLE_LINE_PIVOT_SPEED_CM_S /* pivot: REC_SPEED -/+ REC_TURN */
#endif
#ifndef LF_SL_REC_TURN
#define LF_SL_REC_TURN VEHICLE_LINE_PIVOT_TURN_CM_S
#endif
#ifndef LF_SL_BLIND_MS
#define LF_SL_BLIND_MS VEHICLE_LINE_BLIND_MS       /* both white this long -> search; 0 = off */
#endif
#ifndef LF_SL_PIVOT_MAX_MS
#define LF_SL_PIVOT_MAX_MS VEHICLE_LINE_PIVOT_MAX_MS /* a pivot/search this long = lost */
#endif
#ifndef JN_GAP_MM
#define JN_GAP_MM 3              /* bridge both-black dropouts shorter than this */
#endif
#ifndef JN_REARM_MM
#define JN_REARM_MM 30           /* not-both-black this far before the next junction */
#endif

#if LF_STRADDLE
_Static_assert(LF_SL_TURN >= 1 && LF_SL_TURN < LF_SL_SPEED && LF_SL_SPEED + LF_SL_TURN <= 20,
	       "need 1 <= LF_SL_TURN < LF_SPEED and LF_SPEED + LF_SL_TURN <= 20 cm/s");
_Static_assert(LF_SL_REC_SPEED >= 1 && LF_SL_REC_TURN >= 1 &&
	       LF_SL_REC_SPEED + LF_SL_REC_TURN <= 20,
	       "need LF_SL_REC_SPEED, LF_SL_REC_TURN >= 1 and their sum <= 20 cm/s");
_Static_assert(LF_SL_OVERSHOOT_MS >= 50U && LF_SL_OVERSHOOT_MS <= 3000U,
	       "need 50 <= LF_SL_OVERSHOOT_MS <= 3000");
_Static_assert(LF_SL_PIVOT_MAX_MS >= 500U && LF_SL_PIVOT_MAX_MS <= 10000U,
	       "need 500 <= LF_SL_PIVOT_MAX_MS <= 10000");
_Static_assert(LF_SL_BLIND_MS <= 10000U, "need LF_SL_BLIND_MS <= 10000 (0 = off)");
_Static_assert(LF_SL_COUNTER_PCT >= 0 && LF_SL_COUNTER_PCT <= 150 &&
	       LF_SL_COUNTER_MAX_MS <= 2000U,
	       "need 0 <= LF_SL_COUNTER_PCT <= 150 and LF_SL_COUNTER_MAX_MS <= 2000");
_Static_assert(LF_SL_BLACK_ADC == 0 ||
	       (LF_SL_WHITE_ADC < LF_SL_BLACK_ADC && LF_SL_BLACK_ADC <= 4095),
	       "need LF_SL_WHITE_ADC < LF_SL_BLACK_ADC <= 4095 (or LF_SL_BLACK_ADC=0)");
_Static_assert(JN_GAP_MM >= 0 && JN_GAP_MM <= 20 && JN_REARM_MM >= 5 && JN_REARM_MM <= 300,
	       "need 0 <= JN_GAP_MM <= 20 and 5 <= JN_REARM_MM <= 300");

/* The logic lives in the vehicle code (straddle_steering.c and
 * straddle_junction.c); these wrappers feed it this test's LF_SL_* / JN_*
 * settings, so the tests and the vehicle run the same algorithm. */
#include "straddle_steering.h"
#include "../../subsystem3/line_barcode/straddle_junction.h"

static inline const char *sl_state_name(unsigned state)
{
	return straddle_state_name(state);
}

/* Line-sensor hysteresis (see SENSOR HYSTERESIS above); call on every
 * reading before line_following_position. */
static inline void sl_hysteresis(ir_readings_t *ir)
{
	static bool centre_black, right_black;

	if (LF_SL_BLACK_ADC == 0) return;
	if (ir->center.raw >= LF_SL_BLACK_ADC) centre_black = true;
	else if (ir->center.raw <= LF_SL_WHITE_ADC) centre_black = false;
	if (ir->right.raw >= LF_SL_BLACK_ADC) right_black = true;
	else if (ir->right.raw <= LF_SL_WHITE_ADC) right_black = false;
	ir->center.black = centre_black;
	ir->right.black = right_black;
}

static inline void sl_reset(straddle_t *s, int16_t speed_cm_s)
{
	const straddle_config_t config = {
		speed_cm_s, LF_SL_TURN, (uint16_t)LF_SL_OVERSHOOT_MS,
		LF_SL_REC_SPEED, LF_SL_REC_TURN, (uint16_t)LF_SL_BLIND_MS,
		(uint16_t)LF_SL_PIVOT_MAX_MS, (uint8_t)LF_SL_COUNTER_PCT,
		(uint16_t)LF_SL_COUNTER_MAX_MS
	};

	(void)straddle_init(s, &config);        /* refused = latched stop */
}

static inline line_steering_command_t sl_update(straddle_t *s, uint8_t pos, uint32_t now)
{
	return straddle_update(s, pos, now);
}

/* ---- junction: both line sensors black together ---- */

#define SJ_NONE     STRADDLE_JUNCTION_NONE
#define SJ_DETECTED STRADDLE_JUNCTION_DETECTED
#define SJ_NARROW   STRADDLE_JUNCTION_NARROW

static inline void sj_reset(straddle_junction_t *j, float both_min_mm)
{
	const straddle_junction_config_t config = {
		both_min_mm, (float)JN_GAP_MM, (float)JN_REARM_MM
	};

	(void)straddle_junction_init(j, &config);
}

static inline straddle_junction_event_t sj_update(straddle_junction_t *j, bool centre_black,
						  bool right_black, float distance_mm)
{
	return straddle_junction_update(j, centre_black, right_black, distance_mm);
}

#endif /* LF_STRADDLE */

#endif
