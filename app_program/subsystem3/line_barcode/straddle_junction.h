#ifndef STRADDLE_JUNCTION_H
#define STRADDLE_JUNCTION_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Junction detection for straddle line following (the two line sensors
 * further apart than the tape, see straddle_steering.h). Both can only be
 * black at once when a tape crosses the path (a "+", or the top of a "T"
 * reached from its stem); the left/barcode sensor is not used.
 *
 * A junction is centre AND right black for both_min_mm of travel (distance
 * from the encoders, so speed does not matter). Dropouts shorter than gap_mm
 * inside the stretch are bridged (edge flicker); a stretch that ends before
 * the minimum is reported NARROW and leaves the detector armed. After a
 * junction both must stay not-black together for rearm_mm before another one
 * counts, so one crossing is never counted twice (and a spin on the spot,
 * which adds no travel, never re-arms it).
 *
 * Not seen: a side branch (one sensor only) -- it looks like a correction.
 * Pure logic: feed every sample with the distance travelled.
 */

typedef struct {
	float both_min_mm;
	float gap_mm;
	float rearm_mm;
} straddle_junction_config_t;

/* Measured with SENSOR_TEST=junction LF_STRADDLE=1: an 18 mm crossing tape
 * reads ~6 mm both black at the detection (~12 mm in all). */
#define STRADDLE_JUNCTION_DEFAULT_CONFIG { 5.0f, 3.0f, 30.0f }

typedef enum {
	STRADDLE_JUNCTION_NONE = 0,
	STRADDLE_JUNCTION_DETECTED,
	STRADDLE_JUNCTION_NARROW        /* both black, but shorter than both_min_mm */
} straddle_junction_event_t;

typedef struct {
	straddle_junction_config_t config;
	float last_mm;
	float both_mm;          /* both-black travel in the current / last stretch */
	float gap_mm;           /* not-both travel since the last both-black sample */
	float clear_mm;         /* not-both travel since the last junction ended */
	bool have_last;
	bool armed;
	bool in_stretch;
	bool fired;
	bool initialized;
} straddle_junction_t;

/* Validates (both_min_mm > 0, gap_mm >= 0, rearm_mm > 0, all finite). */
bool straddle_junction_init(straddle_junction_t *detector,
			    const straddle_junction_config_t *config);

/* Armed again, no stretch in progress; keeps the config. */
void straddle_junction_reset(straddle_junction_t *detector);

/* One sample: the two line sensors and the distance travelled (mm, forward;
 * going backwards counts as no travel). */
straddle_junction_event_t straddle_junction_update(straddle_junction_t *detector,
						   bool centre_black, bool right_black,
						   float distance_mm);

#if APP_TEST
#include <tk/tkernel.h>
BOOL straddle_junction_test(void);
#endif

#endif
