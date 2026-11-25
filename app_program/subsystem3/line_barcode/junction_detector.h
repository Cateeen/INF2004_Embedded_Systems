#ifndef JUNCTION_DETECTOR_H
#define JUNCTION_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Junction detection from the LEFT (barcode) IR sensor plus the two line
 * sensors -- not from "both line sensors black", which happens in normal
 * edge following (bends, the ramp, partial coverage).
 *
 * Geometry: the left sensor is ~40 mm left of and a little AHEAD of the centre
 * line sensor (its spot leads by ~8 mm, measured); barcodes sit ~20 mm left
 * of the line, under the left sensor.
 * A tape crossing the path (a "+", or the top of a "T" reached from its stem)
 * therefore shows as:
 *   1. the left sensor black for about the tape width as it measures it
 *      (left_min..left_max mm; 18 mm tape measures ~12, barcode bars less
 *      than ~6), and
 *   2. centre AND right both black for at least both_min mm somewhere in the
 *      window where the crossing reaches them: from offset - tol mm after the
 *      left sensor went black, to offset + left width + tol mm.
 * Distances come from the wheel encoders, so speed does not matter.
 *
 * Pure logic: feed every sensor sample with the distance travelled.
 */

typedef struct {
	float left_min_mm;      /* narrower left black = barcode bar / noise */
	float left_max_mm;      /* wider = not a crossing tape (e.g. a long patch) */
	float offset_mm;        /* left sensor's lead over the line sensors */
	float tol_mm;           /* slack each side of the expected window */
	float both_min_mm;      /* centre+right black needed inside the window */
} junction_config_t;

/* Defaults measured with SENSOR_TEST=junction (no-line start over an 18 mm
 * tape): the sensors see tape ~6 mm NARROWER than it is (left 11.6 mm, line
 * sensors ~12 mm -- only the middle reads clearly black), and the line
 * sensors went black ~7-8 mm after the left one, not 20 (the left sensor's
 * spot leads by less than its body). Barcode bars shrink the same way
 * (a ~9 mm wide bar ~3-5 mm), so 8 mm still separates them from a crossing. */
#define JUNCTION_DEFAULT_CONFIG { 8.0f, 80.0f, 8.0f, 12.0f, 5.0f }

typedef enum {
	JUNCTION_NONE = 0,
	JUNCTION_DETECTED,          /* crossing confirmed */
	JUNCTION_REJECT_NARROW,     /* left black too narrow: barcode bar / noise */
	JUNCTION_REJECT_WIDE,       /* left black too wide for a crossing tape */
	JUNCTION_REJECT_UNCONFIRMED /* left width fine, line sensors never both black */
} junction_event_t;

typedef struct {
	junction_config_t config;
	float last_mm;          /* distance at the previous sample */
	bool have_last;
	/* Current candidate: one continuous left-black stretch. */
	bool active;            /* a candidate is being evaluated */
	bool left_on;           /* its left-black stretch is still going */
	float start_mm;         /* where the left sensor went black */
	float left_mm;          /* left-black width so far */
	float both_mm;          /* centre+right black inside the window */
	bool wait_white;        /* after a too-wide stretch, until the left sees white */
	bool initialized;
} junction_detector_t;

bool junction_detector_init(junction_detector_t *detector, const junction_config_t *config);

/* Forget any candidate (e.g. after a turn); keeps the config. */
void junction_detector_reset(junction_detector_t *detector);

/* One sample: the three black flags and the distance travelled (mm, forward;
 * going backwards counts as no travel). Returns an event when a candidate is
 * decided, else JUNCTION_NONE. After an event, junction_detector_last()
 * describes the candidate that produced it. */
junction_event_t junction_detector_update(junction_detector_t *detector, bool left_black,
					  bool centre_black, bool right_black,
					  float distance_mm);

/* The latest candidate: left-black width and centre+right black in window. */
void junction_detector_last(const junction_detector_t *detector, float *left_mm,
			    float *both_mm);

#if APP_TEST
#include <tk/tkernel.h>
BOOL junction_detector_test(void);
#endif

#endif
