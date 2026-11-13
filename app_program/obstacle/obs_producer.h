#ifndef OBS_PRODUCER_H
#define OBS_PRODUCER_H

#include "obs_profile.h"
#include "../vehicle_messages.h"

typedef struct {
	uint32_t sequence;
	uint32_t last_ahead_ms;
	uint32_t last_scan_ms;
	uint16_t emergency_distance_mm;
	bool initialized;
	bool have_ahead;
	bool have_scan;
} obs_producer_t;

/* The single owner supplies the emergency threshold in millimetres. */
bool obs_producer_init(obs_producer_t *producer, uint16_t emergency_distance_mm);

/* OBS_DIST_NONE must mean confirmed no object, never a failed measurement.
 * The owner checks timestamp freshness and reports failures. These functions
 * do not latch safety state, send messages, or prove that a path is clear.
 * Ahead and scan timestamps are ordered independently; an older scan can
 * therefore follow an ahead event. On failure, producer and event are unchanged. */
bool obs_producer_ahead(obs_producer_t *producer, uint16_t distance_mm,
			uint32_t timestamp_ms, obstacle_event_message_t *event);

/* timestamp_ms is a conservative time no later than the oldest scan sample.
 * Advice is unverified and must not directly command motion. */
bool obs_producer_scan(obs_producer_t *producer, const uint16_t *distances,
		       uint8_t count, uint8_t angle0_deg, uint8_t step_deg,
		       uint32_t timestamp_ms, obstacle_event_message_t *event);

#endif
