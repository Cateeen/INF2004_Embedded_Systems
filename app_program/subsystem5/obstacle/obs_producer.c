#include "obs_producer.h"

#include <stddef.h>

static bool valid_distance(uint16_t distance)
{
	return distance == OBS_DIST_NONE ||
	       (distance >= OBS_MIN_RANGE_MM && distance <= OBS_MAX_RANGE_MM);
}

static bool ordered(bool have_previous, uint32_t previous, uint32_t timestamp)
{
	uint32_t elapsed = timestamp - previous;
	return !have_previous || (elapsed != 0U && elapsed <= INT32_MAX);
}

static void init_event(obstacle_event_message_t *event, uint32_t sequence,
		       uint32_t timestamp)
{
	event->header.type = VEHICLE_MESSAGE_OBSTACLE_EVENT;
	event->header.version = VEHICLE_MESSAGE_VERSION;
	event->header.sequence = sequence;
	event->header.timestamp_ms = timestamp;
}

bool obs_producer_init(obs_producer_t *producer, uint16_t emergency_distance_mm)
{
	obs_producer_t initial = {0};
	if (producer == NULL || emergency_distance_mm < OBS_MIN_RANGE_MM ||
	    emergency_distance_mm > OBS_MAX_RANGE_MM)
		return false;
	initial.emergency_distance_mm = emergency_distance_mm;
	initial.initialized = true;
	*producer = initial;
	return true;
}

bool obs_producer_ahead(obs_producer_t *producer, uint16_t distance_mm,
			uint32_t timestamp_ms, obstacle_event_message_t *event)
{
	obstacle_event_message_t next = {0};
	if (producer == NULL || event == NULL || !producer->initialized ||
	    !valid_distance(distance_mm) ||
	    !ordered(producer->have_ahead, producer->last_ahead_ms, timestamp_ms))
		return false;
	init_event(&next, producer->sequence + 1U, timestamp_ms);
	next.obstacle_class = VEHICLE_OBSTACLE_AHEAD_READING;
	next.distance_mm = distance_mm;
	next.emergency = distance_mm != OBS_DIST_NONE &&
			 distance_mm <= producer->emergency_distance_mm;
	producer->sequence = next.header.sequence;
	producer->last_ahead_ms = timestamp_ms;
	producer->have_ahead = true;
	*event = next;
	return true;
}

bool obs_producer_scan(obs_producer_t *producer, const uint16_t *distances,
		       uint8_t count, uint8_t angle0_deg, uint8_t step_deg,
		       uint32_t timestamp_ms, obstacle_event_message_t *event)
{
	obstacle_event_message_t next = {0};
	obs_profile_t profile;
	obs_profile_status_t status;
	uint16_t closest = OBS_DIST_NONE;
	unsigned i;

	if (producer == NULL || event == NULL || !producer->initialized ||
	    !ordered(producer->have_scan, producer->last_scan_ms, timestamp_ms))
		return false;
	status = obs_profile_compute(distances, count, angle0_deg, step_deg, &profile);
	if (status == OBS_PROFILE_INVALID) return false;
	for (i = 0U; i < count; i++) {
		if (distances[i] != OBS_DIST_NONE && distances[i] < closest)
			closest = distances[i];
	}
	init_event(&next, producer->sequence + 1U, timestamp_ms);
	next.distance_mm = closest;
	next.emergency = closest != OBS_DIST_NONE &&
			 closest <= producer->emergency_distance_mm;
	if (status == OBS_PROFILE_CLEAR) {
		next.obstacle_class = VEHICLE_OBSTACLE_SCAN_CLEAR;
	} else {
		next.obstacle_class = VEHICLE_OBSTACLE_SCAN_FOUND;
		next.target_distance_mm = profile.closest_mm;
		next.width_mm = profile.width_mm;
		next.left_mm = profile.left_mm;
		next.right_mm = profile.right_mm;
		next.lateral_mm = profile.lateral_mm;
		next.advance_mm = profile.advance_mm;
		switch (profile.action) {
		case OBS_ACT_LEFT: next.advisory_action = VEHICLE_OBSTACLE_ADVICE_LEFT; break;
		case OBS_ACT_RIGHT: next.advisory_action = VEHICLE_OBSTACLE_ADVICE_RIGHT; break;
		case OBS_ACT_REVERSE: next.advisory_action = VEHICLE_OBSTACLE_ADVICE_REVERSE; break;
		default: return false;
		}
		switch (profile.object_side) {
		case OBS_SIDE_CENTRE: next.object_side = VEHICLE_OBSTACLE_SIDE_CENTRE; break;
		case OBS_SIDE_LEFT: next.object_side = VEHICLE_OBSTACLE_SIDE_LEFT; break;
		case OBS_SIDE_RIGHT: next.object_side = VEHICLE_OBSTACLE_SIDE_RIGHT; break;
		default: next.object_side = VEHICLE_OBSTACLE_SIDE_UNKNOWN; break;
		}
		next.edges_seen = (uint8_t)((profile.left_edge_seen ?
					     VEHICLE_OBSTACLE_EDGE_LEFT_SEEN : 0U) |
					    (profile.right_edge_seen ?
					     VEHICLE_OBSTACLE_EDGE_RIGHT_SEEN : 0U));
	}
	producer->sequence = next.header.sequence;
	producer->last_scan_ms = timestamp_ms;
	producer->have_scan = true;
	*event = next;
	return true;
}
