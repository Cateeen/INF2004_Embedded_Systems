#include "../app.h"
#include "obs_profile_test.h"

#if APP_TEST
#include "../test_report.h"
#include "obs_profile.h"
#include "obs_producer.h"
#include "hcsr04.h"

static BOOL clear_and_centre_test(void)
{
	uint16_t scan[9] = {
		OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE,
		OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE,
		OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE
	};
	obs_profile_t profile;

	if (obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_CLEAR ||
	    profile.valid || profile.action != OBS_ACT_CONTINUE) return false;
	scan[4] = OBS_BLOCK_MM;
	if (obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_CLEAR)
		return false;
	scan[4] = 300U;
	if (obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_FOUND)
		return false;
	return profile.valid && profile.closest_mm == 300U &&
	       profile.action == OBS_ACT_RIGHT &&
	       profile.left_mm == OBS_MAX_RANGE_MM &&
	       profile.right_mm == OBS_MAX_RANGE_MM &&
	       profile.advance_mm >= 658U && profile.advance_mm <= 662U;
}

static BOOL edge_and_cluster_test(void)
{
	uint16_t scan[9] = {
		300U, 300U, OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE,
		OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE
	};
	obs_profile_t profile;
	unsigned i;

	if (obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_FOUND ||
	    profile.right_mm != 0U || profile.action != OBS_ACT_LEFT) return false;
	scan[0] = OBS_DIST_NONE;
	scan[1] = OBS_DIST_NONE;
	scan[7] = 300U;
	scan[8] = 300U;
	if (obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_FOUND ||
	    profile.left_mm != 0U || profile.action != OBS_ACT_RIGHT) return false;
	scan[7] = OBS_DIST_NONE;
	scan[8] = OBS_DIST_NONE;
	scan[0] = 200U;
	scan[1] = OBS_DIST_NONE;
	scan[4] = 500U;
	if (obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_FOUND ||
	    profile.closest_mm != 500U) return false;
	for (i = 0U; i < 9U; i++) scan[i] = 300U;
	return obs_profile_compute(scan, 9U, 80U, 5U, &profile) == OBS_PROFILE_FOUND &&
	       profile.right_mm == 0U && profile.left_mm == 0U &&
	       profile.action == OBS_ACT_REVERSE;
}

static BOOL producer_test(void)
{
	obs_producer_t producer;
	obstacle_event_message_t event = {0};
	uint16_t scan[9] = {
		200U, OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE, 500U,
		OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE, OBS_DIST_NONE
	};
	unsigned i;
	BOOL ok;

	ok = !obs_producer_init(&producer, 0U) &&
	     !obs_producer_init(&producer, OBS_MAX_RANGE_MM + 1U) &&
	     obs_producer_init(&producer, 300U);
	ok = obs_producer_ahead(&producer, 300U, UINT32_MAX, &event) &&
	     event.header.type == VEHICLE_MESSAGE_OBSTACLE_EVENT &&
	     event.header.sequence == 1U && event.emergency &&
	     event.obstacle_class == VEHICLE_OBSTACLE_AHEAD_READING &&
	     event.distance_mm == 300U && ok;
	ok = obs_producer_ahead(&producer, OBS_DIST_NONE, 0U, &event) &&
	     !event.emergency && event.distance_mm == OBS_DIST_NONE &&
	     event.header.sequence == 2U && ok;
	ok = !obs_producer_ahead(&producer, 0U, 1U, &event) &&
	     !obs_producer_ahead(&producer, 400U, 0U, &event) &&
	     event.header.sequence == 2U && producer.sequence == 2U && ok;
	ok = obs_producer_scan(&producer, scan, 9U, 80U, 5U, 1U, &event) &&
	     event.obstacle_class == VEHICLE_OBSTACLE_SCAN_FOUND &&
	     event.distance_mm == 200U && event.target_distance_mm == 500U &&
	     event.emergency && event.advisory_action == VEHICLE_OBSTACLE_ADVICE_LEFT &&
	     event.header.sequence == 3U && ok;
	scan[1] = 0U;
	ok = !obs_producer_scan(&producer, scan, 9U, 80U, 5U, 2U, &event) &&
	     event.header.sequence == 3U && producer.sequence == 3U && ok;
	for (i = 0U; i < 9U; i++) scan[i] = OBS_DIST_NONE;
	ok = obs_producer_scan(&producer, scan, 9U, 80U, 5U, 2U, &event) &&
	     event.obstacle_class == VEHICLE_OBSTACLE_SCAN_CLEAR &&
	     event.distance_mm == OBS_DIST_NONE && !event.emergency &&
	     event.advisory_action == VEHICLE_OBSTACLE_ADVICE_NONE && ok;
	return ok;
}

static BOOL invalid_test(void)
{
	uint16_t scan[9] = {0};
	obs_profile_t profile = {.valid = true, .action = OBS_ACT_RIGHT};

	if (obs_profile_compute(NULL, 9U, 80U, 5U, &profile) != OBS_PROFILE_INVALID ||
	    profile.valid || profile.action != OBS_ACT_STOP) return false;
	if (obs_profile_compute(scan, 0U, 80U, 5U, &profile) != OBS_PROFILE_INVALID ||
	    obs_profile_compute(scan, 10U, 80U, 5U, &profile) != OBS_PROFILE_INVALID ||
	    obs_profile_compute(scan, 9U, 80U, 0U, &profile) != OBS_PROFILE_INVALID ||
	    obs_profile_compute(scan, 9U, 170U, 5U, &profile) != OBS_PROFILE_INVALID ||
	    obs_profile_compute(scan, 9U, 80U, 5U, NULL) != OBS_PROFILE_INVALID ||
	    obs_profile_compute(scan, 9U, 80U, 5U, &profile) != OBS_PROFILE_INVALID)
		return false;
	scan[0] = OBS_MAX_RANGE_MM + 1U;
	return obs_profile_compute(scan, 1U, 80U, 5U, &profile) == OBS_PROFILE_INVALID;
}

static BOOL hcsr04_distance_test(void)
{
	uint16_t distance = 123U;
	BOOL ok;

	ok = hcsr04_distance_mm(1000U, 343000U, &distance) &&
	     distance == 172U && ok;
	ok = hcsr04_distance_mm(5000U, 343000U, &distance) &&
	     distance == 858U && ok;
	distance = 123U;
	ok = !hcsr04_distance_mm(0U, 343000U, &distance) &&
	     distance == 123U && ok;
	ok = !hcsr04_distance_mm(1000U, 0U, &distance) &&
	     !hcsr04_distance_mm(UINT32_MAX, 400000U, &distance) &&
	     distance == 123U && ok;
	return ok;
}

BOOL obs_profile_test(void)
{
	BOOL centre_ok;
	BOOL edges_ok;
	BOOL invalid_ok;
	BOOL producer_ok;
	BOOL hcsr04_ok;
	TEST_INIT();

	centre_ok = clear_and_centre_test();
	edges_ok = edge_and_cluster_test();
	invalid_ok = invalid_test();
	producer_ok = producer_test();
	hcsr04_ok = hcsr04_distance_test();
	TEST_CHECK("OBS-U01", centre_ok, "clear scan and centred obstacle");
	TEST_CHECK("OBS-U02", edges_ok, "scan edges and cluster selection");
	TEST_CHECK("OBS-U03", invalid_ok, "invalid scan rejected");
	TEST_CHECK("OBS-U04", producer_ok, "typed ahead and scan events");
	TEST_CHECK("OBS-U07", hcsr04_ok, "ultrasonic echo width to distance conversion");
	TEST_SUMMARY("obstacle profile");
	return centre_ok && edges_ok && invalid_ok && producer_ok && hcsr04_ok;
}
#endif
