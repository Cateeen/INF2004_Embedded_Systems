#include "../../app.h"
#include "odometry_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "encoder.h"
#include "encoder_pair.h"
#include "wheel_odometry.h"

static BOOL encoder_pair_test(void)
{
	encoder_pair_t pair;
	encoder_snapshot_t snapshot;
	uint32_t pins = 0U;

	encoder_pair_init(&pair, pins);
	pins = (1U << ENCODER_LEFT_B_PIN) | (1U << ENCODER_RIGHT_A_PIN);
	encoder_pair_update(&pair, 0x00000880U, pins);
	pins |= (1U << ENCODER_LEFT_A_PIN) | (1U << ENCODER_RIGHT_B_PIN);
	encoder_pair_update(&pair, 0x00008008U, pins);
	snapshot = encoder_pair_snapshot(&pair);
	return snapshot.valid && snapshot.left_count == 2 && snapshot.right_count == -2;
}

/* Left wheel pins for a quadrature state (A << 1) | B. */
static uint32_t left_pins(uint32_t state)
{
	return (((state >> 1U) & 1U) << ENCODER_LEFT_A_PIN) |
	       ((state & 1U) << ENCODER_LEFT_B_PIN);
}

/* One clean forward step on the left wheel: 00 -> 01 -> 11 -> 10 -> 00. */
static void left_step(encoder_pair_t *pair, uint32_t *state)
{
	static const uint8_t next[4] = {1U, 3U, 0U, 2U};
	uint32_t n = next[*state];
	uint32_t events = ((n ^ *state) == 2U) ? ((n & 2U) ? 0x08U : 0x04U) :
						((n & 1U) ? 0x80U : 0x40U);

	encoder_pair_update(pair, events, left_pins(n));
	*state = n;
}

/* Both left channels jump together in one IRQ (edges on both). */
static void left_jump(encoder_pair_t *pair, uint32_t *state)
{
	uint32_t n = *state ^ 3U;
	uint32_t events = ((n & 2U) ? 0x08U : 0x04U) | ((n & 1U) ? 0x80U : 0x40U);

	encoder_pair_update(pair, events, left_pins(n));
	*state = n;
}

static BOOL encoder_pair_invalid_test(void)
{
	encoder_pair_t pair;
	encoder_snapshot_t snapshot;
	uint32_t pins;
	uint32_t state = 0U;
	unsigned i;

	/* Both-channel jumps beyond the tolerated limit latch invalid. */
	encoder_pair_init(&pair, 0U);
	for (i = 0U; i <= ENCODER_AMBIGUOUS_LIMIT; i++)
		left_jump(&pair, &state);
	snapshot = encoder_pair_snapshot(&pair);
	if (snapshot.valid || encoder_pair_fault(&pair).reason != ENCODER_FAULT_MULTI_EDGE)
		return FALSE;
	pins = left_pins(state);
	pins &= ~(1U << ENCODER_LEFT_A_PIN);
	encoder_pair_update(&pair, 0x00000004U, pins);
	if (encoder_pair_snapshot(&pair).valid) return FALSE;

	encoder_pair_init(&pair, 0U);
	encoder_pair_update(&pair, 0x00000004U, 1U << ENCODER_LEFT_A_PIN);
	if (encoder_pair_snapshot(&pair).valid) return FALSE;

	encoder_pair_init(&pair, 0U);
	encoder_pair_update(&pair, 0x00000008U,
			    (1U << ENCODER_LEFT_A_PIN) | (1U << ENCODER_LEFT_B_PIN));
	return !encoder_pair_snapshot(&pair).valid;
}

/* One-channel spikes resolve exactly from the levels: count them, stay valid. */
static BOOL encoder_pair_glitch_test(void)
{
	encoder_pair_t pair;
	encoder_snapshot_t snapshot;
	uint32_t pins = 0U;

	encoder_pair_init(&pair, pins);
	/* Right B rose and fell inside one IRQ (as seen on the car): net zero. */
	encoder_pair_update(&pair, 0x0000c000U, pins);
	/* A lone latched edge whose pulse already ended: net zero. */
	encoder_pair_update(&pair, 0x00000080U, pins);
	snapshot = encoder_pair_snapshot(&pair);
	if (!snapshot.valid || snapshot.left_count != 0 || snapshot.right_count != 0 ||
	    encoder_pair_fault(&pair).glitches != 2U)
		return FALSE;
	/* Left B toggled an odd number of times: one real step (00 -> 01 = +1). */
	pins = 1U << ENCODER_LEFT_B_PIN;
	encoder_pair_update(&pair, 0x000000c0U, pins);
	snapshot = encoder_pair_snapshot(&pair);
	if (!snapshot.valid || snapshot.left_count != 1 ||
	    encoder_pair_fault(&pair).glitches != 3U ||
	    encoder_pair_fault(&pair).reason != ENCODER_FAULT_NONE)
		return FALSE;
	/* Right A rose (a real step, 00 -> 10 = -1) while right B spiked up and
	 * down in the same IRQ (seen line following): one step, not ambiguous. */
	pins |= 1U << ENCODER_RIGHT_A_PIN;
	encoder_pair_update(&pair, 0x0000c800U, pins);
	snapshot = encoder_pair_snapshot(&pair);
	return snapshot.valid && snapshot.left_count == 1 && snapshot.right_count == -1 &&
	       encoder_pair_fault(&pair).glitches == 4U &&
	       encoder_pair_fault(&pair).reason == ENCODER_FAULT_NONE;
}

/* Both-channel jumps: tolerated as 0 steps up to the limit, clean travel
 * earns them back, one more beyond the limit latches; wheels are separate. */
static BOOL encoder_pair_ambiguous_test(void)
{
	encoder_pair_t pair;
	encoder_snapshot_t snapshot;
	uint32_t state = 0U;
	unsigned i;

	encoder_pair_init(&pair, 0U);
	for (i = 0U; i < ENCODER_AMBIGUOUS_LIMIT; i++)
		left_jump(&pair, &state);
	snapshot = encoder_pair_snapshot(&pair);
	if (!snapshot.valid || snapshot.left_count != 0 ||
	    encoder_pair_fault(&pair).ambiguous != ENCODER_AMBIGUOUS_LIMIT)
		return FALSE;
	/* The right wheel has its own allowance. */
	encoder_pair_update(&pair, 0x00008800U, left_pins(state) |
			    (1U << ENCODER_RIGHT_A_PIN) | (1U << ENCODER_RIGHT_B_PIN));
	if (!encoder_pair_snapshot(&pair).valid) return FALSE;
	/* Clean travel earns back one strike, so one more jump is absorbed. */
	for (i = 0U; i < ENCODER_AMBIGUOUS_LEAK_STEPS; i++)
		left_step(&pair, &state);
	left_jump(&pair, &state);
	snapshot = encoder_pair_snapshot(&pair);
	if (!snapshot.valid || snapshot.left_count != (int32_t)ENCODER_AMBIGUOUS_LEAK_STEPS ||
	    encoder_pair_fault(&pair).ambiguous != ENCODER_AMBIGUOUS_LIMIT + 2U)
		return FALSE;
	/* Back at the limit: the next one latches, recorded on the left wheel. */
	left_jump(&pair, &state);
	snapshot = encoder_pair_snapshot(&pair);
	return !snapshot.valid &&
	       encoder_pair_fault(&pair).reason == ENCODER_FAULT_MULTI_EDGE &&
	       encoder_pair_fault(&pair).wheel == 1U;
}

BOOL odometry_test(void)
{
	encoder_t encoder;
	wheel_odometry_config_t config = {2500U, 20.0f};
	wheel_odometry_result_t result;
	float distance = 0.0f;
	BOOL encoder_ok;
	BOOL distance_ok;
	BOOL invalid_ok;
	BOOL pair_ok;
	BOOL pair_invalid_ok;
	BOOL pair_glitch_ok;
	BOOL pair_ambiguous_ok;
	TEST_INIT();

	encoder_init(&encoder, 0U, 0U);
	encoder_update(&encoder, 0U, 1U);
	encoder_update(&encoder, 1U, 1U);
	encoder_update(&encoder, 1U, 0U);
	encoder_update(&encoder, 0U, 0U);
	encoder_ok = encoder_get_count(&encoder) == 4;
	encoder_update(&encoder, 1U, 0U);
	encoder_update(&encoder, 1U, 1U);
	encoder_update(&encoder, 0U, 1U);
	encoder_update(&encoder, 0U, 0U);
	encoder_ok = encoder_ok && encoder_get_count(&encoder) == 0;
	encoder_update(&encoder, 1U, 1U);
	encoder_ok = encoder_ok && encoder_get_count(&encoder) == 0;
	encoder_init(&encoder, 0U, 0U);
	encoder.count = INT32_MAX;
	encoder_update(&encoder, 0U, 1U);
	encoder_ok = encoder_ok && encoder_get_count(&encoder) == INT32_MIN;
	encoder_update(&encoder, 0U, 0U);
	encoder_ok = encoder_ok && encoder_get_count(&encoder) == INT32_MAX;

	distance_ok = wheel_odometry_calculate(&config, 2500, 1.0f, &result) &&
		      result.distance_cm > 19.999f && result.distance_cm < 20.001f &&
		      result.rpm > 59.999f && result.rpm < 60.001f &&
		      result.speed_cm_per_second > 19.999f &&
		      result.speed_cm_per_second < 20.001f;
	distance_ok = distance_ok &&
		      wheel_odometry_distance(&config, -2500, &distance) &&
		      distance > -20.001f && distance < -19.999f;

	config.counts_per_revolution = 0U;
	invalid_ok = !wheel_odometry_distance(&config, 2500, &distance);
	config.counts_per_revolution = 2500U;
	invalid_ok = invalid_ok &&
		     !wheel_odometry_calculate(&config, 2500, 0.0f, &result);
	pair_ok = encoder_pair_test();
	pair_invalid_ok = encoder_pair_invalid_test();
	pair_glitch_ok = encoder_pair_glitch_test();
	pair_ambiguous_ok = encoder_pair_ambiguous_test();

	TEST_CHECK("ODO-U01", encoder_ok, "quadrature forward, reverse and invalid transitions");
	TEST_CHECK("ODO-U02", distance_ok, "calibrated wheel distance and speed");
	TEST_CHECK("ODO-U03", invalid_ok, "invalid calibration and elapsed time rejected");
	TEST_CHECK("ODO-U06", pair_ok, "coherent independent encoder counts");
	TEST_CHECK("ODO-U07", pair_invalid_ok,
		   "ambiguous encoder events beyond the limit, and inconsistent ones, latch invalid");
	TEST_CHECK("ODO-U11", pair_glitch_ok,
		   "encoder glitches the levels resolve (one channel changed) counted, not latched");
	TEST_CHECK("ODO-U12", pair_ambiguous_ok,
		   "both-channel encoder jumps tolerated up to a limit per wheel, earned back by clean travel");
	TEST_SUMMARY("odometry");
	return encoder_ok && distance_ok && invalid_ok && pair_ok && pair_invalid_ok &&
	       pair_glitch_ok && pair_ambiguous_ok;
}
#endif
