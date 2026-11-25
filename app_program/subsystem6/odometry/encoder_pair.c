#include "encoder_pair.h"

static void update_wheel(encoder_pair_t *pair, encoder_t *encoder, uint8_t wheel,
			 uint32_t events, uint32_t a, uint32_t b)
{
	uint32_t current;
	uint32_t edged;
	uint8_t reason = ENCODER_FAULT_NONE;

	uint32_t diff;
	bool single;

	if (events == 0U) return;
	current = (a << 1U) | b;
	diff = encoder->previous_state ^ current;
	single = (events & (events - 1U)) == 0U;
	/* Channels with at least one edge: bit 1 = A, bit 0 = B (as in state). */
	edged = ((events & 0x0cU) ? 2U : 0U) | ((events & 0xc0U) ? 1U : 0U);
	/* BOTH channels changed level in one IRQ: two steps with no order (+-2 is
	 * ambiguous) -- tolerated as 0 steps up to ENCODER_AMBIGUOUS_LIMIT per
	 * wheel (see encoder_pair.h), invalid beyond it. A channel that changed
	 * without an edge means one was lost: invalid. Anything else the levels
	 * resolve exactly: an unchanged
	 * state is net zero (spike, or a +1/-1 jitter), and one changed channel
	 * is one step whose direction the table gets from the sampled levels --
	 * also when the OTHER channel had edges too but ended where it started (a
	 * spike riding on a real edge, seen on the right wheel with the motors
	 * working hard during line following). Those are counted as glitches and
	 * stay valid. A single clean edge must still agree with the sampled level. */
	if (edged == 3U && diff == 3U) {
		if (pair->strikes[wheel - 1U] >= ENCODER_AMBIGUOUS_LIMIT) {
			reason = ENCODER_FAULT_MULTI_EDGE;
		} else {
			/* The table counts a two-channel jump as 0. */
			pair->strikes[wheel - 1U]++;
			pair->fault.ambiguous++;
		}
	} else if ((diff & ~edged) != 0U)
		reason = ENCODER_FAULT_BAD_STEP;
	else if (single && diff == edged &&
		 ((events & 0x88U) != 0U) != ((current & edged) != 0U))
		reason = ENCODER_FAULT_BAD_LEVEL;
	else if (!single || diff == 0U)
		pair->fault.glitches++;
	if (reason != ENCODER_FAULT_NONE) {
		if (pair->fault.reason == ENCODER_FAULT_NONE) {
			pair->fault.wheel = wheel;
			pair->fault.reason = reason;
			pair->fault.events = (uint8_t)events;
			pair->fault.previous_state = encoder->previous_state;
			pair->fault.current_state = (uint8_t)current;
			pair->fault.count = encoder->count;
		}
		pair->valid = false;
	} else if (diff == 1U || diff == 2U) {
		/* A decoded step: clean travel earns back one held strike. */
		if (++pair->clean_steps[wheel - 1U] >= ENCODER_AMBIGUOUS_LEAK_STEPS) {
			pair->clean_steps[wheel - 1U] = 0U;
			if (pair->strikes[wheel - 1U] != 0U) pair->strikes[wheel - 1U]--;
		}
	}
	encoder_update(encoder, (uint8_t)a, (uint8_t)b);
}

void encoder_pair_init(encoder_pair_t *pair, uint32_t pins)
{
	encoder_init(&pair->left, (pins >> ENCODER_LEFT_A_PIN) & 1U,
		     (pins >> ENCODER_LEFT_B_PIN) & 1U);
	encoder_init(&pair->right, (pins >> ENCODER_RIGHT_A_PIN) & 1U,
		     (pins >> ENCODER_RIGHT_B_PIN) & 1U);
	pair->valid = true;
	pair->fault = (encoder_fault_t){0};
	pair->strikes[0] = pair->strikes[1] = 0U;
	pair->clean_steps[0] = pair->clean_steps[1] = 0U;
}

void encoder_pair_update(encoder_pair_t *pair, uint32_t events, uint32_t pins)
{
	update_wheel(pair, &pair->left, 1U, events & 0xccU,
		     (pins >> ENCODER_LEFT_A_PIN) & 1U,
		     (pins >> ENCODER_LEFT_B_PIN) & 1U);
	update_wheel(pair, &pair->right, 2U, (events >> 8U) & 0xccU,
		     (pins >> ENCODER_RIGHT_A_PIN) & 1U,
		     (pins >> ENCODER_RIGHT_B_PIN) & 1U);
}

encoder_fault_t encoder_pair_fault(const encoder_pair_t *pair)
{
	return pair->fault;
}

encoder_snapshot_t encoder_pair_snapshot(const encoder_pair_t *pair)
{
	encoder_snapshot_t snapshot;
	snapshot.left_count = encoder_get_count(&pair->left);
	snapshot.right_count = encoder_get_count(&pair->right);
	snapshot.valid = pair->valid;
	return snapshot;
}
