#include "encoder_pair.h"

static void update_wheel(encoder_pair_t *pair, encoder_t *encoder, uint8_t wheel,
			 uint32_t events, uint32_t a, uint32_t b)
{
	uint32_t current;
	uint32_t changed;
	uint8_t reason = ENCODER_FAULT_NONE;

	uint32_t diff;
	bool single;

	if (events == 0U) return;
	current = (a << 1U) | b;
	diff = encoder->previous_state ^ current;
	single = (events & (events - 1U)) == 0U;
	changed = (events & 0x0cU) ? 2U : 1U;
	/* Edges on BOTH channels in one IRQ have no order (+-2 is ambiguous), and
	 * a channel that changed without an edge means one was lost: invalid.
	 * Edges on ONE channel only are a glitch the levels resolve exactly: an
	 * unchanged state is net zero (spike, or a +1/-1 jitter), and an odd
	 * number of toggles is one step whose direction the table gets from the
	 * sampled levels. Those are counted as glitches and stay valid. A single
	 * clean edge must still agree with the sampled level. */
	if ((events & 0x0cU) != 0U && (events & 0xc0U) != 0U)
		reason = ENCODER_FAULT_MULTI_EDGE;
	else if ((diff & ~changed) != 0U)
		reason = ENCODER_FAULT_BAD_STEP;
	else if (single && diff == changed &&
		 ((events & 0x88U) != 0U) != ((current & changed) != 0U))
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
