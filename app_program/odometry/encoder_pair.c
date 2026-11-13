#include "encoder_pair.h"

static void update_wheel(encoder_t *encoder, uint32_t events, uint32_t a,
			 uint32_t b, volatile bool *valid)
{
	uint32_t current;
	uint32_t changed;

	if (events == 0U) return;
	current = (a << 1U) | b;
	changed = (events & 0x0cU) ? 2U : 1U;
	/* Multiple pending edges have no order; a single edge must match the
	 * sampled direction and exactly one changed quadrature channel. */
	if ((events & (events - 1U)) != 0U ||
	    (encoder->previous_state ^ current) != changed ||
	    ((events & 0x88U) != 0U) != ((current & changed) != 0U))
		*valid = false;
	encoder_update(encoder, (uint8_t)a, (uint8_t)b);
}

void encoder_pair_init(encoder_pair_t *pair, uint32_t pins)
{
	encoder_init(&pair->left, (pins >> ENCODER_LEFT_A_PIN) & 1U,
		     (pins >> ENCODER_LEFT_B_PIN) & 1U);
	encoder_init(&pair->right, (pins >> ENCODER_RIGHT_A_PIN) & 1U,
		     (pins >> ENCODER_RIGHT_B_PIN) & 1U);
	pair->valid = true;
}

void encoder_pair_update(encoder_pair_t *pair, uint32_t events, uint32_t pins)
{
	update_wheel(&pair->left, events & 0xccU,
		     (pins >> ENCODER_LEFT_A_PIN) & 1U,
		     (pins >> ENCODER_LEFT_B_PIN) & 1U, &pair->valid);
	update_wheel(&pair->right, (events >> 8U) & 0xccU,
		     (pins >> ENCODER_RIGHT_A_PIN) & 1U,
		     (pins >> ENCODER_RIGHT_B_PIN) & 1U, &pair->valid);
}

encoder_snapshot_t encoder_pair_snapshot(const encoder_pair_t *pair)
{
	encoder_snapshot_t snapshot;
	snapshot.left_count = encoder_get_count(&pair->left);
	snapshot.right_count = encoder_get_count(&pair->right);
	snapshot.valid = pair->valid;
	return snapshot;
}
