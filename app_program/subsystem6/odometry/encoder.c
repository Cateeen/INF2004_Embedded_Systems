#include "encoder.h"

#include <stddef.h>

/* Positive sequence: 00 -> 01 -> 11 -> 10 -> 00. */
static const int8_t transition_table[16] = {
	 0,  1, -1,  0,
	-1,  0,  0,  1,
	 1,  0,  0, -1,
	 0, -1,  1,  0
};

static uint8_t make_state(uint8_t a, uint8_t b)
{
	return (uint8_t)(((a != 0U) << 1U) | (b != 0U));
}

void encoder_init(encoder_t *encoder, uint8_t channel_a, uint8_t channel_b)
{
	if (encoder == NULL) {
		return;
	}
	encoder->count = 0;
	encoder->previous_state = make_state(channel_a, channel_b);
}

void encoder_update(encoder_t *encoder, uint8_t channel_a, uint8_t channel_b)
{
	uint8_t current_state;
	uint8_t index;
	uint32_t count_bits;

	if (encoder == NULL) {
		return;
	}
	current_state = make_state(channel_a, channel_b);
	index = (uint8_t)((encoder->previous_state << 2U) | current_state);
	count_bits = (uint32_t)encoder->count +
		     (uint32_t)(int32_t)transition_table[index];
	encoder->count = (count_bits <= INT32_MAX) ? (int32_t)count_bits :
			 INT32_MIN + (int32_t)(count_bits - 0x80000000U);
	encoder->previous_state = current_state;
}

int32_t encoder_get_count(const encoder_t *encoder)
{
	return (encoder == NULL) ? 0 : encoder->count;
}

void encoder_reset_count(encoder_t *encoder)
{
	if (encoder != NULL) {
		encoder->count = 0;
	}
}
