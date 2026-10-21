
#include "encoder.h"

#include <stddef.h>

/*
 * Quadrature state:
 * bit 1 = channel A
 * bit 0 = channel B
 *
 * Positive sequence: 00 -> 01 -> 11 -> 10 -> 00
 * Negative sequence: 00 -> 10 -> 11 -> 01 -> 00
 */
static const int8_t transition_table[16] =
{
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static uint8_t make_state(uint8_t a, uint8_t b)
{
    return (uint8_t)(((a != 0U) << 1U) | (b != 0U));
}

void encoder_init(encoder_t *encoder,
                  uint8_t channel_a,
                  uint8_t channel_b)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->count = 0;
    encoder->previous_state = make_state(channel_a, channel_b);
}

void encoder_update(encoder_t *encoder,
                    uint8_t channel_a,
                    uint8_t channel_b)
{
    if (encoder == NULL)
    {
        return;
    }

    uint8_t current_state = make_state(channel_a, channel_b);

    uint8_t index = (uint8_t)(
        (encoder->previous_state << 2U) | current_state
    );

    encoder->count += transition_table[index];
    encoder->previous_state = current_state;
}

int32_t encoder_get_count(const encoder_t *encoder)
{
    if (encoder == NULL)
    {
        return 0;
    }

    return encoder->count;
}

void encoder_reset_count(encoder_t *encoder)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->count = 0;
}
