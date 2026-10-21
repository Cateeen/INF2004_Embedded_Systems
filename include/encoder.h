
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct
{
    int32_t count;
    uint8_t previous_state;
} encoder_t;

/* Initialise using the current A and B input levels. */
void encoder_init(encoder_t *encoder,
                  uint8_t channel_a,
                  uint8_t channel_b);

/* Process a new pair of quadrature input levels. */
void encoder_update(encoder_t *encoder,
                    uint8_t channel_a,
                    uint8_t channel_b);

/* Read the accumulated quadrature transition count. */
int32_t encoder_get_count(const encoder_t *encoder);

/* Reset the count while preserving quadrature state. */
void encoder_reset_count(encoder_t *encoder);

#endif
