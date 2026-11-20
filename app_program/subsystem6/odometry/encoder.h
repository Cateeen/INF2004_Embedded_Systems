#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
	volatile int32_t count;
	uint8_t previous_state;
} encoder_t;

void encoder_init(encoder_t *encoder, uint8_t channel_a, uint8_t channel_b);
void encoder_update(encoder_t *encoder, uint8_t channel_a, uint8_t channel_b);
int32_t encoder_get_count(const encoder_t *encoder);
void encoder_reset_count(encoder_t *encoder);

#endif
