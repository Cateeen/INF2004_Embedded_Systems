#ifndef ENCODER_PAIR_H
#define ENCODER_PAIR_H

#include "encoder.h"
#include <stdbool.h>

#define ENCODER_LEFT_A_PIN  16U
#define ENCODER_LEFT_B_PIN  17U
#define ENCODER_RIGHT_A_PIN 18U
#define ENCODER_RIGHT_B_PIN 19U
#define ENCODER_EDGE_MASK    0x0000ccccU

typedef struct {
	encoder_t left;
	encoder_t right;
	volatile bool valid;
} encoder_pair_t;

typedef struct {
	int32_t left_count;
	int32_t right_count;
	bool valid;
} encoder_snapshot_t;

/* The IRQ owner initializes the pair before enabling delivery. Only the IRQ
 * updates it; snapshots must exclude the IRQ on the same core. */
void encoder_pair_init(encoder_pair_t *pair, uint32_t pins);
void encoder_pair_update(encoder_pair_t *pair, uint32_t events, uint32_t pins);
encoder_snapshot_t encoder_pair_snapshot(const encoder_pair_t *pair);

#endif
