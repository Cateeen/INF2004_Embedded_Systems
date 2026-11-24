#ifndef ENCODER_PAIR_H
#define ENCODER_PAIR_H

#include "encoder.h"
#include <stdbool.h>

#define ENCODER_LEFT_A_PIN  16U
#define ENCODER_LEFT_B_PIN  17U
#define ENCODER_RIGHT_A_PIN 18U
#define ENCODER_RIGHT_B_PIN 19U
#define ENCODER_EDGE_MASK    0x0000ccccU

/* Both channels of one wheel jumping together (00 <-> 11 or 01 <-> 10 in one
 * IRQ) cannot be decoded: +2, -2 and "nothing" look the same. A real encoder
 * never does it at our speeds; a supply/ground spike on the encoder does
 * (seen on the right wheel when the motor current changes sharply). Each
 * wheel absorbs up to ENCODER_AMBIGUOUS_LIMIT of them as 0 steps (at most
 * 2 counts = 0.16 mm lost each); every ENCODER_AMBIGUOUS_LEAK_STEPS clean
 * steps on that wheel earn one back (1000 steps ~ 82 mm). One more beyond the
 * limit latches ENCODER_FAULT_MULTI_EDGE, so a failing encoder still stops
 * the robot within a few centimetres. */
#define ENCODER_AMBIGUOUS_LIMIT       5U
#define ENCODER_AMBIGUOUS_LEAK_STEPS  1000U

/* Why the pair first went invalid (diagnostics only; the first fault wins).
 * Edges on one channel only that the sampled levels resolve exactly (a short
 * spike or jitter) are not faults; they only increment `glitches`. Both-channel
 * jumps within the limit above only increment `ambiguous`. */
typedef enum {
	ENCODER_FAULT_NONE = 0,
	ENCODER_FAULT_MULTI_EDGE,   /* both channels jumped together, over the limit */
	ENCODER_FAULT_BAD_STEP,     /* a channel changed without an edge on it */
	ENCODER_FAULT_BAD_LEVEL     /* edge direction disagrees with sampled level */
} encoder_fault_reason_t;

typedef struct {
	uint8_t wheel;              /* 0 none, 1 left, 2 right */
	uint8_t reason;             /* encoder_fault_reason_t */
	uint8_t events;             /* that wheel's edge bits: 0x04 A fall, 0x08 A rise,
	                             * 0x40 B fall, 0x80 B rise */
	uint8_t previous_state;     /* (A << 1) | B before the IRQ */
	uint8_t current_state;      /* (A << 1) | B sampled in the IRQ */
	int32_t count;              /* that wheel's count just before the fault */
	uint32_t glitches;          /* tolerated one-channel glitches, both wheels */
	uint32_t ambiguous;         /* tolerated both-channel jumps, both wheels */
} encoder_fault_t;

typedef struct {
	encoder_t left;
	encoder_t right;
	volatile bool valid;
	encoder_fault_t fault;
	uint8_t strikes[2];         /* [0] left, [1] right: both-channel jumps held */
	uint16_t clean_steps[2];    /* clean steps since the last strike was earned back */
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
encoder_fault_t encoder_pair_fault(const encoder_pair_t *pair);

#endif
