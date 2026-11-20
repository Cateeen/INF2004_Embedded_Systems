#ifndef LINE_OBSERVATION_H
#define LINE_OBSERVATION_H

#include <stdbool.h>
#include <stdint.h>

#define POS_JUNCTION 0U
#define POS_LEFT     1U
#define POS_RIGHT    2U
#define POS_LOST     3U

typedef struct {
	uint16_t raw;
	bool black;
} ir_reading_t;

typedef struct {
	ir_reading_t barcode;
	ir_reading_t center;
	ir_reading_t right;
} ir_readings_t;

uint8_t line_following_position(const ir_readings_t *readings);

#endif
