#ifndef ODOMETRY_PRODUCER_H
#define ODOMETRY_PRODUCER_H

#include "wheel_odometry.h"
#include "../vehicle_messages.h"

typedef struct {
	wheel_odometry_config_t left;
	wheel_odometry_config_t right;
	int8_t left_sign;
	int8_t right_sign;
} odometry_producer_config_t;

typedef struct {
	odometry_producer_config_t config;
	uint32_t last_left;
	uint32_t last_right;
	int64_t left_total;
	int64_t right_total;
	uint32_t last_timestamp_ms;
	uint32_t sequence;
	bool initialized;
} odometry_producer_t;

/* Calibration must be supplied by the owning task; signs are +1 or -1.
 * The initial snapshot defines zero distance. Input counts are wrapping
 * encoder values; output counts are polarity-normalized absolute counts.
 * Call only from the single owning task. */
bool odometry_producer_init(odometry_producer_t *producer,
                            const odometry_producer_config_t *config,
                            int32_t left_count, int32_t right_count,
                            uint32_t timestamp_ms);

/* On failure neither state nor message changes. Samples must be ordered:
 * consecutive timestamps and per-wheel count changes must be less than
 * half their respective 32-bit wrap periods. The owner must report failures
 * and consumers must check timestamp freshness and sequence gaps. */
bool odometry_producer_update(odometry_producer_t *producer,
                              int32_t left_count, int32_t right_count,
                              uint32_t timestamp_ms,
                              wheel_odometry_message_t *message);

#endif
