#include "odometry_producer.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

static uint32_t signed_bits(int32_t count, int8_t sign)
{
	uint32_t bits = (uint32_t)count;
	return (sign > 0) ? bits : 0U - bits;
}

static int32_t bits_to_signed(uint32_t bits)
{
	return (bits <= INT32_MAX) ? (int32_t)bits :
	       INT32_MIN + (int32_t)(bits - 0x80000000U);
}

static bool add_delta(int64_t total, int32_t delta, int64_t *result)
{
	if ((delta > 0 && total > INT64_MAX - delta) ||
	    (delta < 0 && total < INT64_MIN - delta)) return false;
	*result = total + delta;
	return true;
}

bool odometry_producer_init(odometry_producer_t *producer,
                            const odometry_producer_config_t *config,
                            int32_t left_count, int32_t right_count,
                            uint32_t timestamp_ms)
{
	double distance;
	odometry_producer_t initial = {0};

	if (producer == NULL || config == NULL ||
	    (config->left_sign != 1 && config->left_sign != -1) ||
	    (config->right_sign != 1 && config->right_sign != -1) ||
	    !wheel_odometry_distance_precise(&config->left, 0, &distance) ||
	    !wheel_odometry_distance_precise(&config->right, 0, &distance)) return false;
	initial.config = *config;
	initial.last_left = signed_bits(left_count, config->left_sign);
	initial.last_right = signed_bits(right_count, config->right_sign);
	initial.last_timestamp_ms = timestamp_ms;
	initial.initialized = true;
	*producer = initial;
	return true;
}

bool odometry_producer_update(odometry_producer_t *producer,
                              int32_t left_count, int32_t right_count,
                              uint32_t timestamp_ms,
                              wheel_odometry_message_t *message)
{
	uint32_t elapsed;
	uint32_t left_bits, right_bits;
	int32_t left_delta, right_delta;
	int64_t left_total, right_total;
	double left_cm, right_cm;
	double distance_mm;
	wheel_odometry_message_t next = {0};

	if (producer == NULL || message == NULL || !producer->initialized)
		return false;
	elapsed = timestamp_ms - producer->last_timestamp_ms;
	if (elapsed == 0U || elapsed > INT32_MAX) return false;
	left_bits = signed_bits(left_count, producer->config.left_sign);
	right_bits = signed_bits(right_count, producer->config.right_sign);
	if (!wheel_odometry_count_delta(left_bits, producer->last_left, &left_delta) ||
	    !wheel_odometry_count_delta(right_bits, producer->last_right, &right_delta) ||
	    !add_delta(producer->left_total, left_delta, &left_total) ||
	    !add_delta(producer->right_total, right_delta, &right_total) ||
	    !wheel_odometry_distance_precise(&producer->config.left, left_total, &left_cm) ||
	    !wheel_odometry_distance_precise(&producer->config.right, right_total, &right_cm))
		return false;

	distance_mm = (left_cm + right_cm) * 5.0;
	if (!isfinite(distance_mm) || distance_mm > INT32_MAX ||
	    distance_mm < INT32_MIN) return false;

	next.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	next.header.version = VEHICLE_MESSAGE_VERSION;
	next.header.sequence = producer->sequence + 1U;
	next.header.timestamp_ms = timestamp_ms;
	next.left_count = bits_to_signed(left_bits);
	next.right_count = bits_to_signed(right_bits);
	next.distance_mm = (int32_t)distance_mm;
	next.valid = 1U;

	producer->last_left = left_bits;
	producer->last_right = right_bits;
	producer->left_total = left_total;
	producer->right_total = right_total;
	producer->last_timestamp_ms = timestamp_ms;
	producer->sequence = next.header.sequence;
	*message = next;
	return true;
}
