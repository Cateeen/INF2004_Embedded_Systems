#ifndef IMU_PRODUCER_H
#define IMU_PRODUCER_H

#include "imu_processor.h"
#include "../vehicle_messages.h"

typedef struct {
	imu_state_t processor;
	wheel_odometry_message_t odometry;
	uint32_t max_odometry_age_ms;
	uint32_t last_sample_ms;
	uint32_t sequence;
	bool initialized;
	bool have_odometry;
	bool have_sample;
} imu_producer_t;

/* The producer is owned by one task; max_odometry_age_ms must be 1..INT32_MAX. */
bool imu_producer_init(imu_producer_t *producer, uint32_t max_odometry_age_ms);

/* Accept one ordered odometry message without producing an estimate.
 * The owner checks its age and retains future messages until their timestamp.
 * On failure the producer is unchanged. */
bool imu_producer_add_odometry(imu_producer_t *producer,
			       const wheel_odometry_message_t *odometry);

/* Pass a new odometry message only when one is available, otherwise NULL.
 * The owner retains future-dated messages until their timestamp. The first
 * sample requires odometry; subsequent samples may reuse it while fresh.
 * On failure neither the producer nor output changes. The owner must report
 * failures and independently validate the sensor sample's age. */
bool imu_producer_update(imu_producer_t *producer, const accel_data_t *accel,
			 const wheel_odometry_message_t *odometry,
			 uint32_t sample_ms, imu_estimate_message_t *message);

#endif
