#include "imu_producer.h"

#include <stddef.h>

bool imu_producer_init(imu_producer_t *producer, uint32_t max_odometry_age_ms)
{
	imu_producer_t initial = {0};

	if (producer == NULL || max_odometry_age_ms == 0U ||
	    max_odometry_age_ms > INT32_MAX ||
	    !imu_processor_init(&initial.processor))
		return false;
	initial.max_odometry_age_ms = max_odometry_age_ms;
	initial.initialized = true;
	*producer = initial;
	return true;
}

bool imu_producer_add_odometry(imu_producer_t *producer,
			       const wheel_odometry_message_t *odometry)
{
	uint32_t elapsed;

	if (producer == NULL || !producer->initialized || odometry == NULL ||
	    odometry->header.type != VEHICLE_MESSAGE_WHEEL_ODOMETRY ||
	    odometry->header.version != VEHICLE_MESSAGE_VERSION ||
	    odometry->valid != 1U)
		return false;
	if (producer->have_odometry) {
		elapsed = odometry->header.timestamp_ms -
			  producer->odometry.header.timestamp_ms;
		if (odometry->header.sequence - producer->odometry.header.sequence != 1U ||
		    elapsed == 0U || elapsed > INT32_MAX)
			return false;
	}
	producer->odometry = *odometry;
	producer->have_odometry = true;
	return true;
}

bool imu_producer_update(imu_producer_t *producer, const accel_data_t *accel,
			 const wheel_odometry_message_t *odometry,
			 uint32_t sample_ms, imu_estimate_message_t *message)
{
	imu_producer_t next;
	imu_estimate_message_t estimate = {0};
	uint32_t elapsed;

	if (producer == NULL || accel == NULL || message == NULL ||
	    !producer->initialized || !accel->i2c_ok)
		return false;
	next = *producer;
	if (next.have_sample) {
		elapsed = sample_ms - next.last_sample_ms;
		if (elapsed == 0U || elapsed > INT32_MAX) return false;
	}
	if (odometry != NULL) {
		if (!imu_producer_add_odometry(&next, odometry)) return false;
	}
	if (!next.have_odometry ||
	    sample_ms - next.odometry.header.timestamp_ms > next.max_odometry_age_ms)
		return false;
	if (!imu_processor_update(&next.processor, accel,
				  (float)next.odometry.distance_mm / 1000.0f,
				  sample_ms))
		return false;
	estimate.header.type = VEHICLE_MESSAGE_IMU_ESTIMATE;
	estimate.header.version = VEHICLE_MESSAGE_VERSION;
	estimate.header.sequence = ++next.sequence;
	estimate.header.timestamp_ms = sample_ms;
	estimate.roll_mdeg = (int32_t)(next.processor.current_roll_deg * 1000.0f);
	estimate.pitch_mdeg = (int32_t)(next.processor.current_pitch_deg * 1000.0f);
	estimate.validity = VEHICLE_IMU_VALID_ROLL_PITCH;
	estimate.terrain_telemetry = next.processor.packed_telemetry;
	estimate.shock_detected = next.processor.shock_detected;
	estimate.tilt_detected = next.processor.tilt_detected;
	next.last_sample_ms = sample_ms;
	next.have_sample = true;
	*producer = next;
	*message = estimate;
	return true;
}
