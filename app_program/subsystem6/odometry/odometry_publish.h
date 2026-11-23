#ifndef ODOMETRY_PUBLISH_H
#define ODOMETRY_PUBLISH_H

#include <tk/tkernel.h>
#include "../../vehicle_messages.h"

typedef struct {
	ER line;
	ER imu;
	ER motion;
	bool motion_attempted;
} odometry_publish_result_t;

/* Line and IMU sends are attempted independently. Motion is not sent by this
 * function; motion_attempted is false and motion is not a delivery result.
 * The caller must handle
 * each result, including a full buffer; this function never discards messages. */
odometry_publish_result_t odometry_publish(const wheel_odometry_message_t *message);

/* Also attempts the dedicated motion buffer for a valid message. All sends
 * are non-blocking and independent, so partial delivery is possible. */
odometry_publish_result_t odometry_publish_with_motion(const wheel_odometry_message_t *message);

#endif
