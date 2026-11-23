#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "odometry_publish.h"

static odometry_publish_result_t publish(const wheel_odometry_message_t *message,
					 bool include_motion)
{
	odometry_publish_result_t result = {E_PAR, E_PAR, E_PAR, false};

	if (message == NULL || message->header.type != VEHICLE_MESSAGE_WHEEL_ODOMETRY ||
	    message->header.version != VEHICLE_MESSAGE_VERSION || message->valid != 1U)
		return result;
	result.line = tk_snd_mbf(g_mbf_line_odometry, message,
	                         (INT)sizeof(*message), TMO_POL);
	result.imu = tk_snd_mbf(g_mbf_imu_odometry, message,
	                        (INT)sizeof(*message), TMO_POL);
	if (include_motion) {
		result.motion_attempted = true;
		result.motion = tk_snd_mbf(g_mbf_motion_odometry, message,
					   (INT)sizeof(*message), TMO_POL);
	}
	return result;
}

odometry_publish_result_t odometry_publish(const wheel_odometry_message_t *message)
{
	return publish(message, false);
}

odometry_publish_result_t odometry_publish_with_motion(const wheel_odometry_message_t *message)
{
	return publish(message, true);
}
