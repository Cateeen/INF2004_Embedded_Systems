#include "app.h"
#include "vehicle_buffers.h"

#if APP_TEST
#include "test_report.h"
#endif

/*
 * The RP2040 kernel uses an INT-sized header and rounds each message up to
 * sizeof(INT) bytes in its ring buffer (kernel/tkernel/messagebuf.h).
 */
#define BUFFER_WORDS(type, capacity) \
	((capacity) * (1U + ((sizeof(type) + sizeof(INT) - 1U) / sizeof(INT))))
#define DEFINE_BUFFER(name, type, capacity) \
	ID g_mbf_##name; \
	static INT s_##name##_storage[BUFFER_WORDS(type, capacity)]

DEFINE_BUFFER(line_observations, line_observation_message_t, VEHICLE_MB_LINE_CAPACITY);
DEFINE_BUFFER(barcode_events, barcode_event_message_t, VEHICLE_MB_BARCODE_CAPACITY);
DEFINE_BUFFER(imu_estimates, imu_estimate_message_t, VEHICLE_MB_IMU_CAPACITY);
DEFINE_BUFFER(obstacle_events, obstacle_event_message_t, VEHICLE_MB_OBSTACLE_CAPACITY);
DEFINE_BUFFER(obstacle_commands, obstacle_command_message_t, VEHICLE_MB_OBSTACLE_COMMAND_CAPACITY);
DEFINE_BUFFER(line_odometry, wheel_odometry_message_t, VEHICLE_MB_ODOMETRY_CAPACITY);
DEFINE_BUFFER(imu_odometry, wheel_odometry_message_t, VEHICLE_MB_ODOMETRY_CAPACITY);
DEFINE_BUFFER(motion_odometry, wheel_odometry_message_t, VEHICLE_MB_ODOMETRY_CAPACITY);
DEFINE_BUFFER(motion_commands, motion_command_message_t, VEHICLE_MB_MOTION_CAPACITY);
DEFINE_BUFFER(motion_status, motion_status_message_t, VEHICLE_MB_STATUS_CAPACITY);
DEFINE_BUFFER(remote_commands, remote_command_message_t, VEHICLE_MB_REMOTE_CAPACITY);
DEFINE_BUFFER(network_status, network_status_message_t, VEHICLE_MB_NETWORK_CAPACITY);
DEFINE_BUFFER(telemetry, telemetry_message_t, VEHICLE_MB_TELEMETRY_CAPACITY);
DEFINE_BUFFER(faults, fault_message_t, VEHICLE_MB_FAULT_CAPACITY);

typedef struct {
	ID *id;
	INT *storage;
	SZ storage_size;
	INT message_size;
	UINT capacity;
	const char *name;
} vehicle_buffer_definition_t;

#define BUFFER_ENTRY(name, type, capacity) \
	{ &g_mbf_##name, s_##name##_storage, (SZ)sizeof(s_##name##_storage), \
	  (INT)sizeof(type), (capacity), #name }

static const vehicle_buffer_definition_t s_buffers[] = {
	BUFFER_ENTRY(line_observations, line_observation_message_t, VEHICLE_MB_LINE_CAPACITY),
	BUFFER_ENTRY(barcode_events, barcode_event_message_t, VEHICLE_MB_BARCODE_CAPACITY),
	BUFFER_ENTRY(imu_estimates, imu_estimate_message_t, VEHICLE_MB_IMU_CAPACITY),
	BUFFER_ENTRY(obstacle_events, obstacle_event_message_t, VEHICLE_MB_OBSTACLE_CAPACITY),
	BUFFER_ENTRY(obstacle_commands, obstacle_command_message_t, VEHICLE_MB_OBSTACLE_COMMAND_CAPACITY),
	BUFFER_ENTRY(line_odometry, wheel_odometry_message_t, VEHICLE_MB_ODOMETRY_CAPACITY),
	BUFFER_ENTRY(imu_odometry, wheel_odometry_message_t, VEHICLE_MB_ODOMETRY_CAPACITY),
	BUFFER_ENTRY(motion_odometry, wheel_odometry_message_t, VEHICLE_MB_ODOMETRY_CAPACITY),
	BUFFER_ENTRY(motion_commands, motion_command_message_t, VEHICLE_MB_MOTION_CAPACITY),
	BUFFER_ENTRY(motion_status, motion_status_message_t, VEHICLE_MB_STATUS_CAPACITY),
	BUFFER_ENTRY(remote_commands, remote_command_message_t, VEHICLE_MB_REMOTE_CAPACITY),
	BUFFER_ENTRY(network_status, network_status_message_t, VEHICLE_MB_NETWORK_CAPACITY),
	BUFFER_ENTRY(telemetry, telemetry_message_t, VEHICLE_MB_TELEMETRY_CAPACITY),
	BUFFER_ENTRY(faults, fault_message_t, VEHICLE_MB_FAULT_CAPACITY)
};

ER vehicle_buffers_init(void)
{
	UINT i;

	for (i = 0; i < sizeof(s_buffers) / sizeof(s_buffers[0]); ++i) {
		T_CMBF config = {0};
		ID id;

		config.mbfatr = TA_TFIFO | TA_USERBUF;
		config.bufsz = s_buffers[i].storage_size;
		config.maxmsz = s_buffers[i].message_size;
		config.bufptr = s_buffers[i].storage;
		id = tk_cre_mbf(&config);
		if (id < E_OK) {
			ER error = id;
			tm_printf((UB*)"ERROR creating vehicle buffer %s: %d\n",
				  s_buffers[i].name, (int)error);
			while (i > 0) {
				ER cleanup;
				--i;
				cleanup = tk_del_mbf(*s_buffers[i].id);
				if (cleanup < E_OK) {
					tm_printf((UB*)"ERROR deleting vehicle buffer %s: %d\n",
						  s_buffers[i].name, (int)cleanup);
				} else {
					*s_buffers[i].id = 0;
				}
			}
			return error;
		}
		*s_buffers[i].id = id;
	}
	return E_OK;
}

#if APP_TEST
BOOL vehicle_buffers_test(void)
{
	union {
		line_observation_message_t line;
		barcode_event_message_t barcode;
		imu_estimate_message_t imu;
		obstacle_event_message_t obstacle;
		obstacle_command_message_t obstacle_command;
		wheel_odometry_message_t odometry;
		motion_command_message_t motion_command;
		motion_status_message_t motion_status;
		remote_command_message_t remote;
		network_status_message_t network;
		telemetry_message_t telemetry;
		fault_message_t fault;
	} sent, received;
	UINT i;
	BOOL ok = TRUE;
	TEST_INIT();

	for (i = 0; i < sizeof(s_buffers) / sizeof(s_buffers[0]); ++i) {
		UINT j;
		const vehicle_buffer_definition_t *buffer = &s_buffers[i];
		BOOL buffer_ok = TRUE;

		for (j = 0; j < buffer->capacity; ++j) {
			UINT k;
			for (k = 0; k < (UINT)buffer->message_size; ++k) {
				((UB *)&sent)[k] = (UB)(j + 1U);
			}
			if (tk_snd_mbf(*buffer->id, &sent, buffer->message_size,
				       TMO_POL) != E_OK) {
				buffer_ok = FALSE;
				break;
			}
		}
		if (j == buffer->capacity &&
		    tk_snd_mbf(*buffer->id, &sent, buffer->message_size,
			       TMO_POL) != E_TMOUT) {
			buffer_ok = FALSE;
		}
		for (j = 0; j < buffer->capacity; ++j) {
			UINT k;
			if (tk_rcv_mbf(*buffer->id, &received, TMO_POL) !=
			    buffer->message_size) {
				buffer_ok = FALSE;
				break;
			}
			for (k = 0; k < (UINT)buffer->message_size; ++k) {
				if (((UB *)&received)[k] != (UB)(j + 1U)) {
					buffer_ok = FALSE;
					break;
				}
			}
		}
		if (!buffer_ok) {
			tm_printf((UB*)"vehicle buffer test failed: %s\n", buffer->name);
			ok = FALSE;
		}
	}
	TEST_CHECK("INT-03", ok, "vehicle buffers copy messages and hold configured capacity");
	TEST_SUMMARY("vehicle buffers");
	return ok;
}
#endif
