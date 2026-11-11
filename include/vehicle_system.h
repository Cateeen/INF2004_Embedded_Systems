#ifndef VEHICLE_SYSTEM_H
#define VEHICLE_SYSTEM_H

#include <tk/tkernel.h>

/*
 * System startup boundary for the uT-Kernel vehicle architecture.
 *
 * The implementation owns message-buffer creation, task creation, and task
 * startup ordering. The application entry point must not access subsystem
 * objects directly.
 */
void vehicle_system_run(void);

/*
 * Message-buffer IDs are created and owned by the system-startup module.
 * Tasks may send and receive through these IDs, but must not delete or
 * recreate the buffers.
 */
extern ID g_mbf_line_observations;
extern ID g_mbf_barcode_events;
extern ID g_mbf_imu_estimates;
extern ID g_mbf_obstacle_events;
extern ID g_mbf_obstacle_commands;
extern ID g_mbf_line_odometry;
extern ID g_mbf_imu_odometry;
extern ID g_mbf_motion_commands;
extern ID g_mbf_motion_status;
extern ID g_mbf_remote_commands;
extern ID g_mbf_network_status;
extern ID g_mbf_telemetry;
extern ID g_mbf_faults;

#endif
