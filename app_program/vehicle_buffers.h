#ifndef VEHICLE_BUFFERS_H
#define VEHICLE_BUFFERS_H

#include <tk/tkernel.h>

extern ID g_mbf_line_observations;
extern ID g_mbf_barcode_events;
extern ID g_mbf_imu_estimates;
extern ID g_mbf_obstacle_events;
extern ID g_mbf_obstacle_commands;
extern ID g_mbf_line_odometry;
extern ID g_mbf_imu_odometry;
extern ID g_mbf_motion_odometry;
extern ID g_mbf_motion_commands;
extern ID g_mbf_motion_status;
extern ID g_mbf_remote_commands;
extern ID g_mbf_network_status;
extern ID g_mbf_telemetry;
extern ID g_mbf_faults;

ER vehicle_buffers_init(void);

#if APP_TEST
BOOL vehicle_buffers_test(void);
#endif

#endif
