#ifndef VEHICLE_TASKS_H
#define VEHICLE_TASKS_H

#include <tk/tkernel.h>

extern ID g_tsk_line_barcode;
extern ID g_tsk_imu;
extern ID g_tsk_obstacle_detection;
extern ID g_tsk_motion_control;
extern ID g_tsk_odometry;
extern ID g_tsk_vehicle_controller;
extern ID g_tsk_wifi_service;

/*
 * Creates all vehicle tasks, then starts them only after every task has been
 * created successfully. Task bodies are implemented by their owning modules.
 */
ER vehicle_tasks_create_and_start(void);

#endif
