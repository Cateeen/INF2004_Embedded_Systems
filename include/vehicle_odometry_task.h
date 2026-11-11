#ifndef VEHICLE_ODOMETRY_TASK_H
#define VEHICLE_ODOMETRY_TASK_H

#include <stdint.h>
#include <tk/tkernel.h>

void vehicle_odometry_task(INT stacd, void *exinf);

/*
 * These hooks are called by the board's shared GPIO dispatcher. They do not
 * access message buffers and are safe to call from the encoder edge path.
 */
void vehicle_odometry_left_edge(uint8_t channel_a, uint8_t channel_b);
void vehicle_odometry_right_edge(uint8_t channel_a, uint8_t channel_b);

#endif
