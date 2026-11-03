/* vehicle_ctrl.h - Vehicle Controller: turns obstacle messages into motor motion. */
#ifndef VEHICLE_CTRL_H
#define VEHICLE_CTRL_H

#include <stdbool.h>

/* Runs forever: init motors, then react to the obstacle task's messages. Run it on vc_core (core 1), or as an RTOS task. */
void vc_task(void);

/*
 * Hook for Buddy 3's line follower: false = "I am taking the motors", true = "line is back, you drive".
 * Weak empty default. With VC_BUILTIN_CRUISE=1 (default) the controller drives straight ahead by itself
 * whenever the line follower would be active, so you can test without Buddy 3's code.
 */
void vc_line_follow_enable(bool on);

#endif