#ifndef LINE_FOLLOW_HWTEST_H
#define LINE_FOLLOW_HWTEST_H

#include <tk/tkernel.h>

/* Line-following test (make SENSOR_TEST=line_follow): drives the real
 * line_steering component on live IR readings through the vehicle's wheel
 * PID, and reports wiggle and curve-tracking metrics. Body of the hardware
 * sensor test task: never returns. Must run on core 0 before the GPIO
 * dispatcher is started. DRIVES THE MOTORS. */
void line_follow_hwtest_run(void);

#endif
