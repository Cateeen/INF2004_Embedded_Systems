#ifndef PID_TUNE_HWTEST_H
#define PID_TUNE_HWTEST_H

#include <tk/tkernel.h>

/* Closed-loop wheel speed PID tuning test (make SENSOR_TEST=pid_tune). Body of
 * the hardware sensor test task: never returns. Must run on core 0 before the
 * GPIO dispatcher is started. DRIVES THE MOTORS. */
void pid_tune_hwtest_run(void);

#endif
