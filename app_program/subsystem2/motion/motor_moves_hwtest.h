#ifndef MOTOR_MOVES_HWTEST_H
#define MOTOR_MOVES_HWTEST_H

#include <tk/tkernel.h>

/* Basic manoeuvre + wheel-balance test (make SENSOR_TEST=motor_moves). Body of
 * the hardware sensor test task: never returns. Must run on core 0 before the
 * GPIO dispatcher is started. DRIVES THE MOTORS. */
void motor_moves_hwtest_run(void);

#endif
