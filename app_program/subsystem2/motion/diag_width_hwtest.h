#ifndef DIAG_WIDTH_HWTEST_H
#define DIAG_WIDTH_HWTEST_H

#include <tk/tkernel.h>

/* Diagonal-pass obstacle width test (make SENSOR_TEST=diag_width). Body of the
 * hardware sensor test task: never returns. Must run on core 0 before the GPIO
 * dispatcher is started. DRIVES THE MOTORS. */
void diag_width_hwtest_run(void);

#endif
