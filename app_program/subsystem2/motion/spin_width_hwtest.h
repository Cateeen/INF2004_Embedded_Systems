#ifndef SPIN_WIDTH_HWTEST_H
#define SPIN_WIDTH_HWTEST_H

#include <tk/tkernel.h>

/* Spin-scan obstacle width test (make SENSOR_TEST=spin_width). Body of the
 * hardware sensor test task: never returns. Must run on core 0 before the
 * GPIO dispatcher is started. DRIVES THE MOTORS. */
void spin_width_hwtest_run(void);

#endif
