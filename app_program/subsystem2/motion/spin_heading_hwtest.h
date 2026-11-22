#ifndef SPIN_HEADING_HWTEST_H
#define SPIN_HEADING_HWTEST_H

#include <tk/tkernel.h>

/* Spin-in-place heading test (make SENSOR_TEST=spin_heading). Body of the
 * hardware sensor test task: never returns. Must run on core 0 before the
 * GPIO dispatcher is started. DRIVES THE MOTORS. */
void spin_heading_hwtest_run(void);

#endif
