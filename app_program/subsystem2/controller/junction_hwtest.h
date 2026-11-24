#ifndef JUNCTION_HWTEST_H
#define JUNCTION_HWTEST_H

#include <tk/tkernel.h>

/* Junction detection + action test (make SENSOR_TEST=junction): follows the
 * line, detects a tape crossing the path with the left (barcode) sensor and
 * the line sensors (junction_detector), then stops, goes straight, turns
 * left/right or U-turns (JN_ACTION). Body of the hardware sensor test task:
 * never returns. Must run on core 0 before the GPIO dispatcher is started.
 * DRIVES THE MOTORS. */
void junction_hwtest_run(void);

#endif
