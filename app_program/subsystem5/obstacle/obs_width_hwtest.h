#ifndef OBS_WIDTH_HWTEST_H
#define OBS_WIDTH_HWTEST_H

#include <tk/tkernel.h>

/* Live obstacle-width diagnostic (make SENSOR_TEST=obstacle_width).
 * Body of the hardware sensor test task: never returns. Must run on core 0
 * before the GPIO dispatcher is started, with vehicle buffers created. */
void obs_width_hwtest_run(void);

#endif
