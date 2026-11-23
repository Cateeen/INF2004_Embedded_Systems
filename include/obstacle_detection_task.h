#ifndef OBSTACLE_DETECTION_TASK_H
#define OBSTACLE_DETECTION_TASK_H

#include <stdbool.h>
#include <tk/tkernel.h>

void obstacle_detection_task(INT stacd, void *exinf);

/* Request a scan through the legacy port while that protocol is being migrated. */
bool obstacle_detection_request_scan(void);

#endif
