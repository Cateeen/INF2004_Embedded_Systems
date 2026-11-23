#ifndef ENCODER_SOURCE_H
#define ENCODER_SOURCE_H

#include <tk/tkernel.h>
#include "encoder_pair.h"

/* Register once from the core-0 startup owner, before the shared dispatcher
 * starts. The wheels must remain stationary through dispatcher startup. */
ER encoder_source_register(void);

/* Call only from a core-0-pinned task after dispatcher startup. A false valid
 * flag is latched until restart; counts are then diagnostic only. */
ER encoder_source_snapshot(encoder_snapshot_t *snapshot);

/* Same rules as encoder_source_snapshot(). Reports why the valid flag first
 * went false (reason ENCODER_FAULT_NONE while still valid). */
ER encoder_source_fault(encoder_fault_t *fault);

#endif
