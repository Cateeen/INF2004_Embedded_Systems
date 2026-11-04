#ifndef LINE_STATUS_H
#define LINE_STATUS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Shared line-presence information.
 *
 * Initialize before launching the second core.
 * Core 1 publishes Buddy 3 sensor observations.
 * Either core can read the latest observation.
 */

void line_status_init(void);

void line_status_publish(bool present, uint32_t now_ms);

bool line_status_is_present(uint32_t now_ms);

#endif