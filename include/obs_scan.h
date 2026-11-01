#ifndef OBS_SCAN_H
#define OBS_SCAN_H

#include <stdint.h>

/* Initialise the task: hardware, servo to straight ahead, flags cleared, state IDLE. Call once before obs_step(). */
void     obs_init(void);

/* Run one pass of the state machine (one state handler). The task calls this forever; the host test calls it step by step. */
void     obs_step(void);               /* one pass of the state machine (host-testable) */
void     obs_task(void);               /* RTOS task body: for (;;) obs_step();          */
uint8_t  obs_get_state(void);

/* Return how long the last coarse scan took in ms (evidence for NFR3, target 500 ms). */
uint16_t obs_get_last_coarse_ms(void); /* NFR3 evidence: log/publish this               */
uint16_t obs_get_last_scan_ms(void);   /* coarse + fine, for the bypass-time metric      */

#endif
/**
 * Return the current state (obs_state_t) for logging or telemetry.
 */
/**
 * Return how long the last coarse plus fine scan took in ms (part of the bypass-time metric).
 */
/**
 * RTOS task body: obs_init() then obs_step() in an endless loop.
 */