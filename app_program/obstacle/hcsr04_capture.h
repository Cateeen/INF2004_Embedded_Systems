#ifndef HCSR04_CAPTURE_H
#define HCSR04_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	HCSR04_CAPTURE_IDLE = 0,
	HCSR04_CAPTURE_WAIT_RISE,
	HCSR04_CAPTURE_WAIT_FALL,
	HCSR04_CAPTURE_COMPLETE,
	HCSR04_CAPTURE_FAULT
} hcsr04_capture_state_t;

typedef enum {
	HCSR04_CAPTURE_PENDING = 0,
	HCSR04_CAPTURE_READY,
	HCSR04_CAPTURE_TIMEOUT,
	HCSR04_CAPTURE_INVALID
} hcsr04_capture_result_t;

typedef struct {
	uint32_t timeout_us;
	uint32_t max_echo_us;
	uint32_t trigger_time_us;
	uint32_t rise_time_us;
	uint32_t pulse_width_us;
	hcsr04_capture_state_t state;
	bool initialized;
} hcsr04_capture_t;

/* Zero-initialize capture. All times are unsigned microseconds from one
 * monotonic 32-bit clock. Configured intervals must be <= INT32_MAX. */
bool hcsr04_capture_init(hcsr04_capture_t *capture, uint32_t timeout_us,
			 uint32_t max_echo_us);

/* Begin after issuing the sensor trigger. May restart only from IDLE or
 * COMPLETE; a faulted capture requires reinitialization. */
bool hcsr04_capture_start(hcsr04_capture_t *capture, uint32_t trigger_time_us);

/* Cancel any pending capture or reset a completed/faulted one to IDLE. */
bool hcsr04_capture_reset(hcsr04_capture_t *capture);

/* Feed each observed Echo edge with its monotonic timestamp; call periodically
 * with the current low level too, so missing edges time out. Sampling this
 * slowly can miss short pulses: a hardware integration should capture both
 * edges (for example via GPIO interrupts). READY publishes the measured high
 * pulse. Timeout/error latches FAULT and leaves pulse_width_us unchanged. */
hcsr04_capture_result_t hcsr04_capture_poll(hcsr04_capture_t *capture,
					    bool echo_high,
					    uint32_t now_us,
					    uint32_t *pulse_width_us);

#endif
