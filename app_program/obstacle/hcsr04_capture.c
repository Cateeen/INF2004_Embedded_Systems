#include "hcsr04_capture.h"

#include <limits.h>
#include <stddef.h>

bool hcsr04_capture_init(hcsr04_capture_t *capture, uint32_t timeout_us,
			 uint32_t max_echo_us)
{
	if (capture == NULL || capture->initialized || timeout_us == 0U ||
	    timeout_us > INT32_MAX || max_echo_us == 0U ||
	    max_echo_us > timeout_us)
		return false;
	capture->timeout_us = timeout_us;
	capture->max_echo_us = max_echo_us;
	capture->trigger_time_us = 0U;
	capture->rise_time_us = 0U;
	capture->pulse_width_us = 0U;
	capture->state = HCSR04_CAPTURE_IDLE;
	capture->initialized = true;
	return true;
}

bool hcsr04_capture_start(hcsr04_capture_t *capture, uint32_t trigger_time_us)
{
	if (capture == NULL || !capture->initialized ||
	    (capture->state != HCSR04_CAPTURE_IDLE &&
	     capture->state != HCSR04_CAPTURE_COMPLETE))
		return false;
	capture->trigger_time_us = trigger_time_us;
	capture->rise_time_us = 0U;
	capture->pulse_width_us = 0U;
	capture->state = HCSR04_CAPTURE_WAIT_RISE;
	return true;
}

bool hcsr04_capture_reset(hcsr04_capture_t *capture)
{
	if (capture == NULL || !capture->initialized) return false;
	capture->trigger_time_us = 0U;
	capture->rise_time_us = 0U;
	capture->pulse_width_us = 0U;
	capture->state = HCSR04_CAPTURE_IDLE;
	return true;
}

hcsr04_capture_result_t hcsr04_capture_poll(hcsr04_capture_t *capture,
					    bool echo_high,
					    uint32_t now_us,
					    uint32_t *pulse_width_us)
{
	uint32_t elapsed;

	if (capture == NULL || pulse_width_us == NULL || !capture->initialized ||
	    capture->state == HCSR04_CAPTURE_IDLE ||
	    capture->state == HCSR04_CAPTURE_FAULT)
		return HCSR04_CAPTURE_INVALID;
	if (capture->state == HCSR04_CAPTURE_COMPLETE) {
		*pulse_width_us = capture->pulse_width_us;
		return HCSR04_CAPTURE_READY;
	}
	elapsed = now_us - capture->trigger_time_us;
	if (elapsed > INT32_MAX) {
		capture->state = HCSR04_CAPTURE_FAULT;
		return HCSR04_CAPTURE_INVALID;
	}
	if (elapsed > capture->timeout_us) {
		capture->state = HCSR04_CAPTURE_FAULT;
		return HCSR04_CAPTURE_TIMEOUT;
	}
	if (capture->state == HCSR04_CAPTURE_WAIT_RISE) {
		if (echo_high) {
			capture->rise_time_us = now_us;
			capture->state = HCSR04_CAPTURE_WAIT_FALL;
		}
		return HCSR04_CAPTURE_PENDING;
	}
	if (!echo_high) {
		uint32_t width = now_us - capture->rise_time_us;
		if (width == 0U || width > capture->max_echo_us) {
			capture->state = HCSR04_CAPTURE_FAULT;
			return HCSR04_CAPTURE_INVALID;
		}
		capture->pulse_width_us = width;
		capture->state = HCSR04_CAPTURE_COMPLETE;
		*pulse_width_us = width;
		return HCSR04_CAPTURE_READY;
	}
	return HCSR04_CAPTURE_PENDING;
}
