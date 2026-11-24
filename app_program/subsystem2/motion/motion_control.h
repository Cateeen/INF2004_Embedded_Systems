#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "motion_feedback.h"
#include "wheel_speed_control.h"

typedef enum {
	MOTION_CONTROL_WAITING = 0,
	MOTION_CONTROL_READY,
	MOTION_CONTROL_FAULT
} motion_control_result_t;

typedef struct {
	motion_feedback_config_t feedback;
	wheel_speed_config_t speed;
} motion_control_config_t;

typedef struct {
	motion_feedback_t feedback;
	wheel_speed_control_t speed;
	wheel_speed_output_t output;
	uint32_t start_ms;
	uint32_t last_check_ms;
	int8_t direction;       /* +1 forward, -1 reverse, 0 spinning */
	int8_t left_direction;  /* each wheel: +1 forward, -1 backward */
	int8_t right_direction;
	bool have_output;
	bool running;
	bool faulted;
} motion_control_t;

/* Straight motion only: direction is +1 forward or -1 reverse for both
 * polarity-normalized wheels. Start only after the hardware is stopped.
 * The owner must use verified calibration and reset on every restart.
 * Failure clears the control state and latches a fault. */
bool motion_control_start(motion_control_t *control,
			  const motion_control_config_t *config,
			  int8_t direction, uint32_t now_ms);

/* Spin on the spot: turn_direction +1 = left (counter-clockwise: left wheel
 * backward, right forward), -1 = right. Wheel targets are speed magnitudes
 * as for straight motion; each wheel's feedback is checked against its own
 * direction. Same start rules and fault latching as motion_control_start. */
bool motion_control_start_spin(motion_control_t *control,
			       const motion_control_config_t *config,
			       int8_t turn_direction, uint32_t now_ms);

/* Call periodically even without a message. The first message after start
 * establishes a baseline; output remains zero until a second valid message.
 * On any fault output is zero and the fault is latched. READY output may be
 * retained between samples only while the caller continues checking time.
 * A NULL output or invalid control is a fault; the caller must stop hardware.
 * This calculation cannot stop hardware or replace an independent watchdog. */
motion_control_result_t motion_control_step(motion_control_t *control,
			 const wheel_odometry_message_t *message,
			 uint32_t now_ms, wheel_speed_output_t *output);

/* Steering: per-wheel target magnitudes in cm/s, both > 0 and in the started
 * direction (no spins). Only while running and not faulted; PID state is
 * kept. Returns false and changes nothing otherwise. */
bool motion_control_set_targets(motion_control_t *control,
				float left_cm_s, float right_cm_s);

/* Clear the output and PID state; caller must also stop the hardware. */
void motion_control_stop(motion_control_t *control);

#endif
