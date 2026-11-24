/*
 * vehicle_bypass.h - runs obstacle_bypass_run() for the vehicle controller in
 * its own task (pinned to core 0: it reads the encoders directly). The motion
 * task owns the motor driver; each run borrows it with
 * motion_task_borrow_motors() (lent only while the motion task is stopped)
 * and gives it back before reporting DONE, so exactly one task ever drives
 * the motors. Progress (profile, passing, rejoined) goes to the dashboard.
 *
 * The controller owns everything else: it forwards each ahead reading
 * (vehicle_bypass_feed), starts a run only after it has sent STOP and the
 * motion task reports idle (vehicle_bypass_request), keeps every safety
 * check running, aborts on a fault or GO (vehicle_bypass_abort), and
 * re-arms line following when the run reports OK.
 */
#ifndef VEHICLE_BYPASS_H
#define VEHICLE_BYPASS_H

#include <tk/tkernel.h>
#include <stdbool.h>
#include <stdint.h>
#include "../motion/obstacle_bypass.h"

typedef enum {
	VEHICLE_BYPASS_IDLE = 0,
	VEHICLE_BYPASS_RUNNING,
	VEHICLE_BYPASS_DONE
} vehicle_bypass_state_t;

/* From the core-0 startup task, after encoder_source_register() and
 * gpio_dispatcher_start(). The config is copied. */
ER vehicle_bypass_start(const obstacle_bypass_config_t *config);

/* Controller task: the newest ahead reading (any core). */
void vehicle_bypass_feed(uint16_t distance_mm, uint32_t timestamp_ms);

/* Start a run; false if not started or not IDLE. */
bool vehicle_bypass_request(void);

/* Stop a run as soon as possible (motors braked). */
void vehicle_bypass_abort(void);

/* State; *result is set when DONE. */
vehicle_bypass_state_t vehicle_bypass_state(obstacle_bypass_status_t *result);

/* DONE -> IDLE. */
void vehicle_bypass_ack(void);

#endif
