#ifndef CAR_SAFETY_H
#define CAR_SAFETY_H

#include "../../vehicle_messages.h"

#define CAR_SAFETY_FAULT_LINE      0x01U
#define CAR_SAFETY_FAULT_IMU       0x02U
#define CAR_SAFETY_FAULT_OBSTACLE  0x04U
#define CAR_SAFETY_FAULT_CLOCK     0x08U
#define CAR_SAFETY_FAULT_EXTERNAL  0x10U
#define CAR_SAFETY_FAULT_EMERGENCY 0x20U

typedef struct {
	uint32_t line_max_age_ms;
	uint32_t imu_max_age_ms;
	uint32_t ahead_max_age_ms;
	uint32_t scan_timeout_ms;
	uint16_t min_line_confidence;
	uint16_t obstacle_stop_mm;
} car_safety_config_t;

typedef struct {
	uint32_t sequence;
	uint32_t timestamp_ms;
	bool have_message;
} car_safety_stream_t;

typedef struct {
	car_safety_config_t config;
	car_safety_stream_t line;
	car_safety_stream_t imu;
	car_safety_stream_t obstacle;
	uint32_t last_clock_ms;
	uint32_t line_wait_ms;
	uint32_t imu_wait_ms;
	uint32_t ahead_wait_ms;
	uint32_t scan_start_ms;
	uint32_t faults;
	bool initialized;
	bool line_clear;
	bool imu_clear;
	bool ahead_clear;
	bool have_ahead;
	bool scanning;
	bool armed;
} car_safety_t;

/* Single controller owner only. Configuration and thresholds must be verified
 * for the robot. Faults latch until reinitialization after system restart. */
bool car_safety_init(car_safety_t *safety, const car_safety_config_t *config,
		     uint32_t now_ms);
bool car_safety_line(car_safety_t *safety, const line_observation_message_t *message,
		     uint32_t now_ms);
bool car_safety_imu(car_safety_t *safety, const imu_estimate_message_t *message,
		    uint32_t now_ms);
bool car_safety_obstacle(car_safety_t *safety, const obstacle_event_message_t *message,
			 uint32_t now_ms);

/* Call only after independently confirming the physical robot stopped, before
 * requesting a scan. Motion must stay stopped until a fresh ahead reading.
 * The helper cannot itself stop or verify the motors. */
bool car_safety_begin_scan(car_safety_t *safety, uint32_t now_ms);

/* Latch a received subsystem fault or an independently detected stop fault. */
void car_safety_latch_fault(car_safety_t *safety);

/* Call periodically even when no messages arrive. A fresh go edge is needed
 * after any stop or unsafe observation. This is only an advisory predicate:
 * it does not send commands, verify a motor stop, or replace a watchdog. */
bool car_safety_follow(car_safety_t *safety, uint32_t now_ms,
		       bool go, bool go_rising, bool emergency_stopped);

/* Re-arm without a new GO press after an automatic manoeuvre the controller
 * ran itself (the obstacle bypass), with exactly the checks a GO press gets:
 * GO still on, no faults, line seen, IMU clear and ahead clear. Returns
 * whether the robot is armed. */
bool car_safety_resume(car_safety_t *safety, uint32_t now_ms,
		       bool go, bool emergency_stopped);

#endif
