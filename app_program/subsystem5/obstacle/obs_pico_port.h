#ifndef OBS_PICO_PORT_H
#define OBS_PICO_PORT_H

#include "obs_task.h"
#include "obs_servo_calibration.h"

typedef struct {
	bool echo_voltage_verified_3v3_safe;
	bool servo_limits_verified;
	obs_servo_calibration_t servo;
	uint32_t sound_speed_mm_s;
	uint32_t rise_timeout_us;
	uint32_t echo_timeout_us;
	uint32_t max_echo_us;
	uint32_t stale_echo_timeout_us;
	uint32_t measurement_gap_ms;
	uint32_t servo_ms_per_degree;
	uint32_t servo_settle_ms;
	uint16_t min_range_mm;
	uint16_t max_range_mm;
	uint8_t scan_start_deg;
	uint8_t scan_step_deg;
	uint8_t scan_count;
	uint8_t max_no_response_attempts;
} obs_pico_port_config_t;

/* Servo angles are estimates; safety confirmations remain unset. */
obs_pico_port_config_t obs_pico_port_default_config(void);

/* Register GP4 with the shared GPIO dispatcher before it is started. */
ER obs_pico_port_register(void);

/* Start the callback-driven obstacle task. Requires verified electrical and
 * servo settings, a registered GP4 route, and a running GPIO dispatcher. */
ER obs_pico_port_task_start(const obs_pico_port_config_t *hardware,
			    const obs_task_config_t *task);

#endif
