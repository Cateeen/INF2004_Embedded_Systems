#include "../app.h"
#include "../gpio/gpio_dispatcher.h"
#include "hcsr04.h"
#include "hcsr04_capture.h"
#include "obs_pico_port.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#define OBS_PICO_ECHO_PIN 4U
#define OBS_PICO_TRIGGER_PIN 5U
#define OBS_PICO_SERVO_PIN 15U
#define OBS_PICO_DEFAULT_SOUND_SPEED_MM_S 343000U
#define OBS_PICO_DEFAULT_RISE_TIMEOUT_US 5000U
#define OBS_PICO_DEFAULT_ECHO_TIMEOUT_US 8000U
#define OBS_PICO_DEFAULT_MAX_ECHO_US 5800U
#define OBS_PICO_DEFAULT_STALE_ECHO_TIMEOUT_US 45000U
#define OBS_PICO_DEFAULT_MEASUREMENT_GAP_MS 60U
#define OBS_PICO_DEFAULT_SERVO_MS_PER_DEG 2U
#define OBS_PICO_DEFAULT_SERVO_SETTLE_MS 20U
#define OBS_PICO_DEFAULT_MIN_RANGE_MM 20U
#define OBS_PICO_DEFAULT_MAX_RANGE_MM 1000U
#define OBS_PICO_DEFAULT_ANGLE_AHEAD_DEG 100U
#define OBS_PICO_DEFAULT_SCAN_START_DEG 80U
#define OBS_PICO_DEFAULT_SCAN_STEP_DEG 5U
#define OBS_PICO_DEFAULT_SCAN_COUNT 9U
#define OBS_PICO_DEFAULT_RETRIES 3U
#define OBS_PICO_SERVO_SLICE 7U
/* The RP2040 system clock is 125 MHz; this 8.4 divider gives 1 MHz ticks. */
#define OBS_PICO_SERVO_DIVIDER 125U
#define OBS_PICO_SERVO_WRAP 19999U
#define OBS_PICO_SERVO_DIV_REG \
	(PWM_BASE + PWM_CHx_DIV + (OBS_PICO_SERVO_SLICE * 0x14U))
#define OBS_PICO_EDGE_RISE_BIT 3U
#define OBS_PICO_EDGE_FALL_BIT 2U
#define OBS_PICO_TRIGGER_PULSE_US 10U

typedef struct {
	obs_pico_port_config_t hardware;
	hcsr04_capture_t capture;
	obs_task_mode_t mode;
	uint32_t trigger_ms;
	uint32_t scan_timestamp_ms;
	uint32_t last_trigger_ms;
	uint32_t settle_until_ms;
	uint32_t stale_echo_started_us;
	uint16_t scan_distances[OBS_FINE_MAX_SAMPLES];
	uint8_t scan_index;
	uint8_t current_angle;
	uint8_t attempts;
	volatile uint32_t rise_us;
	volatile uint32_t fall_us;
	volatile uint8_t irq_edge_state;
	volatile bool capture_active;
	bool initialized;
	bool have_angle;
	bool have_trigger;
	bool have_stale_echo_time;
	bool rise_fed_to_capture;
} obs_pico_port_state_t;

static obs_pico_port_state_t s_port;
static bool s_registered;
static bool s_task_started;

static uint32_t timer_us(void)
{
	return in_w(TIMER_TIMELR);
}

static void echo_irq(const uint32_t events[GPIO_IRQ_GROUPS],
		     uint32_t pin_levels, void *context)
{
	uint32_t pin_events;
	uint32_t rise_bit;
	uint32_t fall_bit;
	uint32_t timestamp;
	unsigned shift = (OBS_PICO_ECHO_PIN % 8U) * 4U;

	(void)pin_levels;
	(void)context;
	pin_events = events[OBS_PICO_ECHO_PIN / 8U] >> shift;
	rise_bit = 1U << OBS_PICO_EDGE_RISE_BIT;
	fall_bit = 1U << OBS_PICO_EDGE_FALL_BIT;
	if (!s_port.capture_active) return;
	if ((pin_events & rise_bit) != 0U && (pin_events & fall_bit) != 0U) {
		s_port.irq_edge_state = 3U;
		return;
	}
	timestamp = timer_us();
	if ((pin_events & rise_bit) != 0U) {
		if (s_port.irq_edge_state != 0U) {
			s_port.irq_edge_state = 3U;
			return;
		}
		s_port.rise_us = timestamp;
		s_port.irq_edge_state = 1U;
	} else if ((pin_events & fall_bit) != 0U) {
		if (s_port.irq_edge_state != 1U) {
			s_port.irq_edge_state = 3U;
			return;
		}
		s_port.fall_us = timestamp;
		s_port.irq_edge_state = 2U;
	}
}

obs_pico_port_config_t obs_pico_port_default_config(void)
{
	obs_pico_port_config_t config = {0};

	config.sound_speed_mm_s = OBS_PICO_DEFAULT_SOUND_SPEED_MM_S;
	config.rise_timeout_us = OBS_PICO_DEFAULT_RISE_TIMEOUT_US;
	config.echo_timeout_us = OBS_PICO_DEFAULT_ECHO_TIMEOUT_US;
	config.max_echo_us = OBS_PICO_DEFAULT_MAX_ECHO_US;
	config.stale_echo_timeout_us = OBS_PICO_DEFAULT_STALE_ECHO_TIMEOUT_US;
	config.measurement_gap_ms = OBS_PICO_DEFAULT_MEASUREMENT_GAP_MS;
	config.servo_ms_per_degree = OBS_PICO_DEFAULT_SERVO_MS_PER_DEG;
	config.servo_settle_ms = OBS_PICO_DEFAULT_SERVO_SETTLE_MS;
	config.min_range_mm = OBS_PICO_DEFAULT_MIN_RANGE_MM;
	config.max_range_mm = OBS_PICO_DEFAULT_MAX_RANGE_MM;
	config.angle_ahead_deg = OBS_PICO_DEFAULT_ANGLE_AHEAD_DEG;
	config.scan_start_deg = OBS_PICO_DEFAULT_SCAN_START_DEG;
	config.scan_step_deg = OBS_PICO_DEFAULT_SCAN_STEP_DEG;
	config.scan_count = OBS_PICO_DEFAULT_SCAN_COUNT;
	config.max_no_response_attempts = OBS_PICO_DEFAULT_RETRIES;
	return config;
}

ER obs_pico_port_register(void)
{
	const uint32_t mask[GPIO_IRQ_GROUPS] = {
		(1U << (((OBS_PICO_ECHO_PIN % 8U) * 4U) +
			OBS_PICO_EDGE_FALL_BIT)) |
		(1U << (((OBS_PICO_ECHO_PIN % 8U) * 4U) +
			OBS_PICO_EDGE_RISE_BIT)),
		0U, 0U, 0U
	};
	ER er;

#if TK_SUPPORT_SMP
	if (tk_get_prc() != TP_PRC1) return E_CTX;
#endif
	if (s_registered || gpio_dispatcher_is_started()) return E_OBJ;
	er = gpio_dispatcher_register(mask, echo_irq, NULL);
	if (er >= E_OK) s_registered = true;
	return er;
}

static bool config_valid(const obs_pico_port_config_t *config)
{
	uint16_t last_scan_angle;

	if (config == NULL || !config->echo_voltage_verified_3v3_safe ||
	    !config->servo_limits_verified || config->servo_min_us == 0U ||
	    config->servo_max_us <= config->servo_min_us ||
	    config->servo_max_us >= 20000U ||
	    config->sound_speed_mm_s < 300000U ||
	    config->sound_speed_mm_s > 400000U ||
	    config->rise_timeout_us == 0U ||
	    config->rise_timeout_us > INT32_MAX ||
	    config->echo_timeout_us <= config->rise_timeout_us ||
	    config->echo_timeout_us > INT32_MAX ||
	    config->max_echo_us == 0U ||
	    config->max_echo_us > config->echo_timeout_us ||
	    config->stale_echo_timeout_us == 0U ||
	    config->stale_echo_timeout_us > INT32_MAX ||
	    config->measurement_gap_ms == 0U ||
	    config->measurement_gap_ms > INT32_MAX ||
	    config->servo_ms_per_degree == 0U ||
	    (uint64_t)config->servo_ms_per_degree * 180U +
		    config->servo_settle_ms > INT32_MAX ||
	    config->min_range_mm == 0U ||
	    config->max_range_mm < config->min_range_mm ||
	    config->max_range_mm >= OBS_DIST_NONE ||
	    config->angle_ahead_deg > 180U ||
	    config->scan_start_deg > 180U ||
	    config->scan_step_deg == 0U ||
	    config->scan_count != OBS_FINE_MAX_SAMPLES ||
	    config->max_no_response_attempts == 0U)
		return false;
	last_scan_angle = (uint16_t)config->scan_start_deg +
			  (uint16_t)(config->scan_count - 1U) * config->scan_step_deg;
	return last_scan_angle <= 180U;
}

static ER servo_set(uint8_t angle)
{
	uint32_t mapped = s_port.hardware.servo_reversed ?
			  180U - angle : angle;
	uint32_t pulse = s_port.hardware.servo_min_us +
		(mapped * (s_port.hardware.servo_max_us -
			   s_port.hardware.servo_min_us)) / 180U;

	return pwm_set_cc(OBS_PICO_SERVO_PIN, pulse);
}

static ER initialize(void *context)
{
	unsigned i;
	ER er;

	(void)context;
	if (!s_registered || !gpio_dispatcher_is_started() ||
	    !config_valid(&s_port.hardware))
		return E_PAR;
	if (!hcsr04_capture_init(&s_port.capture,
				 s_port.hardware.echo_timeout_us,
				 s_port.hardware.max_echo_us))
		return E_PAR;

	s_port.initialized = true;
	er = gpio_set_pin(OBS_PICO_TRIGGER_PIN, GPIO_MODE_OUT);
	if (er < E_OK) return er;
	er = gpio_set_val(OBS_PICO_TRIGGER_PIN, 0U);
	if (er < E_OK) return er;
	er = gpio_set_pin(OBS_PICO_ECHO_PIN, GPIO_MODE_IN);
	if (er < E_OK) return er;
	clr_w(GPIO(OBS_PICO_ECHO_PIN), GPIO_PUE);
	set_w(GPIO(OBS_PICO_ECHO_PIN), GPIO_PDE);
	er = pwm_set_pin(OBS_PICO_SERVO_PIN);
	if (er < E_OK) return er;
	out_w(OBS_PICO_SERVO_DIV_REG, OBS_PICO_SERVO_DIVIDER << 4);
	er = pwm_set_wrap(OBS_PICO_SERVO_PIN, OBS_PICO_SERVO_WRAP);
	if (er < E_OK) return er;
	er = pwm_set_cc(OBS_PICO_SERVO_PIN, 0U);
	if (er < E_OK) return er;
	er = pwm_set_enabled(OBS_PICO_SERVO_PIN, TRUE);
	if (er < E_OK) return er;
	for (i = 0U; i < OBS_FINE_MAX_SAMPLES; i++)
		s_port.scan_distances[i] = 0U;
	s_port.mode = OBS_TASK_AHEAD;
	s_port.current_angle = s_port.hardware.angle_ahead_deg;
	s_port.have_angle = false;
	s_port.settle_until_ms = 0U;
	s_port.scan_index = 0U;
	s_port.attempts = 0U;
	s_port.have_trigger = false;
	s_port.have_stale_echo_time = false;
	s_port.capture_active = false;
	return E_OK;
}

static void cancel_capture(void)
{
	UINT saved;

	DI(saved);
	s_port.capture_active = false;
	s_port.irq_edge_state = 0U;
	s_port.rise_us = 0U;
	s_port.fall_us = 0U;
	EI(saved);
	(void)gpio_set_val(OBS_PICO_TRIGGER_PIN, 0U);
	(void)hcsr04_capture_reset(&s_port.capture);
}

static ER stop(void *context)
{
	(void)context;
	if (!s_port.initialized) return E_OK;
	cancel_capture();
	return pwm_set_cc(OBS_PICO_SERVO_PIN, 0U);
}

static uint8_t angle_for_scan_index(uint8_t index)
{
	return (uint8_t)(s_port.hardware.scan_start_deg +
			 index * s_port.hardware.scan_step_deg);
}

static ER set_target_angle(uint8_t angle, uint32_t now_ms)
{
	uint32_t delta;
	ER er;

	if (angle > 180U) return E_PAR;
	if (!s_port.have_angle || s_port.current_angle != angle) {
		delta = s_port.have_angle ?
			(uint32_t)(s_port.current_angle > angle ?
				   s_port.current_angle - angle :
				   angle - s_port.current_angle) : 180U;
		er = servo_set(angle);
		if (er < E_OK) return er;
		s_port.current_angle = angle;
		s_port.have_angle = true;
		s_port.settle_until_ms = now_ms +
			delta * s_port.hardware.servo_ms_per_degree +
			s_port.hardware.servo_settle_ms;
	}
	return (int32_t)(now_ms - s_port.settle_until_ms) >= 0 ? E_OK : E_BUSY;
}

static ER start_capture(uint32_t now_ms)
{
	uint32_t trigger_us;
	UINT saved;

	if (gpio_get_val(OBS_PICO_ECHO_PIN) != 0U) {
		uint32_t now_us = timer_us();
		if (!s_port.have_stale_echo_time) {
			s_port.stale_echo_started_us = now_us;
			s_port.have_stale_echo_time = true;
		}
		return now_us - s_port.stale_echo_started_us >
		       s_port.hardware.stale_echo_timeout_us ? E_IO : E_BUSY;
	}
	s_port.have_stale_echo_time = false;
	if (s_port.have_trigger &&
	    now_ms - s_port.last_trigger_ms < s_port.hardware.measurement_gap_ms)
		return E_BUSY;

	trigger_us = timer_us();
	if (!hcsr04_capture_start(&s_port.capture, trigger_us)) return E_SYS;
	s_port.trigger_ms = now_ms;
	if (s_port.scan_index == 0U && s_port.mode == OBS_TASK_SCAN)
		s_port.scan_timestamp_ms = now_ms;
	s_port.last_trigger_ms = now_ms;
	s_port.have_trigger = true;
	s_port.rise_fed_to_capture = false;
	DI(saved);
	s_port.irq_edge_state = 0U;
	s_port.rise_us = 0U;
	s_port.fall_us = 0U;
	s_port.capture_active = true;
	EI(saved);
	if (gpio_set_val(OBS_PICO_TRIGGER_PIN, 1U) < E_OK) return E_IO;
	while (timer_us() - trigger_us < OBS_PICO_TRIGGER_PULSE_US) {
		/* Preserve the minimum trigger pulse without waiting for Echo. */
	}
	if (gpio_set_val(OBS_PICO_TRIGGER_PIN, 0U) < E_OK) return E_IO;
	return E_OK;
}

static uint8_t read_edges(uint32_t *rise_us, uint32_t *fall_us)
{
	UINT saved;
	uint8_t state;

	DI(saved);
	state = s_port.irq_edge_state;
	*rise_us = s_port.rise_us;
	*fall_us = s_port.fall_us;
	EI(saved);
	return state;
}

static void complete_capture(void)
{
	UINT saved;

	DI(saved);
	s_port.capture_active = false;
	s_port.irq_edge_state = 0U;
	EI(saved);
	(void)hcsr04_capture_reset(&s_port.capture);
}

static ER poll_capture(uint32_t now_ms, uint16_t *distance_mm,
		       bool *measurement_ready)
{
	uint32_t rise_us;
	uint32_t fall_us;
	uint32_t now_us = timer_us();
	uint32_t pulse_us = 0U;
	uint8_t edges = read_edges(&rise_us, &fall_us);
	hcsr04_capture_result_t capture_result;

	*measurement_ready = false;
	if (edges == 3U) return E_IO;
	if (edges == 1U && !s_port.rise_fed_to_capture) {
		capture_result = hcsr04_capture_poll(&s_port.capture, true,
						     rise_us, &pulse_us);
		if (capture_result == HCSR04_CAPTURE_INVALID) return E_IO;
		s_port.rise_fed_to_capture = true;
	}
	if (edges == 2U) {
		if (!s_port.rise_fed_to_capture) {
			capture_result = hcsr04_capture_poll(&s_port.capture, true,
							     rise_us, &pulse_us);
			if (capture_result == HCSR04_CAPTURE_INVALID) return E_IO;
			s_port.rise_fed_to_capture = true;
		}
		capture_result = hcsr04_capture_poll(&s_port.capture, false,
						     fall_us, &pulse_us);
		complete_capture();
		if (capture_result == HCSR04_CAPTURE_INVALID) {
			if (fall_us - rise_us > s_port.hardware.max_echo_us) {
				*distance_mm = OBS_DIST_NONE;
				*measurement_ready = true;
				return E_OK;
			}
			return E_IO;
		}
		if (capture_result == HCSR04_CAPTURE_TIMEOUT) {
			*distance_mm = OBS_DIST_NONE;
			*measurement_ready = true;
			return E_OK;
		}
		if (capture_result != HCSR04_CAPTURE_READY ||
		    !hcsr04_distance_mm(pulse_us,
					s_port.hardware.sound_speed_mm_s,
					distance_mm))
			return E_IO;
		if (*distance_mm > s_port.hardware.max_range_mm)
			*distance_mm = OBS_DIST_NONE;
		else if (*distance_mm < s_port.hardware.min_range_mm)
			*distance_mm = s_port.hardware.min_range_mm;
		*measurement_ready = true;
		s_port.attempts = 0U;
		return E_OK;
	}

	if (edges == 0U &&
	    now_us - s_port.capture.trigger_time_us >=
	    s_port.hardware.rise_timeout_us) {
		complete_capture();
		if (++s_port.attempts >=
		    s_port.hardware.max_no_response_attempts)
			return E_IO;
		return E_OK;
	}

	if (now_us - s_port.capture.trigger_time_us >=
	    s_port.hardware.echo_timeout_us) {
		complete_capture();
		if (!s_port.rise_fed_to_capture) {
			if (++s_port.attempts >=
			    s_port.hardware.max_no_response_attempts)
				return E_IO;
			return E_OK;
		}
		*distance_mm = OBS_DIST_NONE;
		*measurement_ready = true;
		s_port.attempts = 0U;
		return E_OK;
	}
	(void)now_ms;
	return E_BUSY;
}

static ER publish_measurement(uint16_t distance_mm, uint32_t trigger_ms,
			      obs_task_sample_t *sample)
{
	if (s_port.mode == OBS_TASK_AHEAD) {
		sample->ready = true;
		sample->timestamp_ms = trigger_ms;
		sample->distance_mm = distance_mm;
		return E_OK;
	}

	if (s_port.scan_index >= s_port.hardware.scan_count) return E_SYS;
	s_port.scan_distances[s_port.scan_index++] = distance_mm;
	if (s_port.scan_index == s_port.hardware.scan_count) {
		unsigned i;
		sample->ready = true;
		sample->timestamp_ms = s_port.scan_timestamp_ms;
		sample->count = s_port.scan_index;
		sample->angle0_deg = s_port.hardware.scan_start_deg;
		sample->step_deg = s_port.hardware.scan_step_deg;
		for (i = 0U; i < s_port.scan_index; i++)
			sample->distances[i] = s_port.scan_distances[i];
	}
	return E_OK;
}

static ER poll(void *context, obs_task_mode_t mode, obs_task_sample_t *sample)
{
	SYSTIM now;
	uint8_t angle;
	uint16_t distance_mm = OBS_DIST_NONE;
	bool measurement_ready;
	ER er;

	(void)context;
	if (!s_port.initialized || sample == NULL ||
	    (mode != OBS_TASK_AHEAD && mode != OBS_TASK_SCAN))
		return E_OBJ;
	er = tk_get_otm(&now);
	if (er != E_OK) return er;
	if (mode != s_port.mode) {
		cancel_capture();
		s_port.mode = mode;
		s_port.scan_index = 0U;
		s_port.attempts = 0U;
	}
	angle = mode == OBS_TASK_AHEAD ? s_port.hardware.angle_ahead_deg :
		angle_for_scan_index(s_port.scan_index);
	er = set_target_angle(angle, now.lo);
	if (er == E_BUSY) return E_OK;
	if (er != E_OK) return er;

	if (!s_port.capture_active) {
		er = start_capture(now.lo);
		if (er == E_BUSY) return E_OK;
		if (er != E_OK) return er;
		return E_OK;
	}

	er = poll_capture(now.lo, &distance_mm, &measurement_ready);
	if (er == E_BUSY) return E_OK;
	if (er != E_OK) return er;
	if (measurement_ready)
		return publish_measurement(distance_mm, s_port.trigger_ms, sample);
	return E_OK;
}

ER obs_pico_port_task_start(const obs_pico_port_config_t *hardware,
			    const obs_task_config_t *task)
{
	obs_task_config_t configured;

	if (s_task_started) return E_OBJ;
	if (!s_registered || !gpio_dispatcher_is_started()) return E_OBJ;
	if (!config_valid(hardware) || task == NULL) return E_PAR;
	s_port = (obs_pico_port_state_t){0};
	s_port.hardware = *hardware;
	configured = *task;
	configured.poll = poll;
	configured.stop = stop;
	configured.initialize = initialize;
	configured.context = &s_port;
	{
		ER er = obs_task_start(&configured);
		if (er == E_OK) s_task_started = true;
		return er;
	}
}
