#include "../../app.h"
#include "motor_driver.h"

#include <tk/syslib.h>
#include <sys/sysdef.h>

#define MOTOR_1_A_PIN 8U
#define MOTOR_1_B_PIN 9U
#define MOTOR_2_A_PIN 10U
#define MOTOR_2_B_PIN 11U
#define MOTOR_PWM_FREQUENCY_HZ 20000U
#define MOTOR_REVERSE_BRAKE_US 50000U
#define MOTOR_PWM_DIVIDER 1U
#define MOTOR_PWM_DIV_REG(slice) \
	(PWM_BASE + PWM_CHx_DIV + ((slice) * 0x14U))
#define MOTOR_PWM_CC_REG(slice) \
	(PWM_BASE + PWM_CHx_CC + ((slice) * 0x14U))
#define MOTOR_PWM_SLICE(pin) (((pin) >> 1) & 0x07U)
#define MOTOR_CLOCK_HZ (TMCLK_MHz * 1000000U)

typedef struct {
	int8_t last_direction;
	bool braking;
	uint32_t brake_started_us;
} motor_state_t;

static const UINT s_motor_pins[4] = {
	MOTOR_1_A_PIN, MOTOR_1_B_PIN, MOTOR_2_A_PIN, MOTOR_2_B_PIN
};
static motor_state_t s_motor_states[2];
static ID s_owner_tid;
static uint16_t s_pwm_period_ticks;
static bool s_initialized;
static bool s_emergency_stopped = true;	/* __atomic: the borrower reads it */
/* Lending (motor_driver_lend): the task allowed to drive while lent, 0 when
 * not lent. Written by the owner (lend) and the borrower (return, revoke),
 * always with release/acquire so the motor state passes between them. */
static ID s_borrower_tid;
static bool s_revoke;			/* owner asked for the motors back */
/* Signed output last applied to each motor (0 = braked), for the motion
 * status; written by the current writer, read by the owner (__atomic). */
static int16_t s_output[2];

static uint32_t timer_us(void)
{
	return in_w(TIMER_TIMELR);
}

static void brake(motor_id_t motor);

/*
 * Exactly one task writes the motors at a time: the owner, or the borrower
 * while they are lent. E_OK: the caller is that writer. E_BUSY: the caller
 * is the owner but the motors are lent. E_CTX: any other task. A borrower
 * whose motors were revoked brakes them here, gives them back and gets E_CTX.
 */
static ER check_writer(void)
{
	ID tid = tk_get_tid();
	ID borrower;

	if (s_owner_tid <= 0) return E_OBJ;
	borrower = __atomic_load_n(&s_borrower_tid, __ATOMIC_ACQUIRE);
	if (borrower <= 0) return tid == s_owner_tid ? E_OK : E_CTX;
	if (tid == s_owner_tid) return E_BUSY;
	if (tid != borrower) return E_CTX;
	if (__atomic_load_n(&s_revoke, __ATOMIC_ACQUIRE)) {
		brake(MOTOR_1);
		brake(MOTOR_2);
		__atomic_store_n(&s_revoke, false, __ATOMIC_RELAXED);
		__atomic_store_n(&s_borrower_tid, 0, __ATOMIC_RELEASE);
		return E_CTX;
	}
	return E_OK;
}

/* The owner wants the motors stopped while they are lent: it must not touch
 * them, so the borrower brakes at its next driver call (its loops call the
 * driver every ~10 ms) and the motors return to the owner. */
static void request_revoke(void)
{
	__atomic_store_n(&s_revoke, true, __ATOMIC_RELEASE);
}

static bool valid_motor(motor_id_t motor)
{
	return motor == MOTOR_1 || motor == MOTOR_2;
}

static void write_pair(motor_id_t motor, uint16_t duty_a, uint16_t duty_b)
{
	UINT pin_a = motor == MOTOR_1 ? MOTOR_1_A_PIN : MOTOR_2_A_PIN;
	UINT slice = MOTOR_PWM_SLICE(pin_a);
	out_w(MOTOR_PWM_CC_REG(slice),
	      (UW)duty_a | ((UW)duty_b << 16));
}

static void brake(motor_id_t motor)
{
	motor_state_t *state = &s_motor_states[(UINT)motor];

	write_pair(motor, 0U, 0U);
	__atomic_store_n(&s_output[(UINT)motor], (int16_t)0, __ATOMIC_RELAXED);
	if (!state->braking) {
		state->braking = true;
		state->brake_started_us = timer_us();
	}
}

static ER safe_gpio_low(void)
{
	UINT i;
	ER result = E_OK;

	for (i = 0U; i < 4U; i++) {
		ER er = gpio_set_pin(s_motor_pins[i], GPIO_MODE_OUT);
		if (er < E_OK && result == E_OK) result = er;
		er = gpio_set_val(s_motor_pins[i], 0U);
		if (er < E_OK && result == E_OK) result = er;
	}
	return result;
}

static ER stop_pwm_slices(void)
{
	ER first = E_OK;
	ER er;

	er = pwm_set_enabled(MOTOR_1_A_PIN, FALSE);
	if (er < E_OK) first = er;
	er = pwm_set_enabled(MOTOR_2_A_PIN, FALSE);
	if (er < E_OK && first == E_OK) first = er;
	return first;
}

static ER initialization_failure(ER cause)
{
	ER cleanup;

	write_pair(MOTOR_1, 0U, 0U);
	write_pair(MOTOR_2, 0U, 0U);
	cleanup = stop_pwm_slices();
	if (cleanup < E_OK)
		tm_printf((UB*)"motor PWM cleanup failed: %d\n", (int)cleanup);
	cleanup = safe_gpio_low();
	if (cleanup < E_OK)
		tm_printf((UB*)"motor GPIO cleanup failed: %d\n", (int)cleanup);
	s_initialized = false;
	s_emergency_stopped = true;
	return cause;
}

ER motor_driver_init(void)
{
	ID tid = tk_get_tid();
	uint32_t period_ticks;
	UINT i;
	ER er;

	if (tid <= 0) return E_CTX;
	if (s_owner_tid > 0 && s_owner_tid != tid) return E_OBJ;
	if (s_initialized) return E_OBJ;
	s_owner_tid = tid;
	s_emergency_stopped = true;
	s_pwm_period_ticks = 0U;

	er = safe_gpio_low();
	if (er < E_OK) return initialization_failure(er);
	er = stop_pwm_slices();
	if (er < E_OK) return initialization_failure(er);

	period_ticks = (MOTOR_CLOCK_HZ + MOTOR_PWM_FREQUENCY_HZ - 1U) /
		       MOTOR_PWM_FREQUENCY_HZ;
	if (period_ticks < 2U || period_ticks > UINT16_MAX)
		return initialization_failure(E_SYS);
	s_pwm_period_ticks = (uint16_t)period_ticks;

	out_w(MOTOR_PWM_DIV_REG(MOTOR_PWM_SLICE(MOTOR_1_A_PIN)),
	      MOTOR_PWM_DIVIDER << 4);
	out_w(MOTOR_PWM_DIV_REG(MOTOR_PWM_SLICE(MOTOR_2_A_PIN)),
	      MOTOR_PWM_DIVIDER << 4);
	er = pwm_set_wrap(MOTOR_1_A_PIN, (UW)(s_pwm_period_ticks - 1U));
	if (er < E_OK) return initialization_failure(er);
	er = pwm_set_wrap(MOTOR_2_A_PIN, (UW)(s_pwm_period_ticks - 1U));
	if (er < E_OK) return initialization_failure(er);
	write_pair(MOTOR_1, 0U, 0U);
	write_pair(MOTOR_2, 0U, 0U);

	for (i = 0U; i < 4U; i++) {
		er = pwm_set_pin(s_motor_pins[i]);
		if (er < E_OK) return initialization_failure(er);
	}
	er = pwm_set_enabled(MOTOR_1_A_PIN, TRUE);
	if (er < E_OK) return initialization_failure(er);
	er = pwm_set_enabled(MOTOR_2_A_PIN, TRUE);
	if (er < E_OK) return initialization_failure(er);

	for (i = 0U; i < 2U; i++) {
		s_motor_states[i].last_direction = 0;
		s_motor_states[i].braking = true;
		s_motor_states[i].brake_started_us = timer_us();
	}
	s_initialized = true;
	return E_OK;
}

ER motor_set_speed(motor_id_t motor, int16_t speed_percent)
{
	motor_state_t *state;
	int8_t direction;
	uint16_t magnitude;
	uint16_t duty;
	ER er = check_writer();

	if (er == E_BUSY) return E_CTX;	/* owner while lent: not its turn */
	if (er < E_OK) return er;
	if (!s_initialized || __atomic_load_n(&s_emergency_stopped, __ATOMIC_ACQUIRE))
		return E_OBJ;
	if (!valid_motor(motor)) return E_PAR;
	if (speed_percent > 100) speed_percent = 100;
	else if (speed_percent < -100) speed_percent = -100;
	if (speed_percent == 0) {
		brake(motor);
		return E_OK;
	}

	state = &s_motor_states[(UINT)motor];
	direction = speed_percent > 0 ? 1 : -1;
	if (state->last_direction != 0 &&
	    direction != state->last_direction) {
		if (!state->braking) {
			brake(motor);
			return E_BUSY;
		}
		if (timer_us() - state->brake_started_us <
		    MOTOR_REVERSE_BRAKE_US)
			return E_BUSY;
	}

	magnitude = (uint16_t)(speed_percent < 0 ?
			      -speed_percent : speed_percent);
	duty = (uint16_t)(((uint32_t)magnitude * s_pwm_period_ticks) / 100U);
	if (direction > 0) write_pair(motor, duty, 0U);
	else write_pair(motor, 0U, duty);
	__atomic_store_n(&s_output[(UINT)motor], speed_percent, __ATOMIC_RELAXED);
	state->last_direction = direction;
	state->braking = false;
	return E_OK;
}

ER motor_brake_all(void)
{
	ER er = check_writer();

	if (er == E_BUSY) {
		request_revoke();
		return E_OK;
	}
	if (er < E_OK) return er;
	if (!s_initialized) return E_OBJ;
	brake(MOTOR_1);
	brake(MOTOR_2);
	return E_OK;
}

ER motor_emergency_stop(void)
{
	ER er = check_writer();

	if (er == E_BUSY) {
		/* The borrower's next motor_set_speed is refused and brakes. */
		__atomic_store_n(&s_emergency_stopped, true, __ATOMIC_RELEASE);
		request_revoke();
		return E_OK;
	}
	if (er < E_OK) return er;
	__atomic_store_n(&s_emergency_stopped, true, __ATOMIC_RELEASE);
	if (!s_initialized) return E_OBJ;
	brake(MOTOR_1);
	brake(MOTOR_2);
	return E_OK;
}

bool motor_is_emergency_stopped(void)
{
	return __atomic_load_n(&s_emergency_stopped, __ATOMIC_ACQUIRE);
}

ER motor_clear_emergency_stop(void)
{
	ER er = check_writer();

	if (er == E_BUSY) return E_OBJ;	/* lent: get them back first */
	if (er < E_OK) return er;
	if (!s_initialized) return E_OBJ;
	brake(MOTOR_1);
	brake(MOTOR_2);
	__atomic_store_n(&s_emergency_stopped, false, __ATOMIC_RELEASE);
	return E_OK;
}

ER motor_driver_lend(ID borrower)
{
	ER er = check_writer();

	if (er == E_BUSY) return E_OBJ;	/* already lent */
	if (er < E_OK) return er;
	if (!s_initialized || __atomic_load_n(&s_emergency_stopped, __ATOMIC_ACQUIRE))
		return E_OBJ;
	if (borrower <= 0 || borrower == s_owner_tid) return E_PAR;
	brake(MOTOR_1);
	brake(MOTOR_2);
	__atomic_store_n(&s_revoke, false, __ATOMIC_RELAXED);
	__atomic_store_n(&s_borrower_tid, borrower, __ATOMIC_RELEASE);
	return E_OK;
}

ER motor_driver_return(void)
{
	ID borrower = __atomic_load_n(&s_borrower_tid, __ATOMIC_ACQUIRE);

	if (borrower <= 0 || tk_get_tid() != borrower) return E_CTX;
	brake(MOTOR_1);
	brake(MOTOR_2);
	__atomic_store_n(&s_revoke, false, __ATOMIC_RELAXED);
	__atomic_store_n(&s_borrower_tid, 0, __ATOMIC_RELEASE);
	return E_OK;
}

bool motor_driver_is_writer(void)
{
	ID borrower = __atomic_load_n(&s_borrower_tid, __ATOMIC_ACQUIRE);

	if (s_owner_tid <= 0) return false;
	return tk_get_tid() == (borrower > 0 ? borrower : s_owner_tid);
}

bool motor_driver_lent(void)
{
	return __atomic_load_n(&s_borrower_tid, __ATOMIC_ACQUIRE) > 0;
}

void motor_driver_get_output(int16_t *motor_1, int16_t *motor_2)
{
	*motor_1 = __atomic_load_n(&s_output[MOTOR_1], __ATOMIC_RELAXED);
	*motor_2 = __atomic_load_n(&s_output[MOTOR_2], __ATOMIC_RELAXED);
}
