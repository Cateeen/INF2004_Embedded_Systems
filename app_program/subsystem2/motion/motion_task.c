#include "../../app.h"
#include "../../vehicle_buffers.h"
#include "motor_driver.h"
#include "motion_task.h"
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#define MOTION_TASK_PRIORITY 8   /* above the other core-1 tasks: wheel PID */
#define MOTION_FAULT_RETRY_MS 1000U

/* Motor lending (motion_task_borrow_motors). One word, atomic access only:
 * 0 = no request; else the borrower's task ID, plus GRANTED once this task
 * has committed to lend (it then calls motor_driver_lend at once) or DENIED
 * if that failed. The borrower may withdraw (CAS back to 0) only while
 * neither bit is set, so a lend can never go to a task that gave up. */
#define MOTION_BORROW_GRANTED 0x40000000U
#define MOTION_BORROW_DENIED  0x20000000U
#define MOTION_BORROW_ID_MASK 0x0000FFFFU
#define MOTION_BORROW_POLL_MS 10U
static uint32_t s_borrow;

/* Compare-and-swap on s_borrow. Cortex-M0+ has no LDREX/STREX, so GCC's
 * __atomic_compare_exchange_n becomes a libatomic call that is not linked
 * here; use the kernel's spinlock-backed atomic_cmpxchg instead (DI/EI is
 * enough on one core). On failure *expected receives the current value. */
static bool borrow_cas(uint32_t *expected, uint32_t desired)
{
	uint32_t old;
#if TK_SUPPORT_SMP
	old = (uint32_t)atomic_cmpxchg((UW *)&s_borrow, (UW)desired, (UW)*expected);
#else
	UINT saved;

	DI(saved);
	old = s_borrow;
	if (old == *expected)
		s_borrow = desired;
	EI(saved);
#endif
	if (old == *expected)
		return true;
	*expected = old;
	return false;
}

typedef struct {
	motion_control_t control;
	wheel_odometry_message_t pending;
	uint32_t last_clock_ms;
	uint32_t last_command_ms;
	uint32_t last_sequence;
	uint32_t issued_ms;
	uint32_t barrier_ms;
	uint32_t status_sequence;
	uint32_t last_status_ms;
	uint16_t duration_ms;
	uint16_t fault_code;
	int16_t velocity;
	int16_t angular;
	int16_t motor_1_pwm;
	int16_t motor_2_pwm;
	bool have_command;
	bool have_pending;
	bool have_barrier;
	bool have_status;
	bool status_dirty;
	bool active;
	bool spinning;          /* active motion is a TURN (spin on the spot) */
	bool hardware_armed;
	bool faulted;
	bool reported_lent;     /* lent flag in the last published status */
} motion_task_state_t;

static motion_task_config_t s_config;
static motion_task_state_t s_state;
static bool s_started;
static bool s_hardware_enabled;

#define MOTION_LEFT_MOTOR MOTOR_1
#define MOTION_RIGHT_MOTOR MOTOR_2
#define MOTION_LEFT_POLARITY 1
#define MOTION_RIGHT_POLARITY (-1)

static int16_t motor_command(uint8_t pwm, int8_t direction, int8_t polarity)
{
	return (int16_t)((int16_t)pwm * direction * polarity);
}

static void set_commanded_pwm(motion_task_state_t *state, int16_t motor_1,
			      int16_t motor_2)
{
	if (state->motor_1_pwm != motor_1 || state->motor_2_pwm != motor_2) {
		bool was_stopped = state->motor_1_pwm == 0 && state->motor_2_pwm == 0;
		bool now_stopped = motor_1 == 0 && motor_2 == 0;

		state->motor_1_pwm = motor_1;
		state->motor_2_pwm = motor_2;
		/* Starting or stopping is reported at once; speed changes while
		 * running (every odometry sample when steering) wait for the
		 * periodic status so the 2-deep status queue cannot overflow. */
		if (was_stopped != now_stopped) state->status_dirty = true;
	}
}

static void make_motion_status_message(const motion_task_state_t *state,
				       uint32_t sequence, uint32_t now,
				       motion_status_message_t *message)
{
	*message = (motion_status_message_t){0};
	message->header.type = VEHICLE_MESSAGE_MOTION_STATUS;
	message->header.version = VEHICLE_MESSAGE_VERSION;
	message->header.sequence = sequence;
	message->header.timestamp_ms = now;
	message->motor_1_pwm = state->motor_1_pwm;
	message->motor_2_pwm = state->motor_2_pwm;
	message->active = state->active ? 1U : 0U;
	message->faulted = state->faulted ? 1U : 0U;
	message->fault_code = state->faulted ? state->fault_code : 0U;
	if (s_hardware_enabled && motor_driver_lent()) {
		/* The borrower drives: report what it applies. */
		message->lent = 1U;
		motor_driver_get_output(&message->motor_1_pwm, &message->motor_2_pwm);
	}
}

static uint16_t publish_motion_status(motion_task_state_t *state, uint32_t now)
{
	motion_status_message_t message;
	INT result;

	if (!s_hardware_enabled) return 0U;
	if (motor_driver_lent() != state->reported_lent) {
		state->reported_lent = !state->reported_lent;
		state->status_dirty = true;     /* lend and return are reported at once */
	}
	if (state->have_status && !state->status_dirty &&
	    now - state->last_status_ms < VEHICLE_MOTION_STATUS_PERIOD_MS)
		return 0U;
	make_motion_status_message(state, state->status_sequence + 1U, now, &message);
	result = tk_snd_mbf(g_mbf_motion_status, &message,
			    (INT)sizeof(message), TMO_POL);
	if (result != E_OK) {
		tm_printf((UB*)"motion status delivery failed: %d\n", (int)result);
		return VEHICLE_FAULT_CODE_MOTION_STATUS;
	}
	state->status_sequence = message.header.sequence;
	state->last_status_ms = now;
	state->have_status = true;
	state->status_dirty = false;
	return 0U;
}

static void stop_motion(motion_task_state_t *state)
{
	motion_control_stop(&state->control);
	if (state->active) state->status_dirty = true;
	state->active = false;
	state->spinning = false;
	if (s_hardware_enabled) {
		ER er = motor_brake_all();
		if (er != E_OK)
			tm_printf((UB*)"motor brake failed: %d\n", (int)er);
		else
			set_commanded_pwm(state, 0, 0);
	} else {
		set_commanded_pwm(state, 0, 0);
	}
}

static ER apply_motor_output(motion_task_state_t *state,
			     const wheel_speed_output_t *output)
{
	ER left_er;
	ER right_er;
	int16_t motor_1;
	int16_t motor_2;

	if (!s_hardware_enabled) return E_OK;
	if (!state->hardware_armed || !state->active ||
	    !state->control.have_output)
	{
		ER er = motor_brake_all();
		if (er == E_OK) set_commanded_pwm(state, 0, 0);
		return er;
	}
	motor_1 = motor_command(output->left_pwm_percent,
				state->control.left_direction, MOTION_LEFT_POLARITY);
	motor_2 = motor_command(output->right_pwm_percent,
				state->control.right_direction, MOTION_RIGHT_POLARITY);
	left_er = motor_set_speed(MOTION_LEFT_MOTOR,
				  motor_1);
	right_er = motor_set_speed(MOTION_RIGHT_MOTOR,
				   motor_2);
	if (left_er != E_OK && left_er != E_BUSY) return left_er;
	if (right_er != E_OK && right_er != E_BUSY) return right_er;
	if (left_er == E_BUSY || right_er == E_BUSY) {
		ER er = motor_brake_all();
		if (er == E_OK) set_commanded_pwm(state, 0, 0);
		return er;
	}
	set_commanded_pwm(state, motor_1, motor_2);
	return E_OK;
}

static uint16_t discard_odometry(void)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY; i++) {
		wheel_odometry_message_t message;
		INT received = tk_rcv_mbf(g_mbf_motion_odometry, &message, TMO_POL);
		if (received == E_TMOUT) break;
		if (received != (INT)sizeof(message))
			return VEHICLE_FAULT_CODE_MOTION_ODOMETRY;
	}
	return 0U;
}

static uint16_t receive_commands(motion_task_state_t *state,
				 const motion_task_config_t *config,
				 uint32_t now, bool suppress_motion)
{
	unsigned i;

	for (i = 0U; i < VEHICLE_MB_MOTION_CAPACITY; i++) {
		motion_command_message_t command;
		uint32_t age;
		int32_t speed;
		int32_t turn;
		INT received = tk_rcv_mbf(g_mbf_motion_commands, &command, TMO_POL);
		if (received == E_TMOUT) break;
		if (received != (INT)sizeof(command) ||
		    command.header.type != VEHICLE_MESSAGE_MOTION_COMMAND ||
		    command.header.version != VEHICLE_MESSAGE_VERSION)
			return VEHICLE_FAULT_CODE_MOTION_COMMAND;
		if (command.mode == VEHICLE_MOTION_EMERGENCY_STOP) {
			stop_motion(state);
			return VEHICLE_FAULT_CODE_MOTION_EMERGENCY;
		}
		if (command.mode == VEHICLE_MOTION_STOP) {
			stop_motion(state);
			state->barrier_ms = now;
			state->have_barrier = true;
			suppress_motion = true;
		}
		age = now - command.header.timestamp_ms;
		if ((state->have_command &&
		     (command.header.sequence - state->last_sequence != 1U ||
		      command.header.timestamp_ms - state->last_command_ms > INT32_MAX)) ||
		    (command.mode != VEHICLE_MOTION_STOP && age > config->command_max_age_ms))
			return VEHICLE_FAULT_CODE_MOTION_COMMAND;
		switch (command.mode) {
		case VEHICLE_MOTION_STOP:
			break;
		case VEHICLE_MOTION_ARM:
			if (!s_hardware_enabled || state->active ||
			    command.linear_velocity != 0 ||
			    command.angular_velocity != 0 ||
			    command.duration_ms != 0U)
				return VEHICLE_FAULT_CODE_MOTION_COMMAND;
			if (motor_clear_emergency_stop() != E_OK)
				return VEHICLE_FAULT_CODE_MOTION_ACTUATOR;
			state->hardware_armed = true;
			state->barrier_ms = command.header.timestamp_ms;
			state->have_barrier = true;
			break;
		case VEHICLE_MOTION_VELOCITY:
			speed = command.linear_velocity;
			if (speed < 0) speed = -speed;
			turn = command.angular_velocity;
			if (turn < 0) turn = -turn;
			/* Wheel targets are |v - w| (left) and |v + w| (right), so
			 * |w| < |v| keeps both wheels turning the same way. */
			if (speed == 0 || turn >= speed ||
			    speed + turn > config->max_linear_speed_cm_s ||
			    command.duration_ms == 0U ||
			    command.duration_ms > config->max_duration_ms ||
			    age >= command.duration_ms ||
			    (s_hardware_enabled && !state->hardware_armed))
				return VEHICLE_FAULT_CODE_MOTION_COMMAND;
			if (!suppress_motion && (!state->have_barrier ||
			    (command.header.timestamp_ms - state->barrier_ms != 0U &&
			     command.header.timestamp_ms - state->barrier_ms <= INT32_MAX))) {
				int32_t left = (int32_t)command.linear_velocity -
					       command.angular_velocity;
				int32_t right = (int32_t)command.linear_velocity +
						command.angular_velocity;

				/* Steering may change while moving; reversing, or
				 * switching between driving and spinning, needs STOP. */
				if (state->active && (state->spinning ||
				    (state->velocity > 0) != (command.linear_velocity > 0)))
					return VEHICLE_FAULT_CODE_MOTION_COMMAND;
				if (!state->active) {
					motion_control_config_t control = config->control;
					control.speed.target_speed_cm_s = (float)speed;
					if (!motion_control_start(&state->control, &control,
					    command.linear_velocity > 0 ? 1 : -1, now))
						return VEHICLE_FAULT_CODE_MOTION_CONTROL;
					state->active = true;
					state->status_dirty = true;
				}
				if (!motion_control_set_targets(&state->control,
				    (float)(left < 0 ? -left : left),
				    (float)(right < 0 ? -right : right)))
					return VEHICLE_FAULT_CODE_MOTION_CONTROL;
				state->velocity = command.linear_velocity;
				state->angular = command.angular_velocity;
				state->issued_ms = command.header.timestamp_ms;
				state->duration_ms = command.duration_ms;
			}
			break;
		case VEHICLE_MOTION_TURN:
			/* Spin on the spot: angular_velocity is each wheel's speed in
			 * cm/s, + = left (left wheel back, right forward); linear 0.
			 * Same lifetime and arming rules as VELOCITY. */
			turn = command.angular_velocity;
			if (turn < 0) turn = -turn;
			if (command.linear_velocity != 0 || turn == 0 ||
			    turn > config->max_linear_speed_cm_s ||
			    command.duration_ms == 0U ||
			    command.duration_ms > config->max_duration_ms ||
			    age >= command.duration_ms ||
			    (s_hardware_enabled && !state->hardware_armed))
				return VEHICLE_FAULT_CODE_MOTION_COMMAND;
			if (!suppress_motion && (!state->have_barrier ||
			    (command.header.timestamp_ms - state->barrier_ms != 0U &&
			     command.header.timestamp_ms - state->barrier_ms <= INT32_MAX))) {
				int8_t direction = command.angular_velocity > 0 ? 1 : -1;

				/* Changing spin direction or from driving needs STOP. */
				if (state->active && (!state->spinning ||
				    (state->angular > 0) != (command.angular_velocity > 0)))
					return VEHICLE_FAULT_CODE_MOTION_COMMAND;
				if (!state->active) {
					motion_control_config_t control = config->control;
					control.speed.target_speed_cm_s = (float)turn;
					if (!motion_control_start_spin(&state->control, &control,
								       direction, now))
						return VEHICLE_FAULT_CODE_MOTION_CONTROL;
					state->active = true;
					state->spinning = true;
					state->status_dirty = true;
				}
				if (!motion_control_set_targets(&state->control, (float)turn,
								(float)turn))
					return VEHICLE_FAULT_CODE_MOTION_CONTROL;
				state->velocity = 0;
				state->angular = command.angular_velocity;
				state->issued_ms = command.header.timestamp_ms;
				state->duration_ms = command.duration_ms;
			}
			break;
		default:
			return VEHICLE_FAULT_CODE_MOTION_COMMAND;
		}
		state->last_command_ms = command.header.timestamp_ms;
		state->last_sequence = command.header.sequence;
		state->have_command = true;
	}
	return 0U;
}

/* Lend the motors to a waiting borrower, only while stopped and armed. */
static void serve_borrow(const motion_task_state_t *state)
{
	uint32_t want = __atomic_load_n(&s_borrow, __ATOMIC_ACQUIRE);

	if (want == 0U || (want & (MOTION_BORROW_GRANTED | MOTION_BORROW_DENIED)) != 0U)
		return;
	if (!s_hardware_enabled || state->active || !state->hardware_armed)
		return;         /* the borrower times out and withdraws */
	if (!borrow_cas(&want, want | MOTION_BORROW_GRANTED))
		return;         /* withdrawn just now */
	if (motor_driver_lend((ID)(want & MOTION_BORROW_ID_MASK)) != E_OK)
		__atomic_store_n(&s_borrow, want | MOTION_BORROW_DENIED, __ATOMIC_RELEASE);
}

static uint16_t run_cycle(motion_task_state_t *state,
			  const motion_task_config_t *config, uint32_t now)
{
	wheel_speed_output_t output;
	motion_control_result_t control_result;
	uint16_t fault;
	bool expired = false;
	unsigned i;

	if (now - state->last_clock_ms > INT32_MAX)
		return VEHICLE_FAULT_CODE_MOTION_CLOCK;
	state->last_clock_ms = now;
	if (state->active && now - state->issued_ms >= state->duration_ms) {
		stop_motion(state);
		state->barrier_ms = now;
		state->have_barrier = true;
		expired = true;
	}
	if (!state->active) {
		state->have_pending = false;
		fault = discard_odometry();
		if (fault != 0U) return fault;
	}
	fault = receive_commands(state, config, now, expired);
	if (fault != 0U) return fault;
	if (!state->active) {
		serve_borrow(state);
		fault = discard_odometry();
		if (fault != 0U) return fault;
		return publish_motion_status(state, now);
	}
	for (i = 0U; i < VEHICLE_MB_ODOMETRY_CAPACITY + 1U; i++) {
		wheel_odometry_message_t message;
		uint32_t ahead;
		if (state->have_pending) {
			message = state->pending;
		} else {
			INT received = tk_rcv_mbf(g_mbf_motion_odometry, &message, TMO_POL);
			if (received == E_TMOUT) break;
			if (received != (INT)sizeof(message))
				return VEHICLE_FAULT_CODE_MOTION_ODOMETRY;
		}
		ahead = message.header.timestamp_ms - now;
		if (ahead != 0U && ahead <= INT32_MAX) {
			state->pending = message;
			state->have_pending = true;
			break;
		}
		state->have_pending = false;
		if (motion_control_step(&state->control, &message, now, &output) ==
		    MOTION_CONTROL_FAULT)
			return VEHICLE_FAULT_CODE_MOTION_ODOMETRY;
	}
	control_result = motion_control_step(&state->control, NULL, now, &output);
	if (control_result == MOTION_CONTROL_FAULT)
		return VEHICLE_FAULT_CODE_MOTION_ODOMETRY;
	if (apply_motor_output(state, &output) != E_OK)
		return VEHICLE_FAULT_CODE_MOTION_ACTUATOR;
	return publish_motion_status(state, now);
}

static void report_fault(uint16_t code, uint32_t now)
{
	fault_message_t fault = {0};
	bool sent = false;

	stop_motion(&s_state);
	if (s_hardware_enabled) {
		ER er = motor_emergency_stop();
		if (er != E_OK) {
			tm_printf((UB*)"motor emergency stop failed: %d\n", (int)er);
		} else {
			set_commanded_pwm(&s_state, 0, 0);
		}
	}
	s_state.faulted = true;
	s_state.fault_code = code;
	s_state.status_dirty = true;
	if (publish_motion_status(&s_state, now) != 0U)
		tm_printf((UB*)"motion fault status could not be queued\n");
	fault.header.type = VEHICLE_MESSAGE_FAULT;
	fault.header.version = VEHICLE_MESSAGE_VERSION;
	fault.header.sequence = 1U;
	fault.header.timestamp_ms = now;
	fault.source = VEHICLE_FAULT_MOTION;
	fault.code = code;
	fault.recoverable = 0U;
	tm_printf((UB*)"motion task fault; code %u\n", (unsigned)code);
	for (;;) {
		ER er;
		if (!sent) {
			er = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
			if (er == E_OK) sent = true;
			else tm_printf((UB*)"motion fault delivery failed: %d\n", (int)er);
		}
		er = tk_dly_tsk(MOTION_FAULT_RETRY_MS);
		if (er != E_OK) {
			tm_printf((UB*)"motion fault retry delay failed: %d\n", (int)er);
			er = tk_slp_tsk(TMO_FEVR);
			tm_printf((UB*)"motion fault sleep returned: %d\n", (int)er);
		}
	}
}

static void motion_task(INT stacd, void *exinf)
{
	SYSTIM now;
	uint32_t timestamp = 0U;
	ER er;

	(void)stacd;
	(void)exinf;
	if (s_hardware_enabled) {
		er = motor_driver_init();
		if (er != E_OK) report_fault(VEHICLE_FAULT_CODE_MOTION_ACTUATOR, timestamp);
	}
	er = tk_get_otm(&now);
	if (er != E_OK) report_fault(VEHICLE_FAULT_CODE_MOTION_CLOCK, timestamp);
	timestamp = now.lo;
	s_state.last_clock_ms = timestamp;
	for (;;) {
		uint16_t fault;
		er = tk_get_otm(&now);
		if (er != E_OK) report_fault(VEHICLE_FAULT_CODE_MOTION_CLOCK, timestamp);
		timestamp = now.lo;
		fault = run_cycle(&s_state, &s_config, timestamp);
		if (fault != 0U) report_fault(fault, timestamp);
		er = tk_dly_tsk(s_config.period_ms);
		if (er != E_OK) report_fault(VEHICLE_FAULT_CODE_MOTION_SCHEDULE, timestamp);
	}
}

static ER motion_task_start_internal(const motion_task_config_t *config,
				     bool hardware_enabled)
{
	motion_control_t validation = {0};
	T_CTSK task = {0};
	ID id;
	ER er;

	if (s_started) return E_OBJ;
	if (config == NULL || config->period_ms == 0U ||
	    config->period_ms > config->control.feedback.max_age_ms ||
	    config->period_ms > config->control.feedback.max_interval_ms ||
	    config->command_max_age_ms == 0U || config->command_max_age_ms > INT32_MAX ||
	    config->max_duration_ms == 0U || config->max_linear_speed_cm_s == 0U ||
	    !motion_control_start(&validation, &config->control, 1, 0U))
		return E_PAR;
	if (g_mbf_motion_commands <= 0 || g_mbf_motion_odometry <= 0 ||
	    g_mbf_faults <= 0 ||
	    (hardware_enabled && g_mbf_motion_status <= 0))
		return E_OBJ;
	s_config = *config;
	s_state = (motion_task_state_t){0};
	s_hardware_enabled = hardware_enabled;
	s_state.status_dirty = hardware_enabled;
	task.task = (FP)motion_task;
	task.tskatr = TA_HLNG | TA_RNG3;
	task.itskpri = MOTION_TASK_PRIORITY;
	task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
	/* Core 1 (TP_PRC2): core 0 runs the radio (priority 4) and USB console
	 * (3), which pre-empt any task there for milliseconds at a time. */
	task.tskatr |= TA_ASSPRC;
	task.assprc = TP_PRC2;
#endif
	id = tk_cre_tsk(&task);
	if (id < E_OK) {
		s_hardware_enabled = false;
		return id;
	}
	s_started = true;
	er = tk_sta_tsk(id, 0);
	if (er != E_OK) {
		ER cleanup = tk_del_tsk(id);
		if (cleanup != E_OK) tm_printf((UB*)"motion task cleanup failed: %d\n", (int)cleanup);
		else {
			s_started = false;
			s_hardware_enabled = false;
		}
		return er < E_OK ? er : E_SYS;
	}
	return E_OK;
}

ER motion_task_start(const motion_task_config_t *config)
{
	return motion_task_start_internal(config, false);
}

ER motion_task_start_hardware(const motion_task_config_t *config)
{
	return motion_task_start_internal(config, true);
}

bool motion_task_borrow_motors(bool (*abort)(void *context), void *context,
			       uint32_t timeout_ms)
{
	ID tid = tk_get_tid();
	uint32_t self;
	uint32_t expected = 0U;
	uint32_t waited = 0U;

	if (!s_started || !s_hardware_enabled || tid <= 0 ||
	    (uint32_t)tid > MOTION_BORROW_ID_MASK)
		return false;
	self = (uint32_t)tid;
	if (!borrow_cas(&expected, self))
		return false;   /* another task is borrowing */
	for (;;) {
		uint32_t r = __atomic_load_n(&s_borrow, __ATOMIC_ACQUIRE);

		if ((r & MOTION_BORROW_DENIED) != 0U) {
			__atomic_store_n(&s_borrow, 0U, __ATOMIC_RELEASE);
			return false;
		}
		if ((r & MOTION_BORROW_GRANTED) != 0U) {
			/* Committed: the lend follows within microseconds. */
			if (motor_driver_is_writer()) {
				__atomic_store_n(&s_borrow, 0U, __ATOMIC_RELEASE);
				return true;
			}
		} else if (waited >= timeout_ms || (abort != NULL && abort(context))) {
			expected = self;
			if (borrow_cas(&expected, 0U))
				return false;
			continue;       /* granted meanwhile: take it; the caller returns it */
		}
		(void)tk_dly_tsk(MOTION_BORROW_POLL_MS);
		waited += MOTION_BORROW_POLL_MS;
	}
}

void motion_task_return_motors(void)
{
	(void)motor_brake_all();
	(void)motor_driver_return();    /* E_CTX if already revoked: nothing to do */
}

#if APP_TEST
#include "../../test_report.h"

static motion_command_message_t test_command(uint32_t sequence, uint32_t timestamp,
					      uint8_t mode, int16_t velocity,
					      uint16_t duration)
{
	motion_command_message_t command = {0};
	command.header.type = VEHICLE_MESSAGE_MOTION_COMMAND;
	command.header.version = VEHICLE_MESSAGE_VERSION;
	command.header.sequence = sequence;
	command.header.timestamp_ms = timestamp;
	command.mode = mode;
	command.linear_velocity = velocity;
	command.duration_ms = duration;
	return command;
}

static wheel_odometry_message_t test_odometry(uint32_t sequence, uint32_t timestamp,
					      int32_t count)
{
	wheel_odometry_message_t message = {0};
	message.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = sequence;
	message.header.timestamp_ms = timestamp;
	message.left_count = count;
	message.right_count = count;
	message.valid = 1U;
	return message;
}

static BOOL send_command(const motion_command_message_t *command)
{
	return tk_snd_mbf(g_mbf_motion_commands, command, (INT)sizeof(*command), TMO_POL) == E_OK;
}

static BOOL send_odometry(const wheel_odometry_message_t *message)
{
	return tk_snd_mbf(g_mbf_motion_odometry, message, (INT)sizeof(*message), TMO_POL) == E_OK;
}

static BOOL drain_test_buffers(void)
{
	union {
		motion_command_message_t command;
		wheel_odometry_message_t odometry;
	} message;
	INT result;

	do {
		result = tk_rcv_mbf(g_mbf_motion_commands, &message, TMO_POL);
	} while (result > 0);
	if (result != E_TMOUT) return FALSE;
	return discard_odometry() == 0U;
}

static const motion_task_config_t test_config = {
	{{{2500U, 20.0f}, {2500U, 20.0f}, 150U, 250U},
	 {2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U}},
	10U, 100U, 500U, 20U
};

static BOOL simulation_flow_test(void)
{
	motion_task_state_t state = {0};
	motion_command_message_t command;
	wheel_odometry_message_t odometry;
	BOOL ok;

	state.last_clock_ms = 100U;
	odometry = test_odometry(20U, 99U, 0);
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_odometry(&odometry) && send_command(&command);
	ok = run_cycle(&state, &test_config, 100U) == 0U && state.active &&
	     !state.control.have_output && ok;
	odometry = test_odometry(21U, 101U, 0);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 100U) == 0U &&
	     state.have_pending && ok;
	ok = run_cycle(&state, &test_config, 101U) == 0U && !state.have_pending &&
	     state.control.feedback.have_baseline && ok;
	odometry = test_odometry(22U, 201U, 125);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 201U) == 0U &&
	     state.control.have_output && state.control.output.left_pwm_percent == 20U && ok;
	command = test_command(2U, 202U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 202U) == 0U &&
	     state.issued_ms == 202U && state.control.have_output && ok;
	command = test_command(3U, 502U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 502U) == 0U &&
	     !state.active && ok;
	command = test_command(4U, 503U, VEHICLE_MOTION_VELOCITY, -10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 503U) == 0U &&
	     state.active && state.control.direction == -1 && ok;
	command = test_command(5U, 504U, VEHICLE_MOTION_STOP, 0, 0U);
	ok = send_command(&command) && ok;
	command = test_command(6U, 504U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 504U) == 0U &&
	     !state.active && ok;
	command = test_command(7U, 504U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 505U) == 0U &&
	     !state.active && ok;
	command = test_command(8U, 505U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 505U) == 0U &&
	     state.active && ok;
	stop_motion(&state);
	return drain_test_buffers() && ok;
}

static BOOL simulation_fault_test(void)
{
	motion_task_state_t state = {0};
	motion_command_message_t command;
	wheel_odometry_message_t odometry;
	BOOL ok;

	state.last_clock_ms = 100U;
	command = test_command(1U, 100U, VEHICLE_MOTION_ARM, 0, 0U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) ==
	     VEHICLE_FAULT_CODE_MOTION_COMMAND;
	/* Steering that would stop or reverse the inner wheel. */
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 10, 100U);
	command.angular_velocity = 10;
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	/* Outer wheel over the speed limit: 15 + 6 > 20. */
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 15, 100U);
	command.angular_velocity = -6;
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	command = test_command(1U, 99U, VEHICLE_MOTION_VELOCITY, 10, 1U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	command = test_command(1U, 101U, VEHICLE_MOTION_VELOCITY, 10, 100U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 21, 100U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 100U) == 0U && ok;
	command = test_command(3U, 101U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 101U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	command = test_command(9U, 0U, VEHICLE_MOTION_EMERGENCY_STOP, 0, 0U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 101U) == VEHICLE_FAULT_CODE_MOTION_EMERGENCY &&
	     !state.active && ok;
	state = (motion_task_state_t){0};
	state.last_clock_ms = 100U;
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 100U) == 0U && ok;
	odometry = test_odometry(1U, 100U, 0);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 100U) == 0U && ok;
	odometry = test_odometry(3U, 101U, 0);
	ok = send_odometry(&odometry) &&
	     run_cycle(&state, &test_config, 101U) == VEHICLE_FAULT_CODE_MOTION_ODOMETRY && ok;
	stop_motion(&state);
	return drain_test_buffers() && ok;
}

static BOOL simulation_deadline_test(void)
{
	motion_task_state_t state = {0};
	motion_command_message_t command;
	wheel_odometry_message_t odometry;
	BOOL ok;

	state.last_clock_ms = UINT32_MAX - 10U;
	command = test_command(UINT32_MAX, UINT32_MAX - 10U,
			       VEHICLE_MOTION_VELOCITY, 10, 100U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, UINT32_MAX - 10U) == 0U && state.active;
	odometry = test_odometry(UINT32_MAX, UINT32_MAX - 10U, 0);
	ok = send_odometry(&odometry) &&
	     run_cycle(&state, &test_config, UINT32_MAX - 10U) == 0U && ok;
	odometry = test_odometry(0U, 39U, 62);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 39U) == 0U &&
	     state.control.have_output && ok;
	ok = run_cycle(&state, &test_config, 88U) == 0U && state.active && ok;
	ok = run_cycle(&state, &test_config, 89U) == 0U && !state.active &&
	     state.control.output.left_pwm_percent == 0U && ok;
	command = test_command(0U, 0U, VEHICLE_MOTION_VELOCITY, 10, 500U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 101U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	return drain_test_buffers() && ok;
}

static BOOL simulation_no_status_test(void)
{
	motion_task_state_t state = {0};
	motion_command_message_t command;
	motion_status_message_t status;
	BOOL ok;

	state.last_clock_ms = 100U;
	ok = run_cycle(&state, &test_config, 100U) == 0U &&
	     tk_rcv_mbf(g_mbf_motion_status, &status, TMO_POL) == E_TMOUT;
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 100U) == 0U &&
	     state.active && tk_rcv_mbf(g_mbf_motion_status, &status, TMO_POL) == E_TMOUT && ok;
	command = test_command(2U, 101U, VEHICLE_MOTION_STOP, 0, 0U);
	ok = send_command(&command) && run_cycle(&state, &test_config, 101U) == 0U &&
	     !state.active && tk_rcv_mbf(g_mbf_motion_status, &status, TMO_POL) == E_TMOUT && ok;
	stop_motion(&state);
	return drain_test_buffers() && ok;
}

/* Steering: per-wheel targets from v and w, updates while moving keep the
 * PIDs, each wheel is corrected towards its own target, reversing faults. */
static BOOL simulation_steering_test(void)
{
	motion_task_state_t state = {0};
	motion_command_message_t command;
	wheel_odometry_message_t odometry;
	BOOL ok;

	state.last_clock_ms = 100U;
	command = test_command(1U, 100U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	command.angular_velocity = 3;
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 100U) == 0U && state.active &&
	     state.control.speed.left_target_cm_s == 7.0f &&
	     state.control.speed.right_target_cm_s == 13.0f;
	/* Odometry before the start is discarded; this sample is the baseline. */
	odometry = test_odometry(20U, 100U, 0);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 100U) == 0U &&
	     state.control.feedback.have_baseline && ok;
	/* Both wheels measured at 10 cm/s: base 20 + 2 * (target - 10). */
	odometry = test_odometry(21U, 200U, 125);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 200U) == 0U &&
	     state.control.have_output && state.control.output.left_pwm_percent == 14U &&
	     state.control.output.right_pwm_percent == 26U && ok;
	command = test_command(2U, 201U, VEHICLE_MOTION_VELOCITY, 12, 300U);
	command.angular_velocity = -2;
	ok = send_command(&command) && run_cycle(&state, &test_config, 201U) == 0U &&
	     state.active && state.issued_ms == 201U &&
	     state.control.speed.left_target_cm_s == 14.0f &&
	     state.control.speed.right_target_cm_s == 10.0f &&
	     !state.control.speed.left_pid.first_update && ok;
	command = test_command(3U, 202U, VEHICLE_MOTION_VELOCITY, -10, 300U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 202U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	stop_motion(&state);
	ok = drain_test_buffers() && ok;

	/* Reverse with steering: left |-10 - 3| = 13, right |-10 + 3| = 7. */
	state = (motion_task_state_t){0};
	state.last_clock_ms = 300U;
	command = test_command(1U, 300U, VEHICLE_MOTION_VELOCITY, -10, 300U);
	command.angular_velocity = 3;
	ok = send_command(&command) && run_cycle(&state, &test_config, 300U) == 0U &&
	     state.active && state.control.direction == -1 &&
	     state.control.speed.left_target_cm_s == 13.0f &&
	     state.control.speed.right_target_cm_s == 7.0f && ok;
	stop_motion(&state);
	return drain_test_buffers() && ok;
}

static wheel_odometry_message_t test_odometry_lr(uint32_t sequence, uint32_t timestamp,
						 int32_t left, int32_t right)
{
	wheel_odometry_message_t message = test_odometry(sequence, timestamp, 0);
	message.left_count = left;
	message.right_count = right;
	return message;
}

/* TURN: spin on the spot with the wheels in opposite directions; feedback
 * is checked per wheel; switching to VELOCITY or bad TURNs are faults. */
static BOOL simulation_spin_test(void)
{
	motion_task_state_t state = {0};
	motion_command_message_t command;
	wheel_odometry_message_t odometry;
	BOOL ok;

	/* Spin left at 8 cm/s per wheel. */
	state.last_clock_ms = 100U;
	command = test_command(1U, 100U, VEHICLE_MOTION_TURN, 0, 300U);
	command.angular_velocity = 8;
	ok = send_command(&command) && run_cycle(&state, &test_config, 100U) == 0U &&
	     state.active && state.spinning && state.control.direction == 0 &&
	     state.control.left_direction == -1 && state.control.right_direction == 1;
	/* Left wheel backward, right forward (16 cm/s each): valid output. */
	odometry = test_odometry_lr(1U, 101U, 0, 0);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 101U) == 0U && ok;
	odometry = test_odometry_lr(2U, 151U, -100, 100);
	ok = send_odometry(&odometry) && run_cycle(&state, &test_config, 151U) == 0U &&
	     state.control.have_output && state.control.output.left_pwm_percent == 4U &&
	     state.control.output.right_pwm_percent == 4U && ok;
	/* A refresh in the same direction is accepted. */
	command = test_command(2U, 160U, VEHICLE_MOTION_TURN, 0, 300U);
	command.angular_velocity = 8;
	ok = send_command(&command) && run_cycle(&state, &test_config, 160U) == 0U &&
	     state.issued_ms == 160U && state.spinning && ok;
	/* The left wheel turning forward during a left spin is a fault. */
	odometry = test_odometry_lr(3U, 201U, 0, 200);
	ok = send_odometry(&odometry) &&
	     run_cycle(&state, &test_config, 201U) == VEHICLE_FAULT_CODE_MOTION_ODOMETRY && ok;
	ok = drain_test_buffers() && ok;

	/* VELOCITY while spinning, or reversing the spin, needs STOP first. */
	state = (motion_task_state_t){0};
	state.last_clock_ms = 300U;
	command = test_command(1U, 300U, VEHICLE_MOTION_TURN, 0, 300U);
	command.angular_velocity = -8;
	ok = send_command(&command) && run_cycle(&state, &test_config, 300U) == 0U &&
	     state.control.left_direction == 1 && state.control.right_direction == -1 && ok;
	command = test_command(2U, 301U, VEHICLE_MOTION_VELOCITY, 10, 300U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 301U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	ok = drain_test_buffers() && ok;
	state = (motion_task_state_t){0};
	state.last_clock_ms = 400U;
	command = test_command(1U, 400U, VEHICLE_MOTION_TURN, 0, 300U);
	command.angular_velocity = 8;
	ok = send_command(&command) && run_cycle(&state, &test_config, 400U) == 0U && ok;
	command = test_command(2U, 401U, VEHICLE_MOTION_TURN, 0, 300U);
	command.angular_velocity = -8;
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 401U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	ok = drain_test_buffers() && ok;

	/* Invalid TURNs: linear speed set, zero speed, too fast. */
	state = (motion_task_state_t){0};
	state.last_clock_ms = 500U;
	command = test_command(1U, 500U, VEHICLE_MOTION_TURN, 5, 300U);
	command.angular_velocity = 8;
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 500U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	ok = drain_test_buffers() && ok;
	state = (motion_task_state_t){0};
	state.last_clock_ms = 600U;
	command = test_command(1U, 600U, VEHICLE_MOTION_TURN, 0, 300U);
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 600U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	ok = drain_test_buffers() && ok;
	state = (motion_task_state_t){0};
	state.last_clock_ms = 700U;
	command = test_command(1U, 700U, VEHICLE_MOTION_TURN, 0, 300U);
	command.angular_velocity = 21;
	ok = send_command(&command) &&
	     run_cycle(&state, &test_config, 700U) == VEHICLE_FAULT_CODE_MOTION_COMMAND && ok;
	return drain_test_buffers() && ok;
}

static BOOL actuator_mapping_test(void)
{
	return motor_command(40U, 1, MOTION_LEFT_POLARITY) == 40 &&
	       motor_command(40U, 1, MOTION_RIGHT_POLARITY) == -40 &&
	       motor_command(40U, -1, MOTION_LEFT_POLARITY) == -40 &&
	       motor_command(40U, -1, MOTION_RIGHT_POLARITY) == 40;
}

static BOOL status_format_test(void)
{
	motion_task_state_t state = {0};
	motion_status_message_t status;

	state.active = true;
	state.motor_1_pwm = 40;
	state.motor_2_pwm = -40;
	make_motion_status_message(&state, 3U, 125U, &status);
	if (status.header.type != VEHICLE_MESSAGE_MOTION_STATUS ||
	    status.header.version != VEHICLE_MESSAGE_VERSION ||
	    status.header.sequence != 3U || status.header.timestamp_ms != 125U ||
	    status.active != 1U || status.faulted != 0U || status.fault_code != 0U ||
	    status.motor_1_pwm != 40 || status.motor_2_pwm != -40)
		return false;
	state.faulted = true;
	state.fault_code = VEHICLE_FAULT_CODE_MOTION_ACTUATOR;
	make_motion_status_message(&state, 4U, 225U, &status);
	return status.faulted == 1U &&
	       status.fault_code == VEHICLE_FAULT_CODE_MOTION_ACTUATOR &&
	       status.motor_1_pwm == 40 && status.motor_2_pwm == -40;
}

BOOL motion_task_test(void)
{
	BOOL flow_ok;
	BOOL fault_ok;
	BOOL deadline_ok;
	BOOL status_ok;
	BOOL mapping_ok;
	BOOL status_format_ok;
	BOOL steering_ok;
	BOOL spin_ok;
	TEST_INIT();

	flow_ok = simulation_flow_test();
	spin_ok = simulation_spin_test();
	steering_ok = simulation_steering_test();
	fault_ok = simulation_fault_test();
	deadline_ok = simulation_deadline_test();
	status_ok = simulation_no_status_test();
	mapping_ok = actuator_mapping_test();
	status_format_ok = status_format_test();
	TEST_CHECK("MOT-U07", flow_ok, "simulation command lifetime, feedback and stop precedence");
	TEST_CHECK("MOT-U08", fault_ok, "invalid commands, emergency and odometry faults");
	TEST_CHECK("MOT-U09", deadline_ok, "timestamp expiry and clock wrap");
	TEST_CHECK("MOT-U10", status_ok, "simulation does not publish motion status");
	TEST_CHECK("MOT-U11", mapping_ok, "forward and reverse commands preserve source motor polarity");
	TEST_CHECK("MOT-U12", status_format_ok, "hardware status reports commanded PWM and latched faults");
	TEST_CHECK("MOT-U13", steering_ok, "per-wheel steering targets, live updates and reverse-while-moving fault");
	TEST_CHECK("MOT-U15", spin_ok, "TURN spins with opposite wheels, per-wheel feedback, needs STOP to switch");
	TEST_SUMMARY("motion task simulation");
	return flow_ok && fault_ok && deadline_ok && status_ok && mapping_ok &&
	       status_format_ok && steering_ok && spin_ok;
}
#endif
