/*
 * obs_port_pico_tk.c - Pico SDK + micro T-Kernel implementation of obs_port.h.
 *
 * STATUS: NOT COMPILED OR TESTED. The portable logic (obs_scan/obs_profile/
 * obs_pack) is host-tested; this file is the only board-specific part.
 * It is written against the standard micro T-Kernel 2.0 API and the Pico SDK.
 * Every "VERIFY" marks a name, header or behaviour to confirm against the
 * team's GitHub template.
 */
#include "obs_port.h"
#include "obs_cfg.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <tk/tkernel.h>                   /* VERIFY: header path in the template */
#include <stdio.h>
#include <string.h>

/* ---- Objects owned elsewhere (agree names with the team) --------------- */
extern ID g_flg_fault;                    /* team fault word, event flag (32 bit)      */
extern ID g_flg_status;                   /* status flags: obstacle present            */
extern bool line_following_line_detected(void);   /* Buddy 3 API (name to agree) */

#define STATUS_OBSTACLE_PRESENT   (1UL << 0)      /* PROPOSAL: bit in g_flg_status */
#define FLG_ECHO_RISE             (1UL << 0)
#define FLG_ECHO_DONE             (1UL << 1)
#define OBS_TASK_PRI              6               /* PLACEHOLDER: team assigns numbers   */
#define OBS_TASK_STACK_BYTES      2048            /* PLACEHOLDER: measure stack high-water */

static ID s_flg_echo;
static volatile uint16_t s_rise_us;               /* written only by the ISR            */
static volatile uint16_t s_pulse_us;              /* written only by the ISR            */
static uint32_t s_obstacle_sequence;
static uint32_t s_fault_sequence;

/**
 * obs_echo_irq - ECHO pin edge handler (runs in interrupt context).
 * What it does: on the rising edge it stores the low 16 bits of the microsecond
 * timer and sets the RISE event flag. On the falling edge it computes the pulse
 * width (valid because the pulse is under 65 ms) and sets the DONE event flag.
 * It does no maths and no waiting. The ISR is the only writer of s_rise_us and
 * s_pulse_us, so no lock is needed.
 * Params : gpio = pin that fired, events = GPIO_IRQ_EDGE_RISE / _FALL bits.
 */
/*
 * ECHO edge handler. Do NOT register it with gpio_set_irq_enabled_with_callback():
 * the SDK keeps one callback per core, shared with the encoders (GP16-19) and
 * any IMU INT1. The team's single GPIO dispatcher calls this (see INTEGRATION.md).
 */
void obs_echo_irq(unsigned int gpio, uint32_t events)
{
    (void)gpio;
    if ((events & GPIO_IRQ_EDGE_RISE) != 0U) {
        s_rise_us = (uint16_t)time_us_32();
        (void)tk_set_flg(s_flg_echo, FLG_ECHO_RISE);        /* VERIFY: allowed from handler */
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0U) {
        s_pulse_us = (uint16_t)((uint16_t)time_us_32() - s_rise_us);   /* pulse < 65 ms */
        (void)tk_set_flg(s_flg_echo, FLG_ECHO_DONE);
    }
}

/**
 * obs_port_init - set up Buddy 5's hardware and kernel objects.
 * What it does: TRIG as output (low), ECHO as input with pull-down and edge
 * interrupts enabled (the shared dispatcher calls obs_echo_irq), servo pin as PWM
 * at 50 Hz with 1 us per tick, and creates the echo event flag.
 */
void obs_port_init(void)
{
    uint slice;
    pwm_config cfg;
    T_CFLG cflg;

    gpio_init(OBS_PIN_TRIG);
    gpio_set_dir(OBS_PIN_TRIG, GPIO_OUT);
    gpio_put(OBS_PIN_TRIG, 0);

    gpio_init(OBS_PIN_ECHO);
    gpio_set_dir(OBS_PIN_ECHO, GPIO_IN);
    gpio_pull_down(OBS_PIN_ECHO);
    gpio_set_irq_enabled(OBS_PIN_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_function(OBS_PIN_SERVO, GPIO_FUNC_PWM);       /* GP15 = PWM slice 7, channel B */
    slice = pwm_gpio_to_slice_num(OBS_PIN_SERVO);
    cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);   /* 1 tick = 1 us */
    pwm_config_set_wrap(&cfg, 19999U);                                         /* 20 ms = 50 Hz */
    pwm_init(slice, &cfg, true);

    cflg.exinf = NULL;                                     /* VERIFY: T_CFLG fields */
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    s_flg_echo = tk_cre_flg(&cflg);
}

/**
 * obs_port_ms - milliseconds since boot, from the Pico hardware timer.
 */
uint32_t obs_port_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

/**
 * obs_port_delay_ms - sleep the calling task for ms milliseconds.
 * What it does: uses the RTOS delay so other tasks run meanwhile.
 */
void obs_port_delay_ms(uint32_t ms)
{
    (void)tk_dly_tsk((RELTIM)ms);                          /* RTOS delay, other tasks run */
}

/**
 * obs_port_servo_deg - command the servo to an angle.
 * What it does: maps 0..180 deg to a pulse between OBS_SERVO_MIN_US and
 * OBS_SERVO_MAX_US (optionally reversed) and writes it to the PWM level. Returns
 * immediately; the caller waits for the servo to arrive.
 */
void obs_port_servo_deg(uint8_t deg)
{
    uint32_t d = (OBS_SERVO_REVERSED != 0U) ? (180U - deg) : deg;
    uint32_t us = OBS_SERVO_MIN_US + ((d * (OBS_SERVO_MAX_US - OBS_SERVO_MIN_US)) / 180U);
    pwm_set_gpio_level(OBS_PIN_SERVO, (uint16_t)us);
}

/**
 * obs_port_measure - take one non-blocking ultrasonic measurement.
 * What it does:
 *  1. Waits (RTOS sleep, max 45 ms) for ECHO to be low, so a stale echo from the
 *     last ping cannot be mistaken for a new one.
 *  2. Clears the echo event flags, then sends the 10 us TRIG pulse (the only
 *     short wait).
 *  3. Waits on the RISE flag with a timeout: no rise -> NO_RESPONSE (sensor fault).
 *  4. Waits on the DONE flag with a timeout: no fall -> NO_OBJECT (beyond range).
 *  5. Otherwise returns the pulse width.
 * Params : pulse_us = pulse width output (valid only when OK).
 */
obs_echo_t obs_port_measure(uint16_t *pulse_us)
{
    UINT ptn;
    uint32_t guard = 0U;

    /* A "no object" echo can stay high ~38 ms: wait for ECHO low before a new TRIG. */
    while ((gpio_get(OBS_PIN_ECHO) != 0) && (guard < 45U)) {
        (void)tk_dly_tsk(1);
        guard++;
    }
    (void)tk_clr_flg(s_flg_echo, 0U);                      /* clear all bits; VERIFY (AND with 0) */

    gpio_put(OBS_PIN_TRIG, 1);
    busy_wait_us_32(10U);                                  /* the only short wait: 10 us TRIG */
    gpio_put(OBS_PIN_TRIG, 0);

    if (tk_wai_flg(s_flg_echo, FLG_ECHO_RISE, TWF_ORW, &ptn, (TMO)OBS_RISE_TIMEOUT_MS) != E_OK) {
        return OBS_ECHO_NO_RESPONSE;                       /* ECHO never rose */
    }
    if (tk_wai_flg(s_flg_echo, FLG_ECHO_DONE, TWF_ORW, &ptn, (TMO)OBS_ECHO_TIMEOUT_MS) != E_OK) {
        return OBS_ECHO_NO_OBJECT;                         /* still high: beyond the range window */
    }
    *pulse_us = s_pulse_us;
    return OBS_ECHO_OK;
}

/**
 * obs_port_cmd_recv - receive one command byte from the Vehicle Controller.
 * What it does: reads the command message buffer, waiting up to timeout_ms (0 =
 * poll without waiting).
 * Returns: true if a command was read.
 */
bool obs_port_cmd_recv(uint8_t *cmd, uint32_t timeout_ms)
{
    obstacle_command_message_t message;
    INT received;

    if (cmd == NULL) {
        return false;
    }

    received = tk_rcv_mbf(g_mbf_obstacle_commands, &message,
                          (TMO)timeout_ms);
    if (received != (INT)sizeof(message)) {
        return false;
    }

    switch (message.command) {
        case VEHICLE_OBSTACLE_SCAN_REQUEST:
            *cmd = (uint8_t)OBS_CMD_SCAN_REQ;
            return true;
        case VEHICLE_OBSTACLE_BYPASS_COMPLETE:
            *cmd = (uint8_t)OBS_CMD_BYPASS_DONE;
            return true;
        case VEHICLE_OBSTACLE_ABORT:
            *cmd = (uint8_t)OBS_CMD_ABORT;
            return true;
        case VEHICLE_OBSTACLE_RESET:
            *cmd = (uint8_t)OBS_CMD_RESET;
            return true;
        default:
            return false;
    }
}

/**
 * obs_port_msg_send - translate one legacy event into a typed message.
 * What it does: the kernel copies the typed value into its message buffer.
 */
void obs_port_msg_send(const obs_msg_t *m)
{
    obstacle_event_message_t obstacle;
    fault_message_t fault;
    obs_msg_type_t type;
    uint16_t distance_mm = 0U;
    INT result;

    if (m == NULL) {
        return;
    }

    type = obs_msg_type(m);
    if (type == OBS_MSG_FAULT_EVT) {
        memset(&fault, 0, sizeof(fault));
        fault.header.type = VEHICLE_MESSAGE_FAULT;
        fault.header.version = VEHICLE_MESSAGE_VERSION;
        fault.header.sequence = ++s_fault_sequence;
        fault.header.timestamp_ms = obs_port_ms();
        fault.source = VEHICLE_FAULT_SENSOR;
        fault.recoverable = 1U;
        fault.code = (uint16_t)obs_msg_unpack_fault(m);
        result = tk_snd_mbf(g_mbf_faults, &fault, (INT)sizeof(fault), TMO_POL);
    } else {
        if (type == OBS_MSG_PROXIMITY) {
            distance_mm = (uint16_t)obs_msg_unpack_proximity(m);
        } else if (type == OBS_MSG_PROFILE) {
            obs_profile_msg_t profile;
            obs_msg_unpack_profile(m, &profile);
            distance_mm = profile.closest_mm;
        } else {
            return;
        }

        memset(&obstacle, 0, sizeof(obstacle));
        obstacle.header.type = VEHICLE_MESSAGE_OBSTACLE_EVENT;
        obstacle.header.version = VEHICLE_MESSAGE_VERSION;
        obstacle.header.sequence = ++s_obstacle_sequence;
        obstacle.header.timestamp_ms = obs_port_ms();
        obstacle.distance_mm = distance_mm;
        obstacle.obstacle_class = (distance_mm != 0U) ? 1U : 0U;
        obstacle.emergency = (distance_mm != 0U && distance_mm <= 120U) ?
                             1U : 0U;
        result = tk_snd_mbf(g_mbf_obstacle_events, &obstacle,
                            (INT)sizeof(obstacle), TMO_POL);
    }

    if (result != E_OK) {
        printf("obstacle port: typed event send failed (%d)\n", (int)result);
    }
}

/**
 * obs_port_fault_set - set Buddy 5's fault bits in the team fault word.
 * What it does: shifts the bits to Buddy 5's position and ORs them in with the
 * kernel's event-flag set, which is atomic.
 */
void obs_port_fault_set(uint32_t obs_fault_bits)
{
    (void)tk_set_flg(g_flg_fault, (UINT)(obs_fault_bits << OBS_FAULT_SHIFT));   /* atomic OR in the kernel */
}

/**
 * obs_port_obstacle_flag - set or clear the 'obstacle present' status flag.
 * What it does: sets or clears one bit of the status event flag that the Safety
 * Monitor reads.
 */
void obs_port_obstacle_flag(bool present)
{
    if (present) {
        (void)tk_set_flg(g_flg_status, STATUS_OBSTACLE_PRESENT);
    } else {
        (void)tk_clr_flg(g_flg_status, (UINT)~STATUS_OBSTACLE_PRESENT);
    }
}

/**
 * obs_port_line_detected - ask Buddy 3 whether the line is under the robot.
 * What it does: calls Buddy 3's line_following_line_detected().
 */
bool obs_port_line_detected(void)
{
    return line_following_line_detected();
}

/* ---- Task entry + creation (VERIFY against the template's task setup) --- */
extern void obs_task(void);

/**
 * obs_task_entry - RTOS task entry point.
 * What it does: ignores the start code and extra info, then runs obs_task().
 */
static void obs_task_entry(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    obs_task();
}

/**
 * obs_task_create_and_start - create and start Buddy 5's task.
 * What it does: fills in the task definition (priority, stack size, entry point),
 * creates it and starts it. The team's start-up code calls this once.
 * Returns: the task ID, or a negative error code.
 */
ID obs_task_create_and_start(void)
{
    T_CTSK ctsk;
    ID id;
    ctsk.exinf = NULL;
    ctsk.tskatr = TA_HLNG | TA_RNG0;
    ctsk.task = obs_task_entry;
    ctsk.itskpri = OBS_TASK_PRI;
    ctsk.stksz = OBS_TASK_STACK_BYTES;
    id = tk_cre_tsk(&ctsk);
    if (id > 0) {
        (void)tk_sta_tsk(id, 0);
    }
    return id;
}