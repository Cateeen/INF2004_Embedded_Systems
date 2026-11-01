/*obs_scan.c - Obstacle Scan and Avoid task (State Machine)
*
* Rule for this file:
* 1. Touch the servo and ultrasonic sensor only through obs_port_*.
* 2. Never write motor outputs directly. Ask Vehicle Controller via messages
*/

#include "obs_scan.h"
#include "obs_cfg.h"
#include "obs_pack.h"
#include "obs_port.h"
#include "obs_profile.h"
#include <string.h>

typedef struct {                 // 31 bits, task-private
    uint32_t state          : 3; // 3 bits for state obs_state_t
    uint32_t prox           : 1; // 1 bit for proximity flag, Proximity sent, awaiting SCAN_REQ
    uint32_t side           : 2; // 2 bits for side, 0=left, 1=right, 2=both
    uint32_t rescans        : 2; // 2 bits for rescans, 0=none, 1=1 rescan, 2=2 rescans
    uint32_t deb            : 2; // 2 bits for debounce counter, 0=none, 1=1, 2=2, 3=3
    uint32_t lateral_mm     : 10; // 10 bits for lateral distance in mm, 0-1023mm
    uint32_t sweep_up       : 1; // 1 bit for sweep direction, 0=down, 1=up
    uint32_t fine_n         : 5; // 5 bits for fine scan count, 0-31
    uint32_t fine_lo_idx    : 5; // 5 bits for fine scan low index, 0-31
}obs_ctx_t;

typedef enum { MEAS_OK = 0, MEAS_NONE, MEAS_FAULT } meas_t;

static obs_ctx_t s_ctx;
static uint8_t   s_cur_deg;
static uint8_t   s_coarse[8];                    /* 5 x 12 bit = 60 bit */
static uint8_t   s_coarse_blocked;               /* 5-bit mask          */
static uint8_t   s_fine[(OBS_FINE_MAX_SAMPLES * OBS_DIST_BITS + 7U) / 8U];
static uint16_t  s_scan_t0, s_phase_t0;          /* 16-bit ms, wraps ~65 s */
static uint16_t  s_last_coarse_ms;
static uint16_t  s_last_scan_ms;                 /* coarse + fine */
static uint16_t  s_last_trig_ms;

/**
 * now16 - current time in milliseconds as a 16-bit number.
 * What it does: keeps only the low 16 bits (wraps every ~65 s). That is enough for
 * every timeout here and keeps the stored timestamps small.
 */
static uint16_t now16(void)
{
    return (uint16_t)obs_port_ms();
}

/**
 * since - milliseconds elapsed since an earlier now16() timestamp.
 * What it does: unsigned 16-bit subtraction, which stays correct across the
 * 65-second wrap-around.
 */
static uint16_t since(uint16_t t0)
{
    return (uint16_t)(now16() - t0);
}

/**
 * set_state - change the state machine's state.
 * What it does: stores the new state in the 3-bit context field.
 */
static void set_state(obs_state_t s)
{
    s_ctx.state = (uint32_t)s & 0x7U;
}

/**
 * send_simple - send a message that has no payload except one number.
 * What it does: packs the type and a 10-bit value and sends it to the Vehicle
 * Controller. Used for SCAN_CLEAR (coarse-scan ms) and LINE_REACQUIRED.
 */
static void send_simple(obs_msg_type_t t, uint16_t v)
{
    obs_msg_t m;
    obs_msg_pack_simple(&m, t, v);
    obs_port_msg_send(&m);
}

/**
 * raise_fault - latch a fault and stop working.
 * What it does: sets the given bits in the team fault word (atomic, through the
 * port layer), sends FAULT_EVT to the Vehicle Controller, and moves to FAULT.
 * Only a RESET command (after the self-check passes) leaves the FAULT state.
 */
static void raise_fault(uint32_t bits)
{
    obs_msg_t m;
    obs_port_fault_set(bits);                    /* atomic set in team fault word */
    obs_msg_pack_fault(&m, bits);
    obs_port_msg_send(&m);
    set_state(OBS_STATE_FAULT);
}

/**
 * to_idle - go back to IDLE and clear 'something is happening' markers.
 * What it does: clears the 'Vehicle Controller already told' marker and the
 * debounce counter, clears the obstacle flag for the Safety Monitor, sets IDLE.
 */
static void to_idle(void)
{
    s_ctx.prox = 0U;
    s_ctx.deb = 0U;
    obs_port_obstacle_flag(false);
    set_state(OBS_STATE_IDLE);
}

/**
 * abort_pending - check, without waiting, whether an ABORT command arrived.
 * What it does: drains the command queue; returns true if ABORT was among the
 * commands. Other commands are discarded. Called between measurements so a long
 * scan can be cancelled.
 */
static bool abort_pending(void)
{
    uint8_t c;
    while (obs_port_cmd_recv(&c, 0U)) {
        if (c == (uint8_t)OBS_CMD_ABORT) {
            return true;
        }
    }
    return false;
}

/**
 * servo_to - move the servo and wait until it has arrived.
 * What it does: does nothing if the servo is already at that angle. Otherwise it
 * commands the angle and sleeps for (angle change x OBS_SERVO_MS_PER_DEG +
 * settling margin). The servo gives no position feedback, so the task tracks the
 * current angle itself.
 */
static void servo_to(uint8_t deg)
{
    uint32_t delta = (deg > s_cur_deg) ? (uint32_t)(deg - s_cur_deg) : (uint32_t)(s_cur_deg - deg);
    if (delta != 0U) {
        obs_port_servo_deg(deg);
        obs_port_delay_ms((delta * OBS_SERVO_MS_PER_DEG) + OBS_SERVO_EXTRA_MS);
        s_cur_deg = deg;
    }
}

/**
 * measure_at - point the servo and take one distance reading.
 * What it does: moves the servo (servo_to), waits so that at least
 * OBS_MEASURE_GAP_MS has passed since the previous TRIG (counted from the previous
 * trigger so it overlaps the servo move), then measures. If the sensor does not
 * respond it retries up to OBS_ECHO_FAIL_LIMIT times.
 * Params : deg = servo angle, mm = distance output.
 * Returns: MEAS_OK (mm valid), MEAS_NONE (nothing within range), or MEAS_FAULT
 *          (sensor never responded: caller raises NO_ECHO).
 * Pulse width is converted to mm as us x 10 / 58 and clamped to the sensor
 * minimum range.
 */
static meas_t measure_at(uint8_t deg, uint16_t *mm)
{
    obs_echo_t e = OBS_ECHO_NO_RESPONSE;
    uint16_t pulse = 0U;
    uint8_t tries;

    servo_to(deg);
    for (tries = 0U; tries < OBS_ECHO_FAIL_LIMIT; tries++) {
        uint16_t gap = since(s_last_trig_ms);
        if (gap < OBS_MEASURE_GAP_MS) {
            obs_port_delay_ms((uint32_t)OBS_MEASURE_GAP_MS - gap);   /* also lets a stale echo end */
        }
        s_last_trig_ms = now16();
        e = obs_port_measure(&pulse);
        if (e != OBS_ECHO_NO_RESPONSE) {
            break;
        }
    }
    if (e == OBS_ECHO_NO_RESPONSE) {
        return MEAS_FAULT;
    }
    if (e == OBS_ECHO_NO_OBJECT) {
        return MEAS_NONE;
    }
    {
        uint32_t d = ((uint32_t)pulse * 10U) / 58U;              /* us/58 = cm */
        if (d < OBS_MIN_RANGE_MM) {
            d = OBS_MIN_RANGE_MM;
        }
        if (d > OBS_MAX_RANGE_MM) {
            return MEAS_NONE;
        }
        *mm = (uint16_t)d;
    }
    return MEAS_OK;
}

/**
 * store12 - save one scan result in a packed 12-bit array.
 * What it does: stores the distance in mm, or OBS_DIST_NONE (0xFFF) when nothing
 * was seen, at slot idx of the array.
 */
static void store12(uint8_t *buf, uint32_t idx, meas_t r, uint16_t mm)
{
    obs_bits_put(buf, idx * OBS_DIST_BITS, OBS_DIST_BITS,
                 (r == MEAS_OK) ? (uint32_t)mm : OBS_DIST_NONE);
}

/* ---------------- states ---------------- */

/**
 * step_idle - IDLE state: watch ahead and wait for the Vehicle Controller.
 * What it does:
 *  - Keeps the servo pointing straight ahead.
 *  - Waits up to OBS_IDLE_PING_MS for a command. SCAN_REQ starts the coarse scan,
 *    ABORT clears everything.
 *  - If PROXIMITY was already sent it only waits for SCAN_REQ, and re-arms after
 *    OBS_SCAN_WAIT_MS if the Vehicle Controller never answers.
 *  - Otherwise it pings ahead. Two consecutive readings below OBS_TRIGGER_MM set
 *    the obstacle flag (Safety Monitor) and send PROXIMITY. The robot is stopped by
 *    the Vehicle Controller, not by this task.
 *  - A sensor that never responds raises NO_ECHO.
 */
static void step_idle(void)
{
    uint8_t cmd;
    uint16_t mm = 0U;

    if (s_ctx.prox == 0U) {
        servo_to(OBS_ANGLE_AHEAD_DEG);
    }
    if (obs_port_cmd_recv(&cmd, OBS_IDLE_PING_MS)) {
        if (cmd == (uint8_t)OBS_CMD_SCAN_REQ) {
            s_scan_t0 = now16();
            set_state(OBS_STATE_COARSE);
        } else if (cmd == (uint8_t)OBS_CMD_ABORT) {
            to_idle();
        }
        return;
    }
    if (s_ctx.prox != 0U) {                      /* told the VC, waiting for SCAN_REQ */
        if (since(s_phase_t0) > OBS_SCAN_WAIT_MS) {
            to_idle();                           /* VC never answered: re-arm */
        }
        return;
    }
    {
        meas_t r = measure_at(OBS_ANGLE_AHEAD_DEG, &mm);
        if (r == MEAS_FAULT) {
            raise_fault(OBS_FAULT_NO_ECHO);
        } else if ((r == MEAS_OK) && (mm < OBS_TRIGGER_MM)) {
            s_ctx.deb = (s_ctx.deb < 3U) ? (s_ctx.deb + 1U) : 3U;
            if (s_ctx.deb >= 2U) {               /* 2 consecutive close readings */
                obs_msg_t m;
                obs_msg_pack_proximity(&m, mm);
                obs_port_obstacle_flag(true);    /* Safety Monitor input */
                obs_port_msg_send(&m);           /* VC stops the robot, then sends SCAN_REQ */
                s_ctx.prox = 1U;
                s_ctx.deb = 0U;
                s_phase_t0 = now16();
            }
        } else {
            s_ctx.deb = 0U;
        }
    }
}

/**
 * step_coarse - COARSE_SCAN state: look at five angles (30, 60, 90, 120, 150).
 * What it does: measures each angle, stores the distances and builds a 5-bit mask
 * of angles with something closer than OBS_BLOCK_MM. The sweep direction
 * alternates each scan to save servo travel. Records the duration (NFR3 evidence).
 * Next state: nothing blocked -> send SCAN_CLEAR and go IDLE; something blocked ->
 * FINE_SCAN; sensor fault -> FAULT; ABORT -> IDLE.
 */
static void step_coarse(void)
{
    uint16_t t0 = now16();
    uint8_t k;

    s_coarse_blocked = 0U;
    for (k = 0U; k < OBS_COARSE_COUNT; k++) {
        uint8_t i = (s_ctx.sweep_up != 0U) ? k : (uint8_t)(OBS_COARSE_COUNT - 1U - k);
        uint16_t mm = 0U;
        meas_t r = measure_at((uint8_t)(OBS_ANGLE_MIN_DEG + (uint32_t)i * OBS_COARSE_STEP_DEG), &mm);
        if (r == MEAS_FAULT) {
            raise_fault(OBS_FAULT_NO_ECHO);
            return;
        }
        store12(s_coarse, i, r, mm);
        if ((r == MEAS_OK) && (mm < OBS_BLOCK_MM)) {
            s_coarse_blocked |= (uint8_t)(1U << i);
        }
        if (abort_pending()) {
            to_idle();
            return;
        }
    }
    s_ctx.sweep_up = (s_ctx.sweep_up == 0U) ? 1U : 0U;        /* alternate to save servo travel */
    s_last_coarse_ms = since(t0);                              /* NFR3 evidence */

    if (s_coarse_blocked == 0U) {
        send_simple(OBS_MSG_SCAN_CLEAR, (s_last_coarse_ms > 1023U) ? 1023U : s_last_coarse_ms);
        s_ctx.rescans = 0U;
        to_idle();
    } else {
        set_state(OBS_STATE_FINE);
    }
}

/**
 * step_fine - FINE_SCAN state: sweep in 5-degree steps around the blocked region.
 * What it does: finds the first and last blocked coarse angle, widens the range by
 * one coarse step on each side (clipped to 30..150 deg), then measures every
 * OBS_FINE_STEP_DEG degrees and stores the results. Remembers the start angle and
 * sample count for the profiling step.
 * Next state: PROFILING; sensor fault -> FAULT; ABORT -> IDLE.
 */
static void step_fine(void)
{
    uint8_t first = 0U, last = 0U, i;
    uint32_t lo, hi, n, k;

    for (i = 0U; i < OBS_COARSE_COUNT; i++) {
        if ((s_coarse_blocked & (1U << i)) != 0U) {
            last = i;
        }
    }
    for (i = (uint8_t)OBS_COARSE_COUNT; i > 0U; i--) {
        if ((s_coarse_blocked & (1U << (i - 1U))) != 0U) {
            first = (uint8_t)(i - 1U);
        }
    }
    lo = OBS_ANGLE_MIN_DEG + (uint32_t)first * OBS_COARSE_STEP_DEG;
    hi = OBS_ANGLE_MIN_DEG + (uint32_t)last * OBS_COARSE_STEP_DEG;
    lo = (lo > (OBS_ANGLE_MIN_DEG + OBS_COARSE_STEP_DEG)) ? (lo - OBS_COARSE_STEP_DEG) : OBS_ANGLE_MIN_DEG;
    hi = ((hi + OBS_COARSE_STEP_DEG) < OBS_ANGLE_MAX_DEG) ? (hi + OBS_COARSE_STEP_DEG) : OBS_ANGLE_MAX_DEG;
    n = ((hi - lo) / OBS_FINE_STEP_DEG) + 1U;

    for (k = 0U; k < n; k++) {
        uint16_t mm = 0U;
        meas_t r = measure_at((uint8_t)(lo + k * OBS_FINE_STEP_DEG), &mm);
        if (r == MEAS_FAULT) {
            raise_fault(OBS_FAULT_NO_ECHO);
            return;
        }
        store12(s_fine, k, r, mm);
        if (abort_pending()) {
            to_idle();
            return;
        }
    }
    s_ctx.fine_n = n & 0x1FU;
    s_ctx.fine_lo_idx = ((lo - OBS_ANGLE_MIN_DEG) / OBS_FINE_STEP_DEG) & 0x1FU;
    set_state(OBS_STATE_PROFILING);
}

/**
 * step_profiling - PROFILING state: work out the obstacle and choose a side.
 * What it does: runs obs_profile_compute on the fine-scan data and always sends a
 * PROFILE message. Then:
 *  - nothing blocked after all (false alarm): send SCAN_CLEAR, go IDLE;
 *  - LEFT or RIGHT chosen: remember the side and lateral distance, send BYPASS_REQ,
 *    go BYPASSING;
 *  - REVERSE (no side fits): count a rescan; wait for the Vehicle Controller to
 *    back off and send SCAN_REQ again; after more than OBS_MAX_RESCANS raise
 *    NO_CLEAR_SIDE.
 */
static void step_profiling(void)
{
    obs_profile_t p;
    obs_profile_msg_t pm;
    obs_msg_t m;

    s_last_scan_ms = since(s_scan_t0);
    if (!obs_profile_compute(s_fine, (uint8_t)s_ctx.fine_n,
                             (uint8_t)(OBS_ANGLE_MIN_DEG + s_ctx.fine_lo_idx * OBS_FINE_STEP_DEG),
                             OBS_FINE_STEP_DEG, &p)) {
        send_simple(OBS_MSG_SCAN_CLEAR, (s_last_coarse_ms > 1023U) ? 1023U : s_last_coarse_ms);   /* false alarm */
        s_ctx.rescans = 0U;
        to_idle();
        return;
    }
    pm.closest_mm = p.closest_mm;
    pm.width_mm = p.width_mm;
    pm.left_mm = p.left_mm;
    pm.right_mm = p.right_mm;
    pm.action = p.action;
    pm.valid = p.valid;
    pm.scan_ms = s_last_coarse_ms;                /* coarse scan time = NFR3 evidence */
    pm.state = (uint8_t)OBS_STATE_PROFILING;
    obs_msg_pack_profile(&m, &pm);
    obs_port_msg_send(&m);

    if ((p.action == (uint8_t)OBS_ACT_LEFT) || (p.action == (uint8_t)OBS_ACT_RIGHT)) {
        s_ctx.side = (p.action == (uint8_t)OBS_ACT_LEFT) ? (uint32_t)OBS_SIDE_LEFT : (uint32_t)OBS_SIDE_RIGHT;
        s_ctx.lateral_mm = (uint32_t)p.lateral_mm & 0x3FFU;
        obs_msg_pack_bypass(&m, (uint8_t)s_ctx.side, p.lateral_mm, p.advance_mm);
        obs_port_msg_send(&m);
        s_phase_t0 = now16();
        set_state(OBS_STATE_BYPASSING);
    } else {                                     /* REVERSE: no side fits */
        s_ctx.rescans = (s_ctx.rescans < 3U) ? (s_ctx.rescans + 1U) : 3U;
        if (s_ctx.rescans > OBS_MAX_RESCANS) {
            raise_fault(OBS_FAULT_NO_CLEAR_SIDE);
        } else {
            s_ctx.prox = 1U;                     /* VC reverses, then sends SCAN_REQ */
            s_phase_t0 = now16();
            set_state(OBS_STATE_IDLE);
        }
    }
}

/**
 * step_bypassing - BYPASSING state: the Vehicle Controller drives the bypass.
 * What it does: keeps the servo straight ahead and waits for commands.
 *  - BYPASS_DONE: send LINE_SEARCH_REQ (direction back toward the line, search
 *    distance = lateral distance + margin) and go RECOVERY.
 *  - ABORT: go IDLE.
 *  - Otherwise it pings ahead: anything closer than OBS_ABORT_MM, or running past
 *    OBS_BYPASS_TIMEOUT_MS, raises BYPASS_BLOCKED; a dead sensor raises NO_ECHO.
 */
static void step_bypassing(void)
{
    uint8_t cmd;
    uint16_t mm = 0U;

    servo_to(OBS_ANGLE_AHEAD_DEG);
    if (obs_port_cmd_recv(&cmd, OBS_IDLE_PING_MS)) {
        if (cmd == (uint8_t)OBS_CMD_BYPASS_DONE) {
            obs_msg_t m;
            uint32_t search = s_ctx.lateral_mm + OBS_SEARCH_EXTRA_MM;
            uint8_t dir = (s_ctx.side == (uint32_t)OBS_SIDE_RIGHT) ? (uint8_t)OBS_SIDE_LEFT : (uint8_t)OBS_SIDE_RIGHT;
            obs_msg_pack_line_search(&m, dir, (uint16_t)((search > OBS_OFFSET_MAX_MM) ? OBS_OFFSET_MAX_MM : search));
            obs_port_msg_send(&m);               /* VC starts Buddy 3's line search */
            s_phase_t0 = now16();
            s_ctx.deb = 0U;
            set_state(OBS_STATE_RECOVERY);
        } else if (cmd == (uint8_t)OBS_CMD_ABORT) {
            to_idle();
        }
        return;
    }
    if (since(s_phase_t0) > OBS_BYPASS_TIMEOUT_MS) {
        raise_fault(OBS_FAULT_BYPASS_BLOCKED);
        return;
    }
    {
        meas_t r = measure_at(OBS_ANGLE_AHEAD_DEG, &mm);
        if (r == MEAS_FAULT) {
            raise_fault(OBS_FAULT_NO_ECHO);
        } else if ((r == MEAS_OK) && (mm < OBS_ABORT_MM)) {
            raise_fault(OBS_FAULT_BYPASS_BLOCKED);
        }
    }
}

/**
 * step_recovery - RECOVERY state: wait until the line is found again.
 * What it does: every OBS_LINE_POLL_MS it asks Buddy 3's line detection. After
 * OBS_LINE_DEBOUNCE consecutive 'line seen' results it sends LINE_REACQUIRED, clears
 * the obstacle flag and goes IDLE. If OBS_RECOVERY_TIMEOUT_MS passes it raises
 * LINE_NOT_REACQUIRED. ABORT goes to IDLE. The robot's search movement is driven
 * by the Vehicle Controller, not by this task.
 */
static void step_recovery(void)
{
    uint8_t cmd;

    if (obs_port_cmd_recv(&cmd, OBS_LINE_POLL_MS)) {
        if (cmd == (uint8_t)OBS_CMD_ABORT) {
            to_idle();
        }
        return;
    }
    if (obs_port_line_detected()) {
        s_ctx.deb = (s_ctx.deb < 3U) ? (s_ctx.deb + 1U) : 3U;
        if (s_ctx.deb >= OBS_LINE_DEBOUNCE) {
            send_simple(OBS_MSG_LINE_REACQUIRED, 0U);
            s_ctx.rescans = 0U;
            to_idle();
            return;
        }
    } else {
        s_ctx.deb = 0U;
    }
    if (since(s_phase_t0) > OBS_RECOVERY_TIMEOUT_MS) {
        raise_fault(OBS_FAULT_LINE_NOT_REACQUIRED);
    }
}

/**
 * step_fault - FAULT state: stay stopped until a RESET passes a self-check.
 * What it does: waits for the RESET command, then pings ahead as a self-check. If
 * the sensor responds it returns to IDLE; otherwise it stays in FAULT. Clearing
 * the fault bits in the team fault word is the Vehicle Controller's job.
 */
static void step_fault(void)
{
    uint8_t cmd;
    uint16_t mm = 0U;

    if (obs_port_cmd_recv(&cmd, OBS_IDLE_PING_MS) && (cmd == (uint8_t)OBS_CMD_RESET)) {
        if (measure_at(OBS_ANGLE_AHEAD_DEG, &mm) != MEAS_FAULT) {   /* self-check */
            s_ctx.rescans = 0U;
            to_idle();                           /* VC owns clearing the fault bits */
        }
    }
}

/* ---------------- public ---------------- */

/**
 * obs_init - initialise Buddy 5's task.
 * What it does: initialises the hardware through the port layer, zeroes all task
 * data, points the servo straight ahead (and waits for it to arrive), clears the
 * obstacle flag and sets the state to IDLE.
 */
void obs_init(void)
{
    obs_port_init();
    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    (void)memset(s_coarse, 0, sizeof(s_coarse));
    (void)memset(s_fine, 0, sizeof(s_fine));
    s_coarse_blocked = 0U;
    s_cur_deg = OBS_ANGLE_AHEAD_DEG;
    s_last_coarse_ms = 0U;
    s_last_scan_ms = 0U;
    s_last_trig_ms = 0U;
    obs_port_servo_deg(OBS_ANGLE_AHEAD_DEG);
    obs_port_delay_ms(300U);
    obs_port_obstacle_flag(false);
    set_state(OBS_STATE_IDLE);
}

/**
 * obs_step - run one pass of the state machine.
 * What it does: calls the handler for the current state. Each handler does a
 * bounded amount of work with timeouts, so this never blocks forever.
 */
void obs_step(void)
{
    switch ((obs_state_t)s_ctx.state) {
    case OBS_STATE_IDLE:      step_idle();      break;
    case OBS_STATE_COARSE:    step_coarse();    break;
    case OBS_STATE_FINE:      step_fine();      break;
    case OBS_STATE_PROFILING: step_profiling(); break;
    case OBS_STATE_BYPASSING: step_bypassing(); break;
    case OBS_STATE_RECOVERY:  step_recovery();  break;
    default:                  step_fault();     break;
    }
}

/**
 * obs_task - the RTOS task body.
 * What it does: initialises, then calls obs_step() forever. Created by
 * obs_task_create_and_start() in the port layer.
 */
void obs_task(void)
{
    obs_init();
    for (;;) {
        obs_step();
    }
}

/**
 * obs_get_state - return the current state (obs_state_t).
 * Used for logging, telemetry and the host test.
 */
uint8_t obs_get_state(void)
{
    return (uint8_t)s_ctx.state;
}

/**
 * obs_get_last_coarse_ms - duration of the last coarse scan in ms.
 * Evidence for NFR3 (coarse scan within 500 ms): log or publish this value.
 */
uint16_t obs_get_last_coarse_ms(void)
{
    return s_last_coarse_ms;
}

/**
 * obs_get_last_scan_ms - duration of the last coarse plus fine scan in ms.
 * Part of the bypass-time metric (rubric: under 3 s for the top grade).
 */
uint16_t obs_get_last_scan_ms(void)
{
    return s_last_scan_ms;
}