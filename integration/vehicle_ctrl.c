/*
 * vehicle_ctrl.c - Vehicle Controller: the link between obs_scan.c and motor_driver.c
 *
 *   obs task  --PROXIMITY-------->  stop, wait, send SCAN_REQ
 *             --PROFILE(REVERSE)->  back up, send SCAN_REQ (rescan)
 *             --BYPASS_REQ------->  turn, drive lateral, turn back, drive advance, send BYPASS_DONE
 *             --LINE_SEARCH_REQ-->  turn toward line, drive slowly until line seen,
 *                                   wait for LINE_REACQUIRED, turn back
 *             --SCAN_CLEAR / LINE_REACQUIRED --> go back to cruising / line following
 *             --FAULT_EVT-------->  stop and stay stopped
 *
 * Only this task ever calls motor_*(). Movement is open loop (time based):
 * calibrate the VC_* numbers on the real robot.
 * Transport and the bench "go" switch live in car_link.h, so this file has no
 * RTOS or Pico SDK dependency.
 *
 * STATUS: host syntax-checked only. NOT run on the robot.
 */
#include "vehicle_ctrl.h"
#include "car_link.h"
#include "motor_driver.h"
#include "obs_cfg.h"
#include "obs_pack.h"
#include "obs_port.h"
#include "obs_scan.h"

/* 1 = this file cruises straight ahead whenever line following would be active (bench test). 0 = only call the hook. */
#ifndef VC_BUILTIN_CRUISE
#define VC_BUILTIN_CRUISE     1
#endif

/* ---- Wiring (PLACEHOLDER: check on the robot) ----------------------------- */
#define VC_LEFT_MOTOR         MOTOR_1
#define VC_RIGHT_MOTOR        MOTOR_2
#define VC_LEFT_SIGN          (1)       /* -1 if that motor turns the wrong way */
#define VC_RIGHT_SIGN         (-1)

/* ---- Motion calibration (PLACEHOLDERS: measure them; start slow!) ---------- */
#define VC_CRUISE_PCT         30
#define VC_TURN_PCT           35
#define VC_REVERSE_PCT        30
#define VC_SEARCH_PCT         25        /* slow, so the line is still under the sensor when we brake */
#define VC_MM_PER_S_FWD       150U      /* measured speed at VC_CRUISE_PCT */
#define VC_MM_PER_S_REV       140U
#define VC_MM_PER_S_SEARCH    110U
#define VC_TURN_MS_PER_90     450U      /* time to spin 90 deg at VC_TURN_PCT */
#define VC_REVERSE_MM         150U

/* ---- Timing ---------------------------------------------------------------- */
#define VC_TICK_MS            10U       /* commands are repeated every tick (reversal rule in motor_driver) */
#define VC_SETTLE_MS          80U       /* > MOTOR_REVERSE_BRAKE_US (50 ms) + coasting */
#define VC_REACQ_WAIT_MS      400U      /* wait for LINE_REACQUIRED after the line search */

typedef enum { RUN_DONE = 0, RUN_STOPPED_BY_COND, RUN_ABORTED } run_t;
typedef bool (*stop_fn_t)(void);

static bool s_following = true;         /* true = "line follower / cruise owns the motors" */
static bool s_prev_run  = false;

__attribute__((weak)) void vc_line_follow_enable(bool on)
{
    (void)on;
}

/* set_following - hand the motors to / take them from the line follower. */
static void set_following(bool on)
{
    s_following = on;
    vc_line_follow_enable(on);
}

/* ---- Motion primitives ------------------------------------------------------ */

static void set_lr(int left, int right)
{
    motor_set_speed(VC_LEFT_MOTOR,  (int16_t)(VC_LEFT_SIGN  * left));
    motor_set_speed(VC_RIGHT_MOTOR, (int16_t)(VC_RIGHT_SIGN * right));
}

/* settle - brake and wait so the robot is (nearly) still before the next move. */
static void settle(void)
{
    motor_brake_all();
    obs_port_delay_ms(VC_SETTLE_MS);
}

/*
 * run - hold a left/right command for ms milliseconds, re-issuing it every tick.
 * RUN_STOPPED_BY_COND: stop() returned true.
 * RUN_ABORTED: emergency stop, the obstacle task is in FAULT, or the bench "go" switch was turned off.
 */
static run_t run(int left, int right, uint32_t ms, stop_fn_t stop)
{
    uint32_t t0 = obs_port_ms();

    while ((uint32_t)(obs_port_ms() - t0) < ms) {
        if (motor_is_emergency_stopped() ||
            (obs_get_state() == (uint8_t)OBS_STATE_FAULT) ||
            !vc_port_run_enabled()) {
            motor_brake_all();
            return RUN_ABORTED;
        }
        if ((stop != 0) && stop()) {
            return RUN_STOPPED_BY_COND;
        }
        set_lr(left, right);
        obs_port_delay_ms(VC_TICK_MS);
    }
    return RUN_DONE;
}

static run_t drive_fwd(uint32_t mm)
{
    return run(VC_CRUISE_PCT, VC_CRUISE_PCT, (mm * 1000U) / VC_MM_PER_S_FWD, 0);
}

static run_t drive_rev(uint32_t mm)
{
    return run(-VC_REVERSE_PCT, -VC_REVERSE_PCT, (mm * 1000U) / VC_MM_PER_S_REV, 0);
}

static run_t turn(uint8_t side, uint32_t deg)
{
    uint32_t ms = (deg * VC_TURN_MS_PER_90) / 90U;

    if (side == (uint8_t)OBS_SIDE_LEFT) {
        return run(-VC_TURN_PCT, VC_TURN_PCT, ms, 0);      /* left wheel back, right wheel forward */
    }
    return run(VC_TURN_PCT, -VC_TURN_PCT, ms, 0);
}

static uint8_t opposite(uint8_t side)
{
    return (side == (uint8_t)OBS_SIDE_LEFT) ? (uint8_t)OBS_SIDE_RIGHT : (uint8_t)OBS_SIDE_LEFT;
}

/* give_up - a manoeuvre was cut short: stop, tell the obstacle task, stay stopped. */
static void give_up(void)
{
    motor_brake_all();
    set_following(false);
    vc_port_cmd_send((uint8_t)OBS_CMD_ABORT);      /* ignored by the obstacle task if it is already in FAULT */
}

/* wait_for - wait up to ms for one message type; false on timeout or if the obstacle task faults. */
static bool wait_for(obs_msg_type_t type, uint32_t ms)
{
    uint32_t t0 = obs_port_ms();
    obs_msg_t m;

    while ((uint32_t)(obs_port_ms() - t0) < ms) {
        if (obs_get_state() == (uint8_t)OBS_STATE_FAULT) {
            return false;
        }
        if (vc_port_msg_recv(&m, VC_TICK_MS) && (obs_msg_type(&m) == type)) {
            return true;
        }
    }
    return false;
}

/* ---- Manoeuvres --------------------------------------------------------------- */

/* do_bypass - turn toward the free side, go out lateral_mm, turn back, go forward advance_mm. */
static void do_bypass(uint8_t side, uint16_t lateral_mm, uint16_t advance_mm)
{
    settle();
    if (turn(side, 90U) != RUN_DONE)             { give_up(); return; }
    settle();
    if (drive_fwd(lateral_mm) != RUN_DONE)       { give_up(); return; }
    settle();
    if (turn(opposite(side), 90U) != RUN_DONE)   { give_up(); return; }
    settle();
    if (drive_fwd(advance_mm) != RUN_DONE)       { give_up(); return; }
    settle();
    vc_port_cmd_send((uint8_t)OBS_CMD_BYPASS_DONE);
}

/* do_line_search - turn toward the line, creep until the line is seen, confirm, realign. */
static void do_line_search(uint8_t dir_side, uint16_t max_mm)
{
    run_t r;

    settle();
    if (turn(dir_side, 90U) != RUN_DONE)         { give_up(); return; }
    settle();
    r = run(VC_SEARCH_PCT, VC_SEARCH_PCT, (max_mm * 1000U) / VC_MM_PER_S_SEARCH, obs_port_line_detected);
    if (r == RUN_ABORTED)                        { give_up(); return; }
    motor_brake_all();                           /* line seen (or distance used up): stop right here */
    if (r == RUN_STOPPED_BY_COND) {
        /* the obstacle task debounces the line for ~60 ms: let it confirm before we move away */
        if (wait_for(OBS_MSG_LINE_REACQUIRED, VC_REACQ_WAIT_MS)) {
            settle();
            if (turn(opposite(dir_side), 90U) != RUN_DONE) { give_up(); return; }
            settle();
            set_following(true);
        }
        /* else: the obstacle task will raise LINE_NOT_REACQUIRED by itself */
    }
    /* r == RUN_DONE: line never found, stay stopped; the obstacle task times out into FAULT */
}

/* do_backoff - no side fits: back up, then ask for another scan. */
static void do_backoff(void)
{
    settle();
    if (drive_rev(VC_REVERSE_MM) != RUN_DONE)    { give_up(); return; }
    settle();
    vc_port_cmd_send((uint8_t)OBS_CMD_SCAN_REQ);
}

/* ---- Task ---------------------------------------------------------------------- */

static void handle(const obs_msg_t *m)
{
    switch (obs_msg_type(m)) {

    case OBS_MSG_PROXIMITY:                      /* STOP, then ask for a scan */
        set_following(false);
        settle();
        vc_port_cmd_send((uint8_t)OBS_CMD_SCAN_REQ);
        break;

    case OBS_MSG_PROFILE: {                      /* LEFT/RIGHT is followed by BYPASS_REQ; REVERSE is handled here */
        obs_profile_msg_t p;
        obs_msg_unpack_profile(m, &p);
        if (p.action == (uint8_t)OBS_ACT_REVERSE) {
            do_backoff();
        }
        break;
    }

    case OBS_MSG_BYPASS_REQ: {
        uint8_t side;
        uint16_t lat, adv;
        obs_msg_unpack_bypass(m, &side, &lat, &adv);
        do_bypass(side, lat, adv);
        break;
    }

    case OBS_MSG_LINE_SEARCH_REQ: {
        uint8_t dir;
        uint16_t mx;
        obs_msg_unpack_line_search(m, &dir, &mx);
        do_line_search(dir, mx);
        break;
    }

    case OBS_MSG_SCAN_CLEAR:                     /* false alarm: carry on */
    case OBS_MSG_LINE_REACQUIRED:
        set_following(true);
        break;

    case OBS_MSG_FAULT_EVT:                      /* stay stopped (use motor_emergency_stop() for a latch) */
        motor_emergency_stop();    
        set_following(false);
        break;

    default:
        break;
    }
}

void vc_task(void)
{
    obs_msg_t m;

    motor_driver_init();
    motor_clear_emergency_stop();                /* start-up only; the Safety Monitor should own this later */
    s_following = true;
    s_prev_run = false;

    for (;;) {
        bool have = vc_port_msg_recv(&m, 2U * VC_TICK_MS);
        bool go = vc_port_run_enabled();

        if (go && !s_prev_run) {
            if (obs_get_state() != (uint8_t)OBS_STATE_FAULT) {
                motor_clear_emergency_stop();
            }
            set_following(true);                 /* (re)start from the bench "go" key */
        }
        s_prev_run = go;

        if (have) {
            handle(&m);
        } else if (!go) {
            motor_brake_all();
        } else if (VC_BUILTIN_CRUISE && s_following) {
            set_lr(VC_CRUISE_PCT, VC_CRUISE_PCT);
        }
    }
}