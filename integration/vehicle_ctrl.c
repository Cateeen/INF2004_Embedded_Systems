/*
 * vehicle_ctrl.c - PicoCar Vehicle Controller (Core 1)
 *
 * Buddy 2: motor driver
 * Buddy 3: IR line following and steering adapter
 * Buddy 5: obstacle events and line-recovery coordination
 *
 * NOTE: Software-integration checkpoint only. Open-loop timing and
 * cross-core fault/queue handling still require review and hardware tests.
 */

#include "vehicle_ctrl.h"
#include "car_link.h"
#include "motor_driver.h"
#include "obs_cfg.h"
#include "obs_pack.h"
#include "obs_port.h"
#include "obs_scan.h"
#include "buddy3.h"
#include "line_steering.h"
#include "line_status.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Physical wiring polarity: verify on the actual robot. */
#define VC_LEFT_MOTOR      MOTOR_1
#define VC_RIGHT_MOTOR     MOTOR_2
#define VC_LEFT_SIGN       (1)
#define VC_RIGHT_SIGN      (-1)

/* Open-loop calibration placeholders: NOT measured on the robot. */
#define VC_CRUISE_PCT      30
#define VC_TURN_PCT        35
#define VC_REVERSE_PCT     30
#define VC_SEARCH_PCT      25
#define VC_MM_PER_S_FWD    150U
#define VC_MM_PER_S_REV    140U
#define VC_MM_PER_S_SEARCH 110U
#define VC_TURN_MS_PER_90  450U
#define VC_REVERSE_MM      150U

#define VC_TICK_MS         10U
#define VC_SETTLE_MS       80U
#define VC_REACQ_WAIT_MS   400U

typedef enum {
    RUN_DONE = 0,
    RUN_STOPPED_BY_COND,
    RUN_ABORTED
} run_t;

typedef bool (*stop_fn_t)(void);

static bool s_following = true;
static bool s_prev_run = false;
static line_steering_t s_line_steering;

/*
 * Shared safety interlock.
 * Fault bits are latched by Buddy 5 and read safely from Core 1.
 */
static bool car_motion_allowed(void)
{
    return vc_port_run_enabled() &&
           !motor_is_emergency_stopped() &&
           car_fault_bits_get() == 0U;
}

__attribute__((weak)) void vc_line_follow_enable(bool on)
{
    (void)on;
}

static void set_following(bool on)
{
    s_following = on;
    vc_line_follow_enable(on);
}

/* Map vehicle-relative left/right motor demands to signed PWM outputs. */
static void set_lr(int left, int right)
{
    motor_set_speed(VC_LEFT_MOTOR, (int16_t)(VC_LEFT_SIGN * left));
    motor_set_speed(VC_RIGHT_MOTOR, (int16_t)(VC_RIGHT_SIGN * right));
}

/* Only Core 1 accesses Buddy 3's ADC hardware. */
static bool sample_line_status(uint8_t *position)
{
    ir_readings_t readings = {0};

    if (!line_following_read(&readings)) {
        line_status_publish(false, obs_port_ms());
        motor_emergency_stop();
        set_following(false);
        return false;
    }

    uint8_t detected_position = line_following_position(&readings);
    line_status_publish(detected_position != POS_LOST, obs_port_ms());

    if (position != NULL) {
        *position = detected_position;
    }

    return true;
}

static void update_line_following(void)
{
    if (!s_following || !car_motion_allowed() || car_obstacle_present()){
        motor_brake_all();
        return;
    }

    uint8_t position = POS_LOST;
    if (!sample_line_status(&position)) {
        return;
    }

    line_steering_command_t command =
        line_steering_update(&s_line_steering, position, obs_port_ms());

    if (command.stop_latched) {
        motor_brake_all();
        return;
    }

    /* Adapter values are already signed (M1 positive, M2 negative). */
    motor_set_speed(MOTOR_1, command.motor_1_pwm);
    motor_set_speed(MOTOR_2, command.motor_2_pwm);
}

/* Return false if braking/settling was interrupted. */
static bool settle(void)
{
    uint32_t start_ms = obs_port_ms();

    motor_brake_all();

    while ((uint32_t)(obs_port_ms() - start_ms) < VC_SETTLE_MS) {
        if (!car_motion_allowed()) {
            motor_brake_all();
            return false;
        }

        if (!sample_line_status(NULL)) {
            motor_brake_all();
            return false;
        }

        obs_port_delay_ms(VC_TICK_MS);
    }

    if (!car_motion_allowed()) {
        motor_brake_all();
        return false;
    }

    return true;
}

/* Blocking, time-based motor movement; sensor snapshots stay fresh. */
static run_t run(int left, int right, uint32_t ms, stop_fn_t stop)
{
    uint32_t t0 = obs_port_ms();

    while ((uint32_t)(obs_port_ms() - t0) < ms) {
        if (!car_motion_allowed()) {
            motor_brake_all();
            return RUN_ABORTED;
        }

        if (!sample_line_status(NULL)) {
            motor_brake_all();
            return RUN_ABORTED;
        }

        if (stop != NULL && stop()) {
            motor_brake_all();
            return RUN_STOPPED_BY_COND;
        }

        set_lr(left, right);
        obs_port_delay_ms(VC_TICK_MS);
    }

    return RUN_DONE;
}

static run_t drive_fwd(uint32_t mm)
{
    return run(VC_CRUISE_PCT, VC_CRUISE_PCT,
               (mm * 1000U) / VC_MM_PER_S_FWD, NULL);
}

static run_t drive_rev(uint32_t mm)
{
    return run(-VC_REVERSE_PCT, -VC_REVERSE_PCT,
               (mm * 1000U) / VC_MM_PER_S_REV, NULL);
}

static run_t turn(uint8_t side, uint32_t deg)
{
    uint32_t ms = (deg * VC_TURN_MS_PER_90) / 90U;

    if (side == (uint8_t)OBS_SIDE_LEFT) {
        return run(-VC_TURN_PCT, VC_TURN_PCT, ms, NULL);
    }

    return run(VC_TURN_PCT, -VC_TURN_PCT, ms, NULL);
}

static uint8_t opposite(uint8_t side)
{
    return side == (uint8_t)OBS_SIDE_LEFT ?
        (uint8_t)OBS_SIDE_RIGHT : (uint8_t)OBS_SIDE_LEFT;
}

/* Controlled cancellation; delivery of ABORT is not yet guaranteed. */
static void give_up(void)
{
    motor_brake_all();
    set_following(false);
    vc_port_cmd_send((uint8_t)OBS_CMD_ABORT);
}

/* Wait for a particular event without silently ignoring critical events. */
static bool wait_for(obs_msg_type_t type, uint32_t ms)
{
    uint32_t t0 = obs_port_ms();
    obs_msg_t m;

    while ((uint32_t)(obs_port_ms() - t0) < ms) {
        if (!car_motion_allowed()) {
            return false;
        }

        if (!sample_line_status(NULL)) {
            return false;
        }

        if (vc_port_msg_recv(&m, VC_TICK_MS)) {
            obs_msg_type_t received = obs_msg_type(&m);

            if (received == OBS_MSG_FAULT_EVT ||
                received == OBS_MSG_PROXIMITY ||
                received == OBS_MSG_BYPASS_REQ ||
                received == OBS_MSG_LINE_SEARCH_REQ) {
                motor_emergency_stop();
                set_following(false);
                return false;
            }

            if (received == type) {
                return true;
            }

            /* Other message types are ignored at this checkpoint. */
        }
    }

    return false;
}

static void do_bypass(uint8_t side, uint16_t lateral_mm,
                      uint16_t advance_mm)
{
    if (!settle()) { give_up(); return; }
    if (turn(side, 90U) != RUN_DONE) { give_up(); return; }
    if (!settle()) { give_up(); return; }
    if (drive_fwd(lateral_mm) != RUN_DONE) { give_up(); return; }
    if (!settle()) { give_up(); return; }
    if (turn(opposite(side), 90U) != RUN_DONE) { give_up(); return; }
    if (!settle()) { give_up(); return; }
    if (drive_fwd(advance_mm) != RUN_DONE) { give_up(); return; }
    if (!settle()) { give_up(); return; }

    vc_port_cmd_send((uint8_t)OBS_CMD_BYPASS_DONE);
}

static void do_line_search(uint8_t dir_side, uint16_t max_mm)
{
    if (!settle()) { give_up(); return; }
    if (turn(dir_side, 90U) != RUN_DONE) { give_up(); return; }
    if (!settle()) { give_up(); return; }

    run_t r = run(VC_SEARCH_PCT, VC_SEARCH_PCT,
                  (max_mm * 1000U) / VC_MM_PER_S_SEARCH,
                  obs_port_line_detected);

    if (r == RUN_ABORTED) {
        give_up();
        return;
    }

    motor_brake_all();

    if (r == RUN_STOPPED_BY_COND) {
        if (wait_for(OBS_MSG_LINE_REACQUIRED, VC_REACQ_WAIT_MS)) {
            if (!settle()) { give_up(); return; }
            if (turn(opposite(dir_side), 90U) != RUN_DONE) {
                give_up();
                return;
            }
            if (!settle()) { give_up(); return; }

            if (!s_line_steering.stop_latched &&
            car_motion_allowed() && !car_obstacle_present()) {
                set_following(true);
            } else {
                motor_brake_all();
                set_following(false);
            }
        } else {
            /* Failed confirmation, STOP or fault: cancel recovery. */
            give_up();
            return;
        }
    } else {
        /* RUN_DONE: search distance exhausted without the line. */
        give_up();
        return;
    }
}

static void do_backoff(void)
{
    if (!settle()) { give_up(); return; }
    if (drive_rev(VC_REVERSE_MM) != RUN_DONE) { give_up(); return; }
    if (!settle()) { give_up(); return; }

    vc_port_cmd_send((uint8_t)OBS_CMD_SCAN_REQ);
}

static void handle(const obs_msg_t *m)
{
    switch (obs_msg_type(m)) {
    case OBS_MSG_PROXIMITY:
        set_following(false);
        if (!settle()) { give_up(); return; }
        vc_port_cmd_send((uint8_t)OBS_CMD_SCAN_REQ);
        break;

    case OBS_MSG_PROFILE: {
        obs_profile_msg_t p;
        obs_msg_unpack_profile(m, &p);
        if (p.action == (uint8_t)OBS_ACT_REVERSE) {
            do_backoff();
        }
        break;
    }

    case OBS_MSG_BYPASS_REQ: {
        uint8_t side;
        uint16_t lat;
        uint16_t adv;
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

    case OBS_MSG_SCAN_CLEAR:
    case OBS_MSG_LINE_REACQUIRED:
        if (!s_line_steering.stop_latched &&
            car_motion_allowed() && !car_obstacle_present()) {
            set_following(true);
        } else {
            motor_brake_all();
            set_following(false);
        }
        break;

    case OBS_MSG_FAULT_EVT:
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
    motor_clear_emergency_stop();

    line_following_init();
    line_steering_reset(&s_line_steering);

    s_following = true;
    s_prev_run = false;

    for (;;) {
        bool have = vc_port_msg_recv(&m, 2U * VC_TICK_MS);
        bool go = vc_port_run_enabled();

        if (go && !s_prev_run) {
            if (car_fault_bits_get() == 0U &&
                !motor_is_emergency_stopped() &&
                !car_obstacle_present()) {
                line_steering_reset(&s_line_steering);
                set_following(true);
            }
        }

        s_prev_run = go;

        if (car_fault_bits_get() != 0U) {
            motor_emergency_stop();
            set_following(false);
        } else if (!go) {
            motor_brake_all();
        } else if (motor_is_emergency_stopped()) {
            motor_brake_all();
        } else if (have) {
            handle(&m);
        } else if (s_following) {
            update_line_following();
        }
    }
}
