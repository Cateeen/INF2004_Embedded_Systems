/*
 * obs_port_pico_car.c - obs_port.h for the real car on a Pico W, NO RTOS.
 *
 * Runs on obs_core (core 0) together with obs_scan.c. The Vehicle Controller runs on vc_core (core 1)
 * (vehicle_ctrl.c) and talks to this file through two thread-safe queues:
 *     obs task -> VC : obs_msg_t  (obs_port_msg_send  -> vc_port_msg_recv)
 *     VC -> obs task : 1-byte cmd (vc_port_cmd_send   -> obs_port_cmd_recv)
 * Everything that passes is printed on USB serial, so you can watch the conversation.
 *
 * ECHO is polled (busy-wait) like obs_port_pico_bare.c: simple and good enough for bring-up.
 *
 * USB-serial keys (handled inside obs_port_cmd_recv, so obs_core must be running obs_step):
 *   g = GO (motors allowed to move / restart)   x = STOP (motors braked, manoeuvres aborted)
 *   l = simulate "line detected" for 1 s (until Buddy 3's sensor is wired)
 *   a = send ABORT to the obstacle task         r = send RESET (leave FAULT)     h = help
 *
 * STATUS: written but NOT compiled or run by the author. Pico SDK 1.5.x APIs.
 */
#include <stdio.h>
#include "obs_port.h"
#include "obs_cfg.h"
#include "car_link.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "line_status.h"

/* Optional real line sensor (TCRT5000 D0). Define CAR_LINE_PIN (any free GPIO) in CMake to use it.
 * CAR_LINE_ACTIVE_LEVEL = pin level that means "line under sensor" (check your module: often 0 = dark/black). */
#ifndef CAR_LINE_ACTIVE_LEVEL
#define CAR_LINE_ACTIVE_LEVEL 0
#endif

#define STALE_ECHO_WAIT_US   45000U     /* a "no object" echo can stay high ~38 ms */
#define LINE_PULSE_MS        1000U
#define QUEUE_DEPTH          8U

static queue_t s_q_to_vc;               /* obs_msg_t */
static queue_t s_q_to_obs;              /* uint8_t   */
static volatile bool     s_run = false;
static volatile uint32_t s_line_until_ms = 0U;

static const char *k_type[8]   = { "NONE", "PROXIMITY", "PROFILE", "BYPASS_REQ",
                                   "LINE_SEARCH_REQ", "LINE_REACQUIRED", "SCAN_CLEAR", "FAULT_EVT" };
static const char *k_action[5] = { "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE" };
static const char *k_side[3]   = { "NONE", "LEFT", "RIGHT" };
static const char *k_cmd[5]    = { "NONE", "SCAN_REQ", "BYPASS_DONE", "ABORT", "RESET" };

static void ts(void)
{
    printf("[%7lu ms] ", (unsigned long)to_ms_since_boot(get_absolute_time()));
}

/* ---------------- link to the Vehicle Controller (vc_core) ---------------- */

void car_link_init(void)
{
    queue_init(&s_q_to_vc, sizeof(obs_msg_t), QUEUE_DEPTH);
    queue_init(&s_q_to_obs, 1U, QUEUE_DEPTH);
}

bool vc_port_msg_recv(obs_msg_t *m, uint32_t timeout_ms)
{
    absolute_time_t end = make_timeout_time_ms(timeout_ms);

    do {
        if (queue_try_remove(&s_q_to_vc, m)) {
            return true;
        }
        sleep_ms(1);
    } while (!time_reached(end));
    return false;
}

void vc_port_cmd_send(uint8_t cmd)
{
    (void)queue_try_add(&s_q_to_obs, &cmd);
}

bool vc_port_run_enabled(void)
{
    return s_run;
}

/* ---------------- obs_port.h ---------------- */

void obs_port_init(void)
{
    uint slice;
    pwm_config cfg;

    gpio_init(OBS_PIN_TRIG);
    gpio_set_dir(OBS_PIN_TRIG, GPIO_OUT);
    gpio_put(OBS_PIN_TRIG, 0);

    gpio_init(OBS_PIN_ECHO);
    gpio_set_dir(OBS_PIN_ECHO, GPIO_IN);
    gpio_pull_down(OBS_PIN_ECHO);

    gpio_set_function(OBS_PIN_SERVO, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(OBS_PIN_SERVO);
    cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);   /* 1 tick = 1 us */
    pwm_config_set_wrap(&cfg, 19999U);                                         /* 20 ms = 50 Hz */
    pwm_init(slice, &cfg, true);

#ifdef CAR_LINE_PIN
    gpio_init(CAR_LINE_PIN);
    gpio_set_dir(CAR_LINE_PIN, GPIO_IN);
#endif
}

uint32_t obs_port_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void obs_port_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

void obs_port_servo_deg(uint8_t deg)
{
    uint32_t d = (OBS_SERVO_REVERSED != 0U) ? (180U - deg) : deg;
    uint32_t us = OBS_SERVO_MIN_US + ((d * (OBS_SERVO_MAX_US - OBS_SERVO_MIN_US)) / 180U);
    pwm_set_gpio_level(OBS_PIN_SERVO, (uint16_t)us);
}

/* obs_port_measure - one blocking, polled HC-SR04 measurement. */
obs_echo_t obs_port_measure(uint16_t *pulse_us)
{
    uint32_t t0 = time_us_32();
    uint32_t t_rise, w;

    while (gpio_get(OBS_PIN_ECHO) != 0) {                 /* let a stale echo end */
        if ((time_us_32() - t0) > STALE_ECHO_WAIT_US) {
            return OBS_ECHO_NO_RESPONSE;
        }
    }
    gpio_put(OBS_PIN_TRIG, 1);
    busy_wait_us_32(10U);
    gpio_put(OBS_PIN_TRIG, 0);

    t0 = time_us_32();
    while (gpio_get(OBS_PIN_ECHO) == 0) {
        if ((time_us_32() - t0) > (OBS_RISE_TIMEOUT_MS * 1000U)) {
            return OBS_ECHO_NO_RESPONSE;
        }
    }
    t_rise = time_us_32();
    while (gpio_get(OBS_PIN_ECHO) != 0) {
        if ((time_us_32() - t_rise) > (OBS_ECHO_TIMEOUT_MS * 1000U)) {
            return OBS_ECHO_NO_OBJECT;
        }
    }
    w = time_us_32() - t_rise;
    *pulse_us = (uint16_t)w;
    return OBS_ECHO_OK;
}

static void help(void)
{
    printf("keys: g=GO  x=STOP  l=fake line (1 s) s=Scan  a=ABORT  r=RESET  h=help\n");
}

/* obs_port_cmd_recv - commands from the Vehicle Controller queue; also polls the USB keys. */
bool obs_port_cmd_recv(uint8_t *cmd, uint32_t timeout_ms)
{
    absolute_time_t end = make_timeout_time_ms(timeout_ms);

    do {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            switch (c) {
            case 'g': s_run = true;  ts(); printf("[KEY] GO\n");   break;
            case 'x': s_run = false; ts(); printf("[KEY] STOP\n"); break;
            case 'l': s_line_until_ms = obs_port_ms() + LINE_PULSE_MS; ts(); printf("[KEY] fake line for 1 s\n"); break;
            case 's': ts(); printf("[KEY] SCAN\n"); *cmd = (uint8_t)OBS_CMD_SCAN_REQ; return true;
            case 'a': ts(); printf("[KEY] ABORT\n"); *cmd = (uint8_t)OBS_CMD_ABORT; return true;
            case 'r': ts(); printf("[KEY] RESET\n"); *cmd = (uint8_t)OBS_CMD_RESET; return true;
            case 'h': help(); break;
            default: break;
            }
        }
        if (queue_try_remove(&s_q_to_obs, cmd)) {
            ts();
            printf("[VC -> OBS] %s\n", (*cmd < 5U) ? k_cmd[*cmd] : "?");
            return true;
        }
        sleep_ms(1);
    } while (!time_reached(end));
    return false;
}

/* obs_port_msg_send - queue the message for the Vehicle Controller and print it. */
void obs_port_msg_send(const obs_msg_t *m)
{
    unsigned t = (unsigned)obs_msg_type(m);

    if (!queue_try_add(&s_q_to_vc, m)) {
        ts(); printf("[WARN] queue to VC full, message dropped\n");
    }
    ts();
    printf("[OBS -> VC] %s", (t < 8U) ? k_type[t] : "?");
    if (t == (unsigned)OBS_MSG_PROXIMITY) {
        uint8_t b[8];
        obs_msg_to_bytes(m, b);
        printf("  closest=%lu mm", (unsigned long)obs_bits_get(b, 0U, 12U));
    } else if (t == (unsigned)OBS_MSG_PROFILE) {
        obs_profile_msg_t p;
        obs_msg_unpack_profile(m, &p);
        printf("  closest=%u width=%u left=%u right=%u action=%s",
               (unsigned)p.closest_mm, (unsigned)p.width_mm, (unsigned)p.left_mm, (unsigned)p.right_mm,
               (p.action < 5U) ? k_action[p.action] : "?");
    } else if (t == (unsigned)OBS_MSG_BYPASS_REQ) {
        uint8_t side; uint16_t lat, adv;
        obs_msg_unpack_bypass(m, &side, &lat, &adv);
        printf("  side=%s lateral=%u advance=%u", (side < 3U) ? k_side[side] : "?", (unsigned)lat, (unsigned)adv);
    } else if (t == (unsigned)OBS_MSG_LINE_SEARCH_REQ) {
        uint8_t dir; uint16_t mx;
        obs_msg_unpack_line_search(m, &dir, &mx);
        printf("  turn=%s max_search=%u", (dir < 3U) ? k_side[dir] : "?", (unsigned)mx);
    } else if (t == (unsigned)OBS_MSG_FAULT_EVT) {
        printf("  bits=0x%lX", (unsigned long)obs_msg_unpack_fault(m));
    }
    printf("\n");
}

void obs_port_fault_set(uint32_t bits)
{
    ts();
    printf("[FAULT] bits=0x%lX (1=NO_ECHO 2=NO_CLEAR_SIDE 4=BYPASS_BLOCKED 8=LINE_NOT_REACQUIRED)\n",
           (unsigned long)bits);
}

void obs_port_obstacle_flag(bool present)
{
    ts();
    printf("[SAFETY FLAG] obstacle present = %s\n", present ? "YES" : "no");
}

bool obs_port_line_detected(void)
{
    uint32_t now_ms = obs_port_ms();

    /*
     * Real Buddy 3 line status.
     * Freshness and synchronization are handled
     * by the line_status module.
     */
    if (line_status_is_present(now_ms))
    {
        return true;
    }

    /*
     * Optional bench-only simulated line.
     * Enable only in a dedicated test build.
     */
#ifdef CAR_ENABLE_FAKE_LINE
    return ((int32_t)(s_line_until_ms - now_ms) > 0);
#else
    return false;
#endif
}