/*
 * obs_port_pico_bare.c - BENCH implementation of obs_port.h on a Pico W, NO RTOS.
 *
 * Lets the REAL obs_scan.c / obs_profile.c / obs_pack.c run against the REAL
 * HC-SR04 and servo. Everything the Vehicle Controller, Safety Monitor and
 * Buddy 3 would do is replaced by USB-serial keys and printed text:
 *
 *   s = SCAN_REQ      d = BYPASS_DONE   a = ABORT       r = RESET
 *   l = toggle "line detected" (stands in for Buddy 3)
 *   v = toggle per-measurement printing
 *   t = print angle/distance table     m = print statistics
 *   z = zero the statistics            h = help
 *
 * This is a measuring tool, not the final design: ECHO is polled (busy-wait)
 * instead of using the interrupt + event flag in obs_port_pico_tk.c, and delays
 * use sleep_ms(). It does NOT test the RTOS port.
 *
 * STATUS: written but NOT compiled or run by the author. Pico SDK 1.5.1 APIs only.
 */
#include <stdio.h>
#include "obs_port.h"
#include "obs_cfg.h"
#include "obs_hw_dbg.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define STALE_ECHO_WAIT_US   45000U   /* a "no object" echo can stay high ~38 ms */
#define DIST_NOT_MEASURED    0U
#define DIST_NO_OBJECT       0xFFFFU
#define BAR_MM_PER_CHAR      20U
#define BAR_MAX_CHARS        50U

static bool     s_line = false;       /* simulated Buddy 3 line detection */
static bool     s_verbose = true;
static uint8_t  s_deg = 90U;          /* last commanded servo angle */
static uint16_t s_servo_us = 1500U;   /* last commanded servo pulse */

/* per-angle result of the most recent measurement at that angle */
static uint16_t s_dist[181];

/* statistics */
static uint32_t s_n_ok, s_n_none, s_n_noresp, s_n_stuck;
static uint16_t s_pmin = 0xFFFFU, s_pmax = 0U;
static uint32_t s_rise_max = 0U;

static const char *k_type[8] = {
    "NONE", "PROXIMITY", "PROFILE", "BYPASS_REQ",
    "LINE_SEARCH_REQ", "LINE_REACQUIRED", "SCAN_CLEAR", "FAULT_EVT"
};
static const char *k_action[5] = { "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE" };
static const char *k_side[3]   = { "NONE", "LEFT", "RIGHT" };

/* ts - print a millisecond timestamp prefix. */
static void ts(void)
{
    printf("[%7lu ms] ", (unsigned long)to_ms_since_boot(get_absolute_time()));
}

/* ---------------- debug helpers ---------------- */

void obs_hw_clear_table(void)
{
    uint32_t i;
    for (i = 0U; i < 181U; i++) {
        s_dist[i] = DIST_NOT_MEASURED;
    }
}

void obs_hw_print_table(void)
{
    uint32_t a, n;
    printf("--- angle / distance table (0 = right, 90 = ahead, 180 = left; # = %u mm) ---\n",
           (unsigned)BAR_MM_PER_CHAR);
    for (a = 0U; a < 181U; a++) {
        uint16_t v = s_dist[a];
        if (v == DIST_NOT_MEASURED) {
            continue;
        }
        if (v == DIST_NO_OBJECT) {
            printf("%4lu   none\n", (unsigned long)a);
        } else {
            printf("%4lu %5u  ", (unsigned long)a, (unsigned)v);
            n = (uint32_t)v / BAR_MM_PER_CHAR;
            if (n > BAR_MAX_CHARS) { n = BAR_MAX_CHARS; }
            while (n > 0U) { putchar('#'); n--; }
            putchar('\n');
        }
    }
    printf("--- end of table ---\n");
}

void obs_hw_print_stats(void)
{
    printf("--- statistics ---\n");
    printf("measurements: ok=%lu no_object=%lu no_response=%lu stuck_high=%lu\n",
           (unsigned long)s_n_ok, (unsigned long)s_n_none,
           (unsigned long)s_n_noresp, (unsigned long)s_n_stuck);
    if (s_n_ok > 0U) {
        printf("pulse min=%u us (%u mm)  max=%u us (%u mm)\n",
               (unsigned)s_pmin, (unsigned)((s_pmin * 10U) / 58U),
               (unsigned)s_pmax, (unsigned)((s_pmax * 10U) / 58U));
        printf("worst TRIG-to-rise delay = %lu us\n", (unsigned long)s_rise_max);
    }
    printf("servo now: %u deg, %u us pulse   line flag: %s   verbose: %s\n",
           (unsigned)s_deg, (unsigned)s_servo_us, s_line ? "YES" : "no", s_verbose ? "on" : "off");
    printf("------------------\n");
}

static void stats_zero(void)
{
    s_n_ok = 0U; s_n_none = 0U; s_n_noresp = 0U; s_n_stuck = 0U;
    s_pmin = 0xFFFFU; s_pmax = 0U; s_rise_max = 0U;
    printf("[statistics zeroed]\n");
}

/* ---------------- obs_port.h ---------------- */

/* obs_port_init - TRIG out, ECHO in (pull-down), servo PWM 50 Hz, 1 us per tick. */
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
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 19999U);
    pwm_init(slice, &cfg, true);

    obs_hw_clear_table();
}

uint32_t obs_port_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void obs_port_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/* obs_port_servo_deg - 0..180 deg -> OBS_SERVO_MIN_US..MAX_US pulse. Returns at once. */
void obs_port_servo_deg(uint8_t deg)
{
    uint32_t d = (OBS_SERVO_REVERSED != 0U) ? (180U - deg) : deg;
    uint32_t us = OBS_SERVO_MIN_US + ((d * (OBS_SERVO_MAX_US - OBS_SERVO_MIN_US)) / 180U);
    s_deg = deg;
    s_servo_us = (uint16_t)us;
    pwm_set_gpio_level(OBS_PIN_SERVO, (uint16_t)us);
}

/* record - remember the result at the current angle. */
static void record(uint16_t v)
{
    if (s_deg <= 180U) {
        s_dist[s_deg] = v;
    }
}

/*
 * obs_port_measure - one blocking HC-SR04 measurement (polled).
 *  no rise within OBS_RISE_TIMEOUT_MS -> NO_RESPONSE (wiring / sensor fault)
 *  no fall within OBS_ECHO_TIMEOUT_MS -> NO_OBJECT   (beyond the range window)
 */
obs_echo_t obs_port_measure(uint16_t *pulse_us)
{
    uint32_t t0 = time_us_32();
    uint32_t t_rise, w, rise_delay;
    uint32_t mm;

    while (gpio_get(OBS_PIN_ECHO) != 0) {                 /* let a stale echo end */
        if ((time_us_32() - t0) > STALE_ECHO_WAIT_US) {
            s_n_stuck++;
            if (s_verbose) { ts(); printf("  meas @%3u deg: ECHO stuck high\n", (unsigned)s_deg); }
            return OBS_ECHO_NO_RESPONSE;
        }
    }

    gpio_put(OBS_PIN_TRIG, 1);
    busy_wait_us_32(10U);                                  /* 10 us TRIG pulse */
    gpio_put(OBS_PIN_TRIG, 0);

    t0 = time_us_32();
    while (gpio_get(OBS_PIN_ECHO) == 0) {
        if ((time_us_32() - t0) > (OBS_RISE_TIMEOUT_MS * 1000U)) {
            s_n_noresp++;
            if (s_verbose) { ts(); printf("  meas @%3u deg: NO_RESPONSE (ECHO never rose)\n", (unsigned)s_deg); }
            return OBS_ECHO_NO_RESPONSE;
        }
    }
    t_rise = time_us_32();
    rise_delay = t_rise - t0;
    if (rise_delay > s_rise_max) { s_rise_max = rise_delay; }

    while (gpio_get(OBS_PIN_ECHO) != 0) {
        if ((time_us_32() - t_rise) > (OBS_ECHO_TIMEOUT_MS * 1000U)) {
            s_n_none++;
            record(DIST_NO_OBJECT);
            if (s_verbose) { ts(); printf("  meas @%3u deg: no object in range\n", (unsigned)s_deg); }
            return OBS_ECHO_NO_OBJECT;
        }
    }
    w = time_us_32() - t_rise;
    mm = (w * 10U) / 58U;
    *pulse_us = (uint16_t)w;

    s_n_ok++;
    if ((uint16_t)w < s_pmin) { s_pmin = (uint16_t)w; }
    if ((uint16_t)w > s_pmax) { s_pmax = (uint16_t)w; }
    record((mm == 0U) ? 1U : (uint16_t)mm);

    if (s_verbose) {
        ts();
        printf("  meas @%3u deg (servo %4u us): %4lu us = %4lu mm  rise delay %lu us\n",
               (unsigned)s_deg, (unsigned)s_servo_us, (unsigned long)w,
               (unsigned long)mm, (unsigned long)rise_delay);
    }
    return OBS_ECHO_OK;
}

/*
 * obs_port_cmd_recv - the keyboard acts as the Vehicle Controller.
 * Waits up to timeout_ms (0 = poll once). Keys l v t m z h are handled here and
 * do not count as commands.
 */
bool obs_port_cmd_recv(uint8_t *cmd, uint32_t timeout_ms)
{
    absolute_time_t end = make_timeout_time_ms(timeout_ms);

    do {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            switch (c) {
            case 's': ts(); printf("[VC] SCAN_REQ\n");    *cmd = (uint8_t)OBS_CMD_SCAN_REQ;    return true;
            case 'd': ts(); printf("[VC] BYPASS_DONE\n"); *cmd = (uint8_t)OBS_CMD_BYPASS_DONE; return true;
            case 'a': ts(); printf("[VC] ABORT\n");       *cmd = (uint8_t)OBS_CMD_ABORT;       return true;
            case 'r': ts(); printf("[VC] RESET\n");       *cmd = (uint8_t)OBS_CMD_RESET;       return true;
            case 'l': s_line = !s_line; ts(); printf("[B3] line detected = %s\n", s_line ? "YES" : "no"); break;
            case 'v': s_verbose = !s_verbose; printf("[verbose %s]\n", s_verbose ? "on" : "off"); break;
            case 't': obs_hw_print_table(); break;
            case 'm': obs_hw_print_stats(); break;
            case 'z': stats_zero(); break;
            case 'h': obs_hw_help(); break;
            default: break;
            }
        } else {
            sleep_ms(1);
        }
    } while (!time_reached(end));
    return false;
}

/* obs_port_msg_send - prints the decoded message and its 8 bytes (the MQTT payload). */
void obs_port_msg_send(const obs_msg_t *m)
{
    uint8_t b[8];
    unsigned t = (unsigned)obs_msg_type(m);
    int i;

    obs_msg_to_bytes(m, b);
    ts();
    printf("[MSG -> VC] %s", (t < 8U) ? k_type[t] : "?");

    if (t == (unsigned)OBS_MSG_PROXIMITY) {
        printf("  closest=%lu mm", (unsigned long)obs_bits_get(b, 0U, 12U));
    } else if (t == (unsigned)OBS_MSG_PROFILE) {
        obs_profile_msg_t p;
        obs_msg_unpack_profile(m, &p);
        printf("  closest=%u width=%u left=%u right=%u action=%s valid=%d coarse=%u ms",
               (unsigned)p.closest_mm, (unsigned)p.width_mm, (unsigned)p.left_mm,
               (unsigned)p.right_mm, (p.action < 5U) ? k_action[p.action] : "?",
               p.valid ? 1 : 0, (unsigned)p.scan_ms);
    } else if (t == (unsigned)OBS_MSG_BYPASS_REQ) {
        uint8_t side; uint16_t lat, adv;
        obs_msg_unpack_bypass(m, &side, &lat, &adv);
        printf("  side=%s lateral=%u mm advance=%u mm",
               (side < 3U) ? k_side[side] : "?", (unsigned)lat, (unsigned)adv);
    } else if (t == (unsigned)OBS_MSG_LINE_SEARCH_REQ) {
        uint8_t dir; uint16_t mx;
        obs_msg_unpack_line_search(m, &dir, &mx);
        printf("  turn=%s max_search=%u mm", (dir < 3U) ? k_side[dir] : "?", (unsigned)mx);
    } else if ((t == (unsigned)OBS_MSG_SCAN_CLEAR) || (t == (unsigned)OBS_MSG_LINE_REACQUIRED)) {
        printf("  scan_ms=%lu", (unsigned long)obs_bits_get(b, 0U, 10U));
    } else if (t == (unsigned)OBS_MSG_FAULT_EVT) {
        printf("  bits=0x%lX", (unsigned long)obs_msg_unpack_fault(m));
    }
    printf("\n         bytes:");
    for (i = 0; i < 8; i++) { printf(" %02X", (unsigned)b[i]); }
    printf("\n");
}

void obs_port_fault_set(uint32_t bits)
{
    ts();
    printf("[FAULT SET] bits=0x%lX (1=NO_ECHO 2=NO_CLEAR_SIDE 4=BYPASS_BLOCKED 8=LINE_NOT_REACQUIRED)\n",
           (unsigned long)bits);
}

void obs_port_obstacle_flag(bool present)
{
    ts();
    printf("[SAFETY FLAG] obstacle present = %s\n", present ? "YES" : "no");
}

bool obs_port_line_detected(void)
{
    return s_line;
}