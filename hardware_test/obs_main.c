/*
 * obs_hw_main.c - runs Buddy 5's REAL state machine on the REAL sensor + servo.
 * No RTOS. See src/obs_port_pico_bare.c for what the serial keys do.
 *
 * Extra output: timestamp on every line, time spent in each state, the
 * angle/distance table when a scan finishes, and statistics on a fault.
 *
 * STATUS: written but NOT compiled or run by the author.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "obs_scan.h"
#include "obs_pack.h"
#include "obs_hw_dbg.h"

static const char *k_state[8] = {
    "IDLE", "COARSE_SCAN", "FINE_SCAN", "PROFILING",
    "BYPASSING", "RECOVERY", "FAULT", "?"
};

void obs_hw_help(void)
{
    printf("\n=== Buddy 5 hardware test: real sensor + servo, real state machine ===\n");
    printf("Vehicle Controller keys: s=SCAN_REQ  d=BYPASS_DONE  a=ABORT  r=RESET\n");
    printf("Buddy 3 stand-in:        l=toggle 'line detected'\n");
    printf("Display keys:            v=per-measurement printing  t=angle/distance table\n");
    printf("                         m=statistics  z=zero statistics  h=this help\n");
    printf("Flow: object 25-30 cm ahead -> PROXIMITY -> press s within 2 s -> scan,\n");
    printf("      table, PROFILE, BYPASS_REQ -> press d -> LINE_SEARCH_REQ -> press l\n");
    printf("      -> LINE_REACQUIRED.\n");
    printf("No motors are driven. Servo uses only 30-150 deg.\n");
    printf("Times include your key-press delays, so they are not the bypass-time metric.\n\n");
}

int main(void)
{
    uint8_t last, s;
    uint32_t t_enter, now;

    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000);
    obs_hw_help();

    obs_init();                      /* servo to 90 deg, state IDLE */
    last = obs_get_state();
    t_enter = to_ms_since_boot(get_absolute_time());
    printf("[%7lu ms] [state] %s\n", (unsigned long)t_enter, k_state[last & 7U]);

    for (;;) {
        obs_step();
        s = obs_get_state();
        if (s != last) {
            now = to_ms_since_boot(get_absolute_time());
            printf("[%7lu ms] [state] %s -> %s   (%lu ms in %s; last coarse scan = %u ms, coarse+fine = %u ms)\n",
                   (unsigned long)now, k_state[last & 7U], k_state[s & 7U],
                   (unsigned long)(now - t_enter), k_state[last & 7U],
                   (unsigned)obs_get_last_coarse_ms(), (unsigned)obs_get_last_scan_ms());

            if (s == (uint8_t)OBS_STATE_COARSE) {
                obs_hw_clear_table();                 /* new scan: start the table fresh */
            } else if (s == (uint8_t)OBS_STATE_PROFILING) {
                obs_hw_print_table();                 /* scan finished: show what it saw */
            } else if (s == (uint8_t)OBS_STATE_FAULT) {
                obs_hw_print_stats();                 /* help diagnose the fault */
            } else {
                /* no extra output */
            }
            last = s;
            t_enter = now;
        }
    }
}