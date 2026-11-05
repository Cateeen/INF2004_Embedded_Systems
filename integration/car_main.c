/* car_main.c - obs_core (core 0): obstacle task.  vc_core (core 1): Vehicle Controller + motors.
 * Bring-up build, no RTOS. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "car_link.h"
#include "obs_scan.h"
#include "vehicle_ctrl.h"
#include "line_status.h"
#include "car_observer.h"
#include "car_measurements.h"

/* vc_core - core 1. The motors live on this core only. */
static void vc_core(void)
{
    vc_task();
}

/* obs_core - core 0. obs_init() + obs_step() forever. */
static void obs_core(void)
{
    obs_init();
    car_observer_init();
    for (;;) {
        obs_step();
        car_observer_poll();
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000);                             /* time to open the USB serial terminal */
    printf("\nPicoCar: obstacle + motors.  Wheels OFF THE GROUND for the first run.\n");
    printf("obs_core = core 0, vc_core = core 1\n");
    printf("keys: g=GO  x=STOP  a=ABORT  r=RESET_DISABLED  h=help\n");

    car_link_init();
    line_status_init();
    car_measurements_init();
    multicore_launch_core1(vc_core);
    obs_core();
    return 0;
}