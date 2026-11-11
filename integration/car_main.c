/*
 * car_main.c - application entry point.
 *
 * The default build remains the existing bare-metal bring-up until the
 * uT-Kernel task modules are migrated. Define PICOCAR_USE_UTKERNEL when the
 * vehicle_system startup module is included in the target.
 */
#include <stdio.h>
#include "pico/stdlib.h"

#if defined(PICOCAR_USE_UTKERNEL)

#include "vehicle_system.h"

int main(void)
{
    stdio_init_all();
    vehicle_system_run();
    return 0;
}

#else

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

#endif
