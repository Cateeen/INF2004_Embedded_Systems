/* car_main.c - obs_core (core 0): obstacle task.  vc_core (core 1): Vehicle Controller + motors.
 * Bring-up build, no RTOS. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "car_link.h"
#include "obs_scan.h"
#include "vehicle_ctrl.h"

/* vc_core - core 1. The motors live on this core only. */
static void vc_core(void)
{
    vc_task();
}

/* obs_core - core 0. obs_init() + obs_step() forever. */
static void obs_core(void)
{
    obs_task();
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000);                             /* time to open the USB serial terminal */
    printf("\nPicoCar: obstacle + motors.  Wheels OFF THE GROUND for the first run.\n");
    printf("obs_core = core 0, vc_core = core 1\n");
    printf("keys: g=GO  x=STOP  l=fake line (1 s)  a=ABORT  r=RESET  h=help\n");

    car_link_init();                            /* queues must exist before vc_core starts */
    multicore_launch_core1(vc_core);
    obs_core();
    return 0;
}