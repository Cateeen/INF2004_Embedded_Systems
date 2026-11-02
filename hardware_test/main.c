
#include <stdio.h>

#include "pico/stdlib.h"
#include "motor_driver.h"

/*
 * Buddy 2 - Motor Driver Hardware Test
 *
 * Safe startup verification.
 *
 * Both motors remain stopped.
 * Movement testing requires a separate,
 * explicitly selected test program.
 */

int main(void)
{
    stdio_init_all();

    motor_driver_init();
    motor_emergency_stop();

    while (true)
    {
        printf("Buddy 2: Motors stopped. Hardware test only.\n");
        sleep_ms(1000);
    }

    return 0;
}
