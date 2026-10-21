
#include <stdio.h>

#include "pico/stdlib.h"
#include "motor_driver.h"

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
