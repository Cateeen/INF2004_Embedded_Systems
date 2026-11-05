#include <assert.h>
#include <stdio.h>
#include "line_steering.h"
#include "buddy3.h"

int main(void)
{
    line_steering_t steering;
    line_steering_reset(&steering);
    line_steering_command_t cmd = line_steering_update(&steering, POS_JUNCTION, 1000);
    assert(cmd.motor_1_pwm == 25 && cmd.motor_2_pwm == -25 && !cmd.stop_latched);
    cmd = line_steering_update(&steering, POS_LEFT, 1010);
    assert(cmd.motor_1_pwm == 15 && cmd.motor_2_pwm == -25);
    cmd = line_steering_update(&steering, POS_RIGHT, 1020);
    assert(cmd.motor_1_pwm == 25 && cmd.motor_2_pwm == -15);
    cmd = line_steering_update(&steering, POS_LOST, 1030);
    assert(cmd.motor_1_pwm == 25 && cmd.motor_2_pwm == -15);
    cmd = line_steering_update(&steering, POS_LOST, 1329);
    assert(!cmd.stop_latched);
    cmd = line_steering_update(&steering, POS_LOST, 1330);
    assert(cmd.stop_latched && cmd.motor_1_pwm == 0 && cmd.motor_2_pwm == 0);
    cmd = line_steering_update(&steering, POS_JUNCTION, 1400);
    assert(cmd.stop_latched);
    line_steering_reset(&steering);
    cmd = line_steering_update(&steering, POS_JUNCTION, 1410);
    assert(!cmd.stop_latched && cmd.motor_1_pwm == 25);
    puts("PASS: line steering decisions and lost-line latch");
    return 0;
}
