#include <assert.h>
#include <stdio.h>
#include "car_gate.h"
int main(void)
{
    assert(!car_gate_motion(false, false, 0U));
    assert(car_gate_motion(true, false, 0U));
    assert(!car_gate_motion(true, true, 0U));
    assert(!car_gate_motion(true, false, 1U));
    assert(!car_gate_motion(true, false, (1UL << 16)));
    assert(!car_gate_follow(true, false, 0U, true));
    assert(car_gate_motion(true, false, 0U)); /* Avoidance may operate */
    assert(!car_gate_rearm(false, false, 0U, false));
    assert(!car_gate_rearm(true, true, 0U, false));
    assert(!car_gate_rearm(true, false, 1U, false));
    assert(!car_gate_rearm(true, false, 0U, true));
    assert(car_gate_rearm(true, false, 0U, false));
    puts("PASS: safety gate STOP/GO/fault/obstacle cases");
    return 0;
}
