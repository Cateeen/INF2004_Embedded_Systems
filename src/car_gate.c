#include "car_gate.h"
bool car_gate_motion(bool go, bool emergency_stopped, uint32_t faults)
{
    return go && !emergency_stopped && faults == 0U;
}
bool car_gate_follow(bool go, bool emergency_stopped, uint32_t faults, bool obstacle)
{
    return car_gate_motion(go, emergency_stopped, faults) && !obstacle;
}
bool car_gate_rearm(bool go_rising, bool emergency_stopped, uint32_t faults, bool obstacle)
{
    return go_rising && car_gate_follow(true, emergency_stopped, faults, obstacle);
}
