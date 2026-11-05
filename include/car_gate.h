#ifndef CAR_GATE_H
#define CAR_GATE_H
#include <stdbool.h>
#include <stdint.h>
/* Faults and emergency stop always veto motion. Obstacle flag vetoes
   ordinary line following but not Buddy 5's controlled avoidance. */
bool car_gate_motion(bool go, bool emergency_stopped, uint32_t faults);
bool car_gate_follow(bool go, bool emergency_stopped, uint32_t faults, bool obstacle);
bool car_gate_rearm(bool go_rising, bool emergency_stopped, uint32_t faults, bool obstacle);
#endif
