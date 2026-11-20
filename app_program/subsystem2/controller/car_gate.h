#ifndef CAR_GATE_H
#define CAR_GATE_H

#include <stdbool.h>
#include <stdint.h>

/* These predicates do not check data freshness or clear a motor E-stop.
 * The controller must validate its inputs and handle stops separately. */
bool car_gate_motion(bool go, bool emergency_stopped, uint32_t faults);
bool car_gate_follow(bool go, bool emergency_stopped, uint32_t faults,
                     bool obstacle);
/* Rearms line following only after the motor E-stop is already clear. */
bool car_gate_rearm(bool go_rising, bool emergency_stopped, uint32_t faults,
                    bool obstacle);

#endif
