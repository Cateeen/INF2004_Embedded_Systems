
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;

    float output_min;
    float output_max;

    bool first_update;
} pid_controller_t;

/* Initialise a PID controller with finite gains and output limits. */
bool pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

/* False means invalid input or arithmetic; state and output are unchanged. */
bool pid_update(pid_controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds,
                 float *output);

/* Reset accumulated state when stopping or restarting. */
bool pid_reset(pid_controller_t *pid);

#endif
