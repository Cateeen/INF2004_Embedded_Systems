
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

/* Initialise a PID controller with gains and output limits. */
void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

/* Calculate the PID output for one control-loop update. */
float pid_update(pid_controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds);

/* Reset accumulated state when stopping or restarting. */
void pid_reset(pid_controller_t *pid);

#endif
