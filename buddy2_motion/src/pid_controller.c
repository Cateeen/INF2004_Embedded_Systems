
#include "pid_controller.h"

#include <math.h>
#include <stddef.h>

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    if (output_min <= output_max)
    {
        pid->output_min = output_min;
        pid->output_max = output_max;
    }
    else
    {
        pid->output_min = output_max;
        pid->output_max = output_min;
    }

    pid_reset(pid);
}

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->first_update = true;
}

float pid_update(pid_controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds)
{
    if ((pid == NULL) ||
        (!isfinite(setpoint)) ||
        (!isfinite(measurement)) ||
        (!isfinite(dt_seconds)) ||
        (dt_seconds <= 0.0f))
    {
        return 0.0f;
    }

    float error = setpoint - measurement;

    float proportional = pid->kp * error;

    float derivative = 0.0f;

    if (!pid->first_update)
    {
        derivative =
            pid->kd *
            (error - pid->previous_error) /
            dt_seconds;
    }

    float candidate_integral =
        pid->integral + (error * dt_seconds);

    float candidate_output =
        proportional +
        (pid->ki * candidate_integral) +
        derivative;

    /*
     * Conditional integration:
     * Do not accumulate more integral error when
     * the output is saturated in the same direction.
     */
    bool allow_integration =
        ((candidate_output <= pid->output_max) &&
         (candidate_output >= pid->output_min)) ||
        ((candidate_output > pid->output_max) &&
         (error < 0.0f)) ||
        ((candidate_output < pid->output_min) &&
         (error > 0.0f));

    if (allow_integration)
    {
        pid->integral = candidate_integral;
    }

    float output =
        proportional +
        (pid->ki * pid->integral) +
        derivative;

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    pid->previous_error = error;
    pid->first_update = false;

    return output;
}
