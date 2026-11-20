
#include "pid_controller.h"

#include <math.h>
#include <stddef.h>

bool pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if (pid == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(output_min) || !isfinite(output_max))
    {
        return false;
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

    return pid_reset(pid);
}

bool pid_reset(pid_controller_t *pid)
{
    if (pid == NULL)
    {
        return false;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->first_update = true;
    return true;
}

bool pid_update(pid_controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds,
                 float *result)
{
    if ((pid == NULL) ||
        (result == NULL) ||
        (!isfinite(pid->kp)) || (!isfinite(pid->ki)) || (!isfinite(pid->kd)) ||
        (!isfinite(pid->integral)) || (!isfinite(pid->previous_error)) ||
        (!isfinite(pid->output_min)) || (!isfinite(pid->output_max)) ||
        (pid->output_min > pid->output_max) ||
        (!isfinite(setpoint)) ||
        (!isfinite(measurement)) ||
        (!isfinite(dt_seconds)) ||
        (dt_seconds <= 0.0f))
    {
        return false;
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

    if (!isfinite(error) || !isfinite(proportional) ||
        !isfinite(derivative) || !isfinite(candidate_integral) ||
        !isfinite(candidate_output))
    {
        return false;
    }

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

    float output =
        proportional +
        (pid->ki * (allow_integration ? candidate_integral : pid->integral)) +
        derivative;

    if (!isfinite(output))
    {
        return false;
    }

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    if (allow_integration)
    {
        pid->integral = candidate_integral;
    }
    pid->previous_error = error;
    pid->first_update = false;

    *result = output;
    return true;
}
