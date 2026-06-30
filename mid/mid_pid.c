#include "mid_pid.h"

void pid_init(PID *pid, float kp, float ki, float kd, float integralLimit,
    float outputLimit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integralLimit;
    pid->output_limit = outputLimit;
    pid_reset(pid);
}

void pid_reset(PID *pid)
{
    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float pid_calc(PID *pid, float reference, float feedback)
{
    pid->previous_error = pid->error;
    pid->error = reference - feedback;
    pid->integral += pid->error;

    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) +
        (pid->kd * (pid->error - pid->previous_error));

    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    return pid->output;
}
