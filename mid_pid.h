#ifndef MID_PID_H
#define MID_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float error;
    float previous_error;
    float integral;
    float output;
} PID;

void pid_init(PID *pid, float kp, float ki, float kd, float integralLimit,
    float outputLimit);
void pid_reset(PID *pid);
float pid_calc(PID *pid, float reference, float feedback);

#endif
