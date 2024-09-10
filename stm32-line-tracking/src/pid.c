#include "pid.h"
#include <math.h>

void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->integral_max = 1000.0f;
    pid->output_max = 999.0f;
}

float PID_Update(PID_Controller *pid, float measurement)
{
    pid->error = pid->setpoint - measurement;

    pid->integral += pid->error;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    float derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;

    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * derivative;

    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
