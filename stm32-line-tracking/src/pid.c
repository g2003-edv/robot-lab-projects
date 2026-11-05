#include "pid.h"

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
    pid->output_max = 999.0f;
    /* Strictly below output_max (see PID_SetOutputLimit). */
    pid->integral_max = 500.0f;
    pid->has_last_error = 0;
}

void PID_SetOutputLimit(PID_Controller *pid, float output_max)
{
    if (output_max < 0.0f) {
        output_max = -output_max;
    }
    pid->output_max = output_max;
    /* Anti-windup: half the output range, never equal to it. */
    pid->integral_max = output_max * 0.5f;
}

float PID_Update(PID_Controller *pid, float measurement, float dt)
{
    if (dt <= 0.0f) {
        /* No time elapsed (or a bad call): do not touch the state. */
        return pid->output;
    }

    pid->error = pid->setpoint - measurement;

    pid->integral += pid->error * dt;

    /* Anti-windup on the integral *contribution* (ki * integral). Limiting
     * that instead of the raw accumulator keeps integral_max comparable with
     * output_max whatever ki is, which is what "integral limit below output
     * limit" is supposed to mean. */
    if (pid->ki != 0.0f) {
        float i_term = pid->ki * pid->integral;
        if (i_term > pid->integral_max) {
            pid->integral = pid->integral_max / pid->ki;
        } else if (i_term < -pid->integral_max) {
            pid->integral = -pid->integral_max / pid->ki;
        }
    }

    /* No derivative on the very first update (and after PID_Reset): with
     * dt-normalised D a zero last_error would produce a huge kick. */
    float derivative = 0.0f;
    if (pid->has_last_error) {
        derivative = (pid->error - pid->last_error) / dt;
    }
    pid->last_error = pid->error;
    pid->has_last_error = 1;

    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * derivative;

    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->has_last_error = 0;
}
