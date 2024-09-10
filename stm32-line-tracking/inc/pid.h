#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float error;
    float last_error;
    float integral;
    float output;
    float integral_max;
    float output_max;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Update(PID_Controller *pid, float measurement);
void PID_Reset(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif
