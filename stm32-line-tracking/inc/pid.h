#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float error;
    float last_error;
    float integral;         /* accumulated error * dt (unit: error * second) */
    float output;
    /* Upper bound of |ki * integral| in output units (not of the raw
     * accumulator); PID_SetOutputLimit() keeps it at half of output_max. */
    float integral_max;
    float output_max;
    uint8_t has_last_error; /* first update after init/reset has no derivative */
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);

/* Sets the symmetric output limit and re-derives integral_max = output_max/2,
 * so the integral term can never wind up past the actuator limit on its own. */
void PID_SetOutputLimit(PID_Controller *pid, float output_max);

/*
 * dt is the real elapsed time in seconds since the previous call
 * (HAL_GetTick() difference on the target). The integral and derivative terms
 * are normalised by it, so the gains keep their meaning when the loop period
 * changes. A non-positive dt leaves the last output untouched.
 */
float PID_Update(PID_Controller *pid, float measurement, float dt);

void PID_Reset(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif
