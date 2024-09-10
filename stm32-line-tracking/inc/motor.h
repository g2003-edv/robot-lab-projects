#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define MOTOR_LEFT_PWM  TIM_CHANNEL_1
#define MOTOR_RIGHT_PWM TIM_CHANNEL_2
#define PWM_MAX         999

void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
