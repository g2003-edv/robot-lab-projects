#include "motor.h"

static TIM_HandleTypeDef *motor_tim = NULL;

void Motor_Init(TIM_HandleTypeDef *htim)
{
    motor_tim = htim;
    HAL_TIM_PWM_Start(motor_tim, MOTOR_LEFT_PWM);
    HAL_TIM_PWM_Start(motor_tim, MOTOR_RIGHT_PWM);
    Motor_Stop();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    if (left_speed > PWM_MAX) left_speed = PWM_MAX;
    if (left_speed < -PWM_MAX) left_speed = -PWM_MAX;
    if (right_speed > PWM_MAX) right_speed = PWM_MAX;
    if (right_speed < -PWM_MAX) right_speed = -PWM_MAX;

    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_LEFT_PWM,
                          (left_speed >= 0) ? left_speed : 0);
    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_RIGHT_PWM,
                          (right_speed >= 0) ? right_speed : 0);
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_LEFT_PWM, 0);
    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_RIGHT_PWM, 0);
}
