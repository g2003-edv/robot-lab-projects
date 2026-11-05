#include "motor.h"
#include <stdlib.h>

static TIM_HandleTypeDef *motor_tim = NULL;

static void Motor_ApplyDirection(GPIO_TypeDef *in_a_port, uint16_t in_a_pin,
                                 GPIO_TypeDef *in_b_port, uint16_t in_b_pin,
                                 int16_t speed)
{
    /* IN1 = HIGH, IN2 = LOW -> forward; the opposite pair -> reverse.
     * speed == 0 leaves both inputs low (coast). */
    HAL_GPIO_WritePin(in_a_port, in_a_pin, (speed > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in_b_port, in_b_pin, (speed < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Init(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    motor_tim = htim;

    /* Enabled unconditionally: GPIOB is where the L298N inputs live today, and
     * an extra clock enable is harmless if they are moved. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /* Initialise each input separately so the port of every macro is honoured. */
    GPIO_InitStruct.Pin = MOTOR_LEFT_IN1_PIN;
    HAL_GPIO_Init(MOTOR_LEFT_IN1_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = MOTOR_LEFT_IN2_PIN;
    HAL_GPIO_Init(MOTOR_LEFT_IN2_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = MOTOR_RIGHT_IN3_PIN;
    HAL_GPIO_Init(MOTOR_RIGHT_IN3_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = MOTOR_RIGHT_IN4_PIN;
    HAL_GPIO_Init(MOTOR_RIGHT_IN4_PORT, &GPIO_InitStruct);

    /* Drive the bridge to "stop" *before* the PWM channels start, otherwise the
     * compare register (0) plus floating inputs can make the motors twitch at
     * power-up. */
    Motor_Stop();
    HAL_TIM_PWM_Start(motor_tim, MOTOR_LEFT_PWM);
    HAL_TIM_PWM_Start(motor_tim, MOTOR_RIGHT_PWM);
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    /* Clamp in 32-bit space before taking abs(): abs(-32768) is undefined for
     * int16_t and would wrap. */
    int32_t left = left_speed;
    int32_t right = right_speed;

    if (motor_tim == NULL) {
        return;
    }

    if (left > PWM_MAX) left = PWM_MAX;
    if (left < -PWM_MAX) left = -PWM_MAX;
    if (right > PWM_MAX) right = PWM_MAX;
    if (right < -PWM_MAX) right = -PWM_MAX;

    Motor_ApplyDirection(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN,
                         MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, (int16_t)left);
    Motor_ApplyDirection(MOTOR_RIGHT_IN3_PORT, MOTOR_RIGHT_IN3_PIN,
                         MOTOR_RIGHT_IN4_PORT, MOTOR_RIGHT_IN4_PIN, (int16_t)right);

    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_LEFT_PWM, (uint32_t)abs((int)left));
    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_RIGHT_PWM, (uint32_t)abs((int)right));
}

void Motor_Stop(void)
{
    if (motor_tim == NULL) {
        return;
    }

    Motor_ApplyDirection(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN,
                         MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, 0);
    Motor_ApplyDirection(MOTOR_RIGHT_IN3_PORT, MOTOR_RIGHT_IN3_PIN,
                         MOTOR_RIGHT_IN4_PORT, MOTOR_RIGHT_IN4_PIN, 0);

    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_LEFT_PWM, 0);
    __HAL_TIM_SET_COMPARE(motor_tim, MOTOR_RIGHT_PWM, 0);
}
