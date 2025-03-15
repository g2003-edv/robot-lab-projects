/*
 * STM32 Line Tracking & Obstacle Avoidance Car
 * Hardware: STM32F103C8T6, 5-channel IR array, HC-SR04 ultrasonic, L298N motor driver
 * Control: PID closed-loop speed control for smooth cornering
 */

#include "stm32f1xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"

TIM_HandleTypeDef htim2;
PID_Controller pid_line;
Sensor_Data sensor_data;

static void SystemClock_Config(void);
static void MX_TIM2_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();
    // 初始化时电机可能会抖动一下，后来加了上电延时解决

    Motor_Init(&htim2);
    Sensor_Init();
    PID_Init(&pid_line, 25.0f, 0.5f, 8.0f);
    pid_line.output_max = 400;

    int16_t base_speed = 500;

    while (1) {
        Sensor_ReadIR(&sensor_data);
        float distance = Sensor_ReadUltrasonic();

        if (Sensor_IsObstacle(distance)) {
            Motor_Stop();
            HAL_Delay(200);
            // FIXME: 避障策略太简单，有时候会撞到斜向障碍物
            Motor_SetSpeed(-300, 300);
            HAL_Delay(400);
            continue;
        }

        float pid_output = PID_Update(&pid_line, (float)sensor_data.line_position);
        int16_t left_spd  = base_speed + (int16_t)pid_output;
        int16_t right_spd = base_speed - (int16_t)pid_output;

        Motor_SetSpeed(left_spd, right_spd);
        HAL_Delay(10);
    }
}

static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
}

static void SystemClock_Config(void)
{
    // Simplified clock config for framework demonstration
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
