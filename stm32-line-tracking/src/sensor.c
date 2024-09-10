#include "sensor.h"
#include <string.h>

static GPIO_TypeDef *IR_PORTS[IR_SENSOR_NUM] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
static uint16_t IR_PINS[IR_SENSOR_NUM] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};

void Sensor_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        GPIO_InitStruct.Pin = IR_PINS[i];
        HAL_GPIO_Init(IR_PORTS[i], &GPIO_InitStruct);
    }
}

void Sensor_ReadIR(Sensor_Data *data)
{
    memset(data->ir_values, 0, sizeof(data->ir_values));
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        data->ir_values[i] = (HAL_GPIO_ReadPin(IR_PORTS[i], IR_PINS[i]) == GPIO_PIN_RESET) ? 1 : 0;
    }

    int8_t weighted_sum = 0;
    int8_t active_count = 0;
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        if (data->ir_values[i]) {
            weighted_sum += (i - 2);
            active_count++;
        }
    }
    data->line_position = (active_count > 0) ? (weighted_sum / active_count) : 0;
}

float Sensor_ReadUltrasonic(void)
{
    // HC-SR04 超声波测距逻辑（简化框架）
    // 实际实现需配置 TRIG 和 ECHO GPIO + TIM 捕获
    // 返回距离：distance = duration * 0.034 / 2 (cm)
    return 30.0f; // placeholder
}

uint8_t Sensor_IsObstacle(float distance)
{
    return (distance < OBSTACLE_THRESHOLD_CM) ? 1 : 0;
}
