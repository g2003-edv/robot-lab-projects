#include "sensor.h"

/* IR array moved off PA0/PA1 (TIM2_CH1/CH2 = motor PWM) and PA2/PA3
 * (USART2) onto PB0..PB4.
 *
 * TODO(verify-on-hardware): PB3/PB4 are JTDO/NJTRST after reset. Sensor_Init()
 * calls __HAL_AFIO_REMAP_SWJ_NOJTAG() so they become GPIO while SWD
 * (PA13/PA14) stays usable for flashing/debugging. */
static GPIO_TypeDef *IR_PORTS[IR_SENSOR_NUM] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB};
static uint16_t IR_PINS[IR_SENSOR_NUM] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2,
                                          GPIO_PIN_3, GPIO_PIN_4};

/* --- microsecond time base -------------------------------------------------
 * TIM2 is taken by the motor PWM, so the Cortex-M3 DWT cycle counter is used
 * instead of adding another timer. 72 MHz / 72 = 1 MHz.
 *
 * TODO(verify-on-hardware): DWT->CYCCNT is present on the STM32F103 core
 * (Cortex-M3 r1p1). If a clone lacks it, replace Sensor_Micros() with a TIM3
 * 1 MHz free-running counter or with input capture on the ECHO pin.
 */
static void Sensor_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t Sensor_Micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Release PB3 (JTDO) and PB4 (NJTRST) for GPIO use, keep SWD. */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        GPIO_InitStruct.Pin = IR_PINS[i];
        HAL_GPIO_Init(IR_PORTS[i], &GPIO_InitStruct);
    }

    /* HC-SR04 trigger: push-pull output, idle low. */
    GPIO_InitStruct.Pin = US_TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(US_TRIG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    /* HC-SR04 echo: floating input (the module drives it). */
    GPIO_InitStruct.Pin = US_ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(US_ECHO_PORT, &GPIO_InitStruct);

    Sensor_DwtInit();
}

void Sensor_ReadIR(Sensor_Data *data)
{
    int16_t weighted_sum = 0;
    int16_t active_count = 0;

    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        /* Sensors are active-low: black line -> low output. */
        uint8_t active =
            (HAL_GPIO_ReadPin(IR_PORTS[i], IR_PINS[i]) == GPIO_PIN_RESET) ? 1U : 0U;
        data->ir_values[i] = active;
        if (active) {
            weighted_sum += (int16_t)(i - IR_SENSOR_NUM / 2);
            active_count++;
        }
    }

    /* Float division on purpose: -3/2 must be -1.5, not -1. */
    data->line_position = (active_count > 0)
                        ? (float)weighted_sum / (float)active_count
                        : 0.0f;
}

float Sensor_ReadUltrasonic(void)
{
    static uint32_t last_ping_ms = 0;
    static uint8_t  pinged_once = 0;
    static float    last_distance = US_NO_ECHO_DISTANCE_CM;

    uint32_t now_ms = HAL_GetTick();
    if (pinged_once && (now_ms - last_ping_ms) < US_MIN_INTERVAL_MS) {
        return last_distance;
    }
    pinged_once = 1;
    last_ping_ms = now_ms;

    /* 1) 10 us trigger pulse. */
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
    uint32_t start = Sensor_Micros();
    while ((Sensor_Micros() - start) < 10U) {
        /* busy wait, ~10 us */
    }
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    /* 2) Wait for the echo pulse to rise. */
    start = Sensor_Micros();
    while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_RESET) {
        if ((Sensor_Micros() - start) > US_TIMEOUT_US) {
            last_distance = US_NO_ECHO_DISTANCE_CM;
            return last_distance;
        }
    }

    /* 3) Measure the high time. */
    uint32_t rise = Sensor_Micros();
    while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_SET) {
        if ((Sensor_Micros() - rise) > US_TIMEOUT_US) {
            last_distance = US_NO_ECHO_DISTANCE_CM;
            return last_distance;
        }
    }
    uint32_t duration_us = Sensor_Micros() - rise;

    /* Sound travels 0.034 cm/us and the pulse covers the path twice. */
    float distance_cm = (float)duration_us * 0.034f / 2.0f;

    if (distance_cm <= 2.0f) {
        /* Below the HC-SR04 dead zone: treat as no usable reading. */
        distance_cm = US_NO_ECHO_DISTANCE_CM;
    }
    last_distance = distance_cm;
    return last_distance;
}

uint8_t Sensor_IsObstacle(float distance)
{
    return (distance < OBSTACLE_THRESHOLD_CM) ? 1U : 0U;
}
