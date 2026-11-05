#ifndef __SENSOR_H
#define __SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define IR_SENSOR_NUM           5
#define OBSTACLE_THRESHOLD_CM   15.0f

/*
 * HC-SR04 ultrasonic.
 *
 * TODO(verify-on-hardware): TRIG/ECHO pins are a free choice after the IR array
 * moved to PB0..PB4 and the L298N inputs to PB12..PB15. PB5/PB6 are unused by
 * this project (PB6 is USART1_TX only on remap, I2C1_SCL by default - neither is
 * enabled here).
 *
 * ECHO is a 5 V output. PB5/PB6 are 5 V-tolerant on the STM32F103, but a
 * divider (e.g. 1k/2k) is still recommended - see README.
 */
#define US_TRIG_PORT            GPIOB
#define US_TRIG_PIN             GPIO_PIN_5
#define US_ECHO_PORT            GPIOB
#define US_ECHO_PIN             GPIO_PIN_6

/* 20 ms round trip ~= 3.4 m, just under the HC-SR04 maximum range. */
#define US_TIMEOUT_US           20000U
/* The HC-SR04 needs >= 60 ms between pings to let the previous echo decay. */
#define US_MIN_INTERVAL_MS      60U
/* Returned when no echo arrives (out of range or sensor unplugged). */
#define US_NO_ECHO_DISTANCE_CM  400.0f

typedef struct {
    uint8_t ir_values[IR_SENSOR_NUM];
    float   ultrasonic_distance_cm;
    float   line_position;      /* weighted average in sensors, -2.0 .. +2.0 */
} Sensor_Data;

void Sensor_Init(void);

/*
 * Fills ir_values and line_position. line_position is a *float*: the weighted
 * sum divided by the active sensor count is fractional (e.g. -3/2 = -1.5) and
 * truncating it to an integer would bias the PID towards the centre.
 */
void Sensor_ReadIR(Sensor_Data *data);

/* Blocking measurement, worst case ~20 ms. Returns US_NO_ECHO_DISTANCE_CM on
 * timeout and repeats the previous value if called faster than 60 ms. */
float Sensor_ReadUltrasonic(void);

uint8_t Sensor_IsObstacle(float distance);

#ifdef __cplusplus
}
#endif

#endif
