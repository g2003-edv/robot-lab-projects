#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/*
 * L298N wiring
 * ------------
 * PWM (speed):   TIM2_CH1 = PA0 -> ENA, TIM2_CH2 = PA1 -> ENB
 * Direction:     IN1/IN2 (left), IN3/IN4 (right)
 *
 * PA0/PA1 are TIM2_CH1/CH2 by default mapping (no AFIO remap needed); they are
 * configured as alternate-function push-pull in MX_TIM2_Init().
 *
 * The IR array used to sit on PA0..PA4 and therefore fought TIM2 for PA0/PA1;
 * it now lives on PB0..PB4 (see sensor.h).
 *
 * TODO(verify-on-hardware): PB12..PB15 were picked because they are free after
 * the IR (PB0..PB4), the ultrasonic (PB5/PB6) and SWD (PA13/PA14) allocations.
 * Check the actual harness and re-assign here if the L298N is wired elsewhere.
 * Direction polarity depends on how the motors are soldered: if a wheel spins
 * backwards, swap IN1/IN2 (or IN3/IN4) below.
 */
#define MOTOR_LEFT_PWM          TIM_CHANNEL_1      /* PA0 -> ENA */
#define MOTOR_RIGHT_PWM         TIM_CHANNEL_2      /* PA1 -> ENB */

#define MOTOR_LEFT_IN1_PORT     GPIOB
#define MOTOR_LEFT_IN1_PIN      GPIO_PIN_12
#define MOTOR_LEFT_IN2_PORT     GPIOB
#define MOTOR_LEFT_IN2_PIN      GPIO_PIN_13
#define MOTOR_RIGHT_IN3_PORT    GPIOB
#define MOTOR_RIGHT_IN3_PIN     GPIO_PIN_14
#define MOTOR_RIGHT_IN4_PORT    GPIOB
#define MOTOR_RIGHT_IN4_PIN     GPIO_PIN_15

/* TIM2: 72 MHz / (71 + 1) / (999 + 1) = 1 kHz, 0..999 counts of resolution. */
#define PWM_MAX                 999

void Motor_Init(TIM_HandleTypeDef *htim);

/*
 * Signed speed interface: +PWM_MAX .. -PWM_MAX.
 * The sign selects the direction pins, abs(speed) becomes the PWM compare
 * value, so Motor_SetSpeed(-300, 300) really does spin in place.
 * Safe to call before Motor_Init(): the call is ignored until the timer is set.
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

/* Both channels to 0 and both direction inputs low (coast). */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
