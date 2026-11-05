/*
 * STM32 Line Tracking & Obstacle Avoidance Car
 * Hardware: STM32F103C8T6, 5-channel IR array, HC-SR04 ultrasonic, L298N motor driver
 * Control: PID closed-loop speed control for smooth cornering
 *
 * Clock tree: HSE 8 MHz -> PLL x9 -> 72 MHz, AHB /1, APB1 /2 (TIM2 still 72 MHz)
 * so TIM2 with Prescaler = 71 and Period = 999 gives exactly 1 kHz PWM.
 */

#include "stm32f1xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"

TIM_HandleTypeDef htim2;
PID_Controller pid_line;
Sensor_Data sensor_data;

/*
 * Obstacle avoidance timing (all non-blocking, driven by HAL_GetTick()).
 *
 * The strategy is deliberately simple: one forward-facing HC-SR04, so an
 * obstacle approaching at a steep angle can still be hit. Speeding it up would
 * need a servo-mounted sensor or side sensors, which this chassis does not have.
 */
#define OBSTACLE_STOP_MS      200U
#define OBSTACLE_TURN_MS      400U
#define OBSTACLE_COOLDOWN_MS  300U
#define TURN_SPEED            300

typedef enum {
    AVOID_IDLE = 0,
    AVOID_STOP,
    AVOID_TURN
} AvoidState;

static AvoidState avoid_state = AVOID_IDLE;
static uint32_t   avoid_state_ms = 0;
static uint8_t    avoid_cooldown = 0;
static uint32_t   avoid_cooldown_ms = 0;

static void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void Avoid_Start(uint32_t now);
static void Avoid_Update(uint32_t now);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();

    Motor_Init(&htim2);
    Sensor_Init();
    /*
     * Gains are expressed "per second" now that PID_Update() normalises by dt.
     * They are the exact equivalent of the old per-cycle tuning (Kp = 25,
     * Ki = 0.5, Kd = 8) at the nominal 10 ms loop:
     *     Ki_new = Ki_old / dt = 0.5 / 0.01 = 50
     *     Kd_new = Kd_old * dt = 8 * 0.01   = 0.08
     *
     * TODO(verify-on-hardware): retune on the real car. The conversion only
     * preserves the behaviour of the 10 ms loop; a different loop period now
     * keeps the same closed-loop response instead of silently changing it.
     */
    PID_Init(&pid_line, 25.0f, 50.0f, 0.08f);
    /* output_max = 400 and integral_max = 200 (half of it), so the integral
     * term alone can never saturate the steering. */
    PID_SetOutputLimit(&pid_line, 400.0f);

    const int16_t base_speed = 500;
    uint32_t last_tick = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();
        float dt = (float)(now - last_tick) / 1000.0f;
        last_tick = now;

        Sensor_ReadIR(&sensor_data);

        if (avoid_cooldown && (int32_t)(now - avoid_cooldown_ms) >= 0) {
            avoid_cooldown = 0;
        }

        if (avoid_state != AVOID_IDLE) {
            /* Keep the loop (and the clock base) running while turning. */
            Avoid_Update(now);
        } else {
            /* Blocking for <= 20 ms worst case; the ping itself is throttled to
             * 60 ms inside Sensor_ReadUltrasonic(). */
            float distance = Sensor_ReadUltrasonic();

            if (Sensor_IsObstacle(distance) && !avoid_cooldown) {
                Avoid_Start(now);
            } else {
                float pid_output = PID_Update(&pid_line, sensor_data.line_position, dt);
                int16_t left_spd  = (int16_t)(base_speed + (int16_t)pid_output);
                int16_t right_spd = (int16_t)(base_speed - (int16_t)pid_output);

                Motor_SetSpeed(left_spd, right_spd);
            }
        }

        HAL_Delay(10);
    }
}

static void Avoid_Start(uint32_t now)
{
    Motor_Stop();
    /* Drop the accumulated error so the car does not lurch when it resumes. */
    PID_Reset(&pid_line);
    avoid_state = AVOID_STOP;
    avoid_state_ms = now;
}

static void Avoid_Update(uint32_t now)
{
    switch (avoid_state) {
    case AVOID_STOP:
        Motor_Stop();
        if ((now - avoid_state_ms) >= OBSTACLE_STOP_MS) {
            /* Spin right in place: signed speeds, -300 on the left track. */
            Motor_SetSpeed(-TURN_SPEED, TURN_SPEED);
            avoid_state = AVOID_TURN;
            avoid_state_ms = now;
        }
        break;

    case AVOID_TURN:
        Motor_SetSpeed(-TURN_SPEED, TURN_SPEED);
        if ((now - avoid_state_ms) >= OBSTACLE_TURN_MS) {
            Motor_Stop();
            PID_Reset(&pid_line);
            avoid_state = AVOID_IDLE;
            avoid_cooldown = 1;
            avoid_cooldown_ms = now + OBSTACLE_COOLDOWN_MS;
        }
        break;

    default:
        avoid_state = AVOID_IDLE;
        break;
    }
}

static void MX_TIM2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TIM2_CH1 = PA0, TIM2_CH2 = PA1 (default mapping, no AFIO remap) drive the
     * L298N ENA/ENB inputs. Without this the pins stay in their reset state and
     * the PWM never reaches the bridge. */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim2.Instance = TIM2;
    /* 72 MHz / (71 + 1) = 1 MHz counter, period 999 -> 1 kHz PWM. */
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*
     * HSE = 8 MHz crystal (STM32F103C8T6 minimum boards ship with one).
     * 8 MHz / 1 (HSE divider) * 9 (PLL) = 72 MHz.
     *
     * TODO(verify-on-hardware): if the board has no 8 MHz crystal, HSE never
     * becomes ready and this ends in Error_Handler(). In that case switch to
     * HSI/2 * 16 = 64 MHz:
     *     OscillatorType = RCC_OSCILLATORTYPE_HSI
     *     HSIState = RCC_HSI_ON, HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT
     *     PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2, PLL.PLLMUL = RCC_PLL_MUL16
     * and change TIM2 Prescaler from 71 to 63 so that
     *     64 MHz / 64 / 1000 = 1 kHz still holds.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* HCLK  = 72 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    /* PCLK1 = 36 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    /* PCLK2 = 72 MHz */
    /* APB1 is /2, so the TIM2 clock is still 72 MHz (PCLK1 x2). */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    /* 72 MHz needs 2 flash wait states; keep the prefetch buffer on. */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
