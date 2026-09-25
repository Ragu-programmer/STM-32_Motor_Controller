/**
 * motor.c  --  H-bridge driver on TIM1_CH1 (PWM) + IN1/IN2/STBY GPIO.
 */
#include "main.h"
#include "motor.h"

TIM_HandleTypeDef htim1;   /* defined here, declared extern in main.h */

/* ------------------------------------------------------------------ helpers */
static inline void dir_forward(void)
{
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
}
static inline void dir_reverse(void)
{
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
}
static inline void set_duty(uint32_t d)
{
    if (d > PWM_ARR) d = PWM_ARR;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d);
}

/* --------------------------------------------------------------------- init */
void motor_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* PWM pin (PA8, AF1 = TIM1_CH1) */
    g.Pin       = MOTOR_PWM_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = MOTOR_PWM_AF;
    HAL_GPIO_Init(MOTOR_PWM_PORT, &g);

    /* Direction + standby pins (push-pull outputs) */
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Alternate = 0;
    g.Pin = MOTOR_IN1_PIN;  HAL_GPIO_Init(MOTOR_IN1_PORT,  &g);
    g.Pin = MOTOR_IN2_PIN;  HAL_GPIO_Init(MOTOR_IN2_PORT,  &g);
    g.Pin = MOTOR_STBY_PIN; HAL_GPIO_Init(MOTOR_STBY_PORT, &g);

    /* Enable the driver (STBY high) */
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);

    /* TIM1 as edge-aligned PWM */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = PWM_PSC;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = PWM_ARR;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode      = TIM_OCMODE_PWM1;
    oc.Pulse       = 0;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState= TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    /* Start PWM. For advanced timer TIM1 this also enables the main output. */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    motor_coast();
}

/* ---------------------------------------------------------------- interface */
void motor_set(float u)
{
#if MOTOR_INVERT
    u = -u;
#endif
    if (u >= 0.0f) {
        dir_forward();
        set_duty((uint32_t)(u + 0.5f));
    } else {
        dir_reverse();
        set_duty((uint32_t)(-u + 0.5f));
    }
}

void motor_coast(void)
{
    set_duty(0);
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
}

void motor_brake(void)
{
    set_duty(0);
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
}
