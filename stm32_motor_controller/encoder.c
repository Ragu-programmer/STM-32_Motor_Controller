/**
 * encoder.c  --  TIM2 (32-bit) in quadrature encoder mode, x4 counting.
 */
#include "main.h"
#include "encoder.h"

TIM_HandleTypeDef htim2;   /* defined here, declared extern in main.h */

void encoder_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Encoder channel pins PA0/PA1 (AF1 = TIM2_CH1/CH2) */
    g.Pin       = ENC_A_PIN | ENC_B_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;          /* helps with open-collector encoders */
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = ENC_AF;
    HAL_GPIO_Init(ENC_A_PORT, &g);      /* PA0 and PA1 share GPIOA            */

    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = 0;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 0xFFFFFFFF;              /* full 32-bit range   */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode  = TIM_ENCODERMODE_TI12;           /* x4: both edges/chans*/
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = ENC_FILTER;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = ENC_FILTER;

    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) Error_Handler();
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();

    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

int32_t encoder_read(void)
{
    /* Reading the unsigned counter as int32 gives signed position around 0,
     * with correct wrap behaviour (e.g. 0xFFFFFFF6 -> -10). */
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

void encoder_set(int32_t value)
{
    __HAL_TIM_SET_COUNTER(&htim2, (uint32_t)value);
}
