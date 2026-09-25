/**
 * main.c  --  STM32F446RE gripper actuator controller: system bring-up.
 *
 * Wires together the modules:
 *   motor.c      -> TIM1_CH1 PWM + direction GPIO
 *   encoder.c    -> TIM2 quadrature counter
 *   controller.c -> PD closed-loop, run from the TIM6 tick interrupt
 *   comms.c      -> USART2 1-byte command interface + telemetry
 *
 * Interrupt map:
 *   TIM6  period-elapsed @ CONTROL_HZ -> controller_tick()   (highest prio)
 *   USART2 RX byte                    -> command decode      (lower prio)
 *
 * NOTE on stm32f4xx_it.c: this file provides SysTick_Handler,
 * TIM6_DAC_IRQHandler and USART2_IRQHandler directly. If you build from a
 * CubeMX project that already generates stm32f4xx_it.c, either delete those
 * three handlers there, or delete them here -- keep exactly one copy of each.
 */
#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "controller.h"
#include "comms.h"

TIM_HandleTypeDef htim6;   /* control-loop tick; other handles live in modules */

/* ------------------------------------------------------- TIM6 control tick */
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance           = TIM6;
    htim6.Init.Prescaler     = TICK_PSC;
    htim6.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim6.Init.Period        = TICK_ARR;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
}

/* ================================================================== main() */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Peripheral / module init */
    motor_init();          /* TIM1 PWM + direction pins  */
    encoder_init();        /* TIM2 encoder               */
    MX_TIM6_Init();        /* control-loop tick          */
    comms_init();          /* USART2 + banner + arm RX   */
    controller_init();     /* seed state from encoder    */

    /* Interrupt priorities: control loop must not be starved by the UART. */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* Start the control loop */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();

    while (1) {
        comms_service();   /* deferred (blocking) telemetry printing */
    }
}

/* ------------------------------------------------------------- HAL callbacks */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        controller_tick();
    }
}
/* HAL_UART_RxCpltCallback is defined in comms.c */

/* ------------------------------------------------------------- IRQ handlers */
void SysTick_Handler(void)      { HAL_IncTick(); }
void TIM6_DAC_IRQHandler(void)  { HAL_TIM_IRQHandler(&htim6);  }
void USART2_IRQHandler(void)    { HAL_UART_IRQHandler(&huart2); }

/* ------------------------------------------------------------ clock @ 180 MHz
 * PLL from internal HSI (16 MHz): M=8 -> 2, N=180 -> 360, P=2 -> 180 MHz.
 * Over-drive + 5 flash wait states are required for 180 MHz on the F446.
 * APB1 = 45 MHz (TIM x2 = 90), APB2 = 90 MHz (TIM x2 = 180). Matches config.h.
 * (Prefer HSE bypass for a more accurate UART clock -- swap the oscillator
 *  block for RCC_OSCILLATORTYPE_HSE / RCC_HSE_BYPASS, PLLM=4 with 8 MHz HSE.)
 * -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 180;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 4;
    osc.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* 180 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;     /*  45 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;     /*  90 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ---------------------------------------------------------------- fault trap */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { /* halt: attach a debugger to inspect */ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
