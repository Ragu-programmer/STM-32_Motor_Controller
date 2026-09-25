/**
 * main.h  --  Shared peripheral handles and top-level prototypes.
 *
 * The HAL handles are defined once in main.c and shared with the driver
 * modules (motor/encoder) and the comms module via these externs. This is
 * the same pattern CubeMX uses, so the project drops straight into a
 * CubeMX/HAL tree.
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"
#include "config.h"

extern TIM_HandleTypeDef  htim1;   /* motor PWM          (TIM1_CH1)          */
extern TIM_HandleTypeDef  htim2;   /* quadrature encoder (TIM2, x4 counting) */
extern TIM_HandleTypeDef  htim6;   /* control-loop tick  (period IRQ)        */
extern UART_HandleTypeDef huart2;  /* command / telemetry link               */

void SystemClock_Config(void);
void Error_Handler(void);

#endif /* MAIN_H */
