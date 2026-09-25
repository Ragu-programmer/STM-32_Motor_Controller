/**
 * comms.c  --  USART2 command/telemetry link.
 *
 * Commands are single bytes decoded in the RX-complete interrupt. The
 * handler only calls the (non-blocking) controller API and sets flags; the
 * actual UART transmission of telemetry happens in comms_service() from the
 * main loop so we never block inside an ISR.
 */
#include "main.h"
#include "comms.h"
#include "controller.h"

#include <string.h>
#include <stdio.h>

UART_HandleTypeDef huart2;   /* defined here, declared extern in main.h */

static uint8_t        rx_byte;                 /* single-byte RX buffer      */
static volatile int   status_req = 0;          /* deferred status print flag */
static volatile int   banner_req = 0;          /* deferred help print flag   */

/* ------------------------------------------------------------- tx helpers */
static void tx(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void print_banner(void)
{
    tx("\r\n=== STM32 Gripper Actuator Controller ===\r\n");
    tx("  h  home (seek stop, settle, zero)\r\n");
    tx("  o  open        c  close\r\n");
    tx("  [  min limit   ]  max limit\r\n");
    tx("  +  nudge open   -  nudge close\r\n");
    tx("  0-9 go to 0..90% of travel\r\n");
    tx("  s  stop/hold    x  disable(coast)   e  enable\r\n");
    tx("  p or ?  status\r\n\r\n");
}

static void print_status(void)
{
    char buf[96];
    /* All-integer formatting: no floating-point printf support required. */
    snprintf(buf, sizeof(buf),
             "STATE=%-13s TGT=%5ld POS=%5ld ERR=%5ld VEL=%6ld c/s PWM=%5ld\r\n",
             controller_state_name(),
             (long)controller_target(),
             (long)controller_position(),
             (long)controller_error(),
             (long)controller_velocity_cps(),
             (long)controller_pwm());
    tx(buf);
}

/* ------------------------------------------------------------------- init */
void comms_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = USART2_TX, PA3 = USART2_RX (AF7) */
    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = UART_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

    print_banner();

    /* Arm the first 1-byte receive. Re-armed after every byte in the ISR. */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* ---------------------------------------------------- command decode (ISR) */
static void handle_command(uint8_t c)
{
    switch (c) {
        case 'h': controller_home();               break;
        case 'o': controller_set_target(OPEN_POS);  break;
        case 'c': controller_set_target(CLOSE_POS); break;
        case '[': controller_set_target(POS_MIN);   break;
        case ']': controller_set_target(POS_MAX);   break;
        case '+':
        case '=': controller_nudge(+NUDGE_STEP);    break;
        case '-':
        case '_': controller_nudge(-NUDGE_STEP);    break;
        case 's': controller_stop();               break;
        case 'x': controller_disable();            break;
        case 'e': controller_enable();             break;
        case 'p':
        case '?': status_req = 1;                   break;

        default:
            if (c >= '0' && c <= '9') {
                /* map digit to a fraction of the travel range */
                int32_t span = POS_MAX - POS_MIN;
                int32_t t    = POS_MIN + (span * (c - '0')) / 9;
                controller_set_target(t);
            }
            break;
    }
}

/* HAL calls this from USART2_IRQHandler once a byte has arrived. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        handle_command(rx_byte);
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* re-arm */
    }
}

/* -------------------------------------------------- deferred work (main loop) */
void comms_service(void)
{
    if (banner_req) { banner_req = 0; print_banner(); }
    if (status_req) { status_req = 0; print_status(); }
}
