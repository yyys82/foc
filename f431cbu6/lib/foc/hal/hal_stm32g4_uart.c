/*
 * STM32G4 USART1 轮询 (printf + 上位机)
 */
#include "hal/hal_uart.h"
#include "usart.h"

static void _init(uint32_t baud) { (void)baud; }

static void _putchar(char c)
{
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {}
    huart1.Instance->TDR = (uint8_t)c;
}

static uint8_t _getchar(char *c)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        *c = (uint8_t)(huart1.Instance->RDR & 0xFF);
        return 1;
    }
    return 0;
}

const hal_uart_t g_hal_uart = {
    .init    = _init,
    .tx = _putchar,
    .rx = _getchar,
};
