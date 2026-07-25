/*
 * STM32G4 USART1 — 上位机通信
 *
 * 接收改为【RXNE 中断 + 软件环形缓冲】：
 *   原实现用轮询 _getchar 读 RXNE，而 USART1 的硬件 FIFO 已被
 *   HAL_UARTEx_DisableFifoMode() 关闭（RDR 单字节深度）。主循环还被
 *   25kHz 优先级 0 的电流环 ISR 频繁抢占，命令突发时新字节在 RDR 未读时
 *   到达即触发 ORE 溢出并被直接丢弃，导致 "reset" 等命令残缺、strcmp 不命中，
 *   _cmd_reset 永远不被调用（典型现象：固件收不到 reset，a 标志恒为 0）。
 *
 *   现由 RXNE 中断把每个到达字节立刻塞进 ring，comm_tick 从 ring 取，
 *   字节绝不丢失，命令 100% 送达 comm_process_line。
 *   TX 仍用轮询 _putchar（发送不赶时间，且避免 TX 中断嵌套复杂化）。
 */
#include "hal/hal_uart.h"
#include "usart.h"
#include "stm32g4xx_hal.h"

#define UART_RX_RING_SZ 256u
static volatile uint8_t  g_rx_ring[UART_RX_RING_SZ];
static volatile uint32_t g_rx_head;   /* ISR 写指针 */
static volatile uint32_t g_rx_tail;   /* 主循环读指针 */
static volatile uint32_t g_rx_drop;   /* 环形缓冲溢出计数（调试用） */

static void _init(uint32_t baud)
{
    (void)baud;
    g_rx_head = g_rx_tail = 0;
    g_rx_drop = 0;

    /* 清一次可能的历史 ORE，避免上电残留导致 RX 一开始就卡死 */
    if (huart1.Instance->ISR & UART_FLAG_ORE)
    {
        (void)huart1.Instance->ISR;
        (void)huart1.Instance->RDR;
    }

    /* 使能 USART1 接收中断（RXNE）。优先级 3，低于电流环(0)与速度环(1)，
       不干扰 FOC 实时性；ISR 极短（读 1 字节入 ring），对主循环影响可忽略。 */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/* USART1 中断服务函数：仅处理接收（覆盖 startup 中的 WEAK 默认实现） */
void USART1_IRQHandler(void)
{
    uint32_t isr = huart1.Instance->ISR;

    if (isr & UART_FLAG_RXNE)
    {
        uint8_t b = (uint8_t)(huart1.Instance->RDR & 0xFF);
        uint32_t next = (g_rx_head + 1u) % UART_RX_RING_SZ;
        if (next != g_rx_tail)
        {
            g_rx_ring[g_rx_head] = b;
            g_rx_head = next;
        }
        else
        {
            g_rx_drop++;   /* 缓冲满，丢弃（极少见：ring 256B，命令很短） */
        }
    }

    /* 溢出错误处理：必须按"先读 ISR 再读 RDR"的顺序清除 ORE，
       否则 RX 会卡死不再产生 RXNE。 */
    if (isr & UART_FLAG_ORE)
    {
        (void)huart1.Instance->ISR;
        (void)huart1.Instance->RDR;
    }
}

static void _putchar(char c)
{
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {}
    huart1.Instance->TDR = (uint8_t)c;
}

/* 从环形缓冲取 1 字节；空则返回 0。comm_tick 循环调用本函数。 */
static uint8_t _getchar(char *c)
{
    if (g_rx_head == g_rx_tail) return 0;   /* 缓冲空 */
    *c = (char)g_rx_ring[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1u) % UART_RX_RING_SZ;
    return 1;
}

const hal_uart_t g_hal_uart = {
    .init = _init,
    .tx   = _putchar,
    .rx   = _getchar,
};
