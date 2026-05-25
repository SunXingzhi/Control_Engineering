#include "uart.h"

/* 单字节接收暂存 */
static uint8_t uart_rx_byte;

/* 当前活跃 UART，供中断回调使用 */
static uart_base_t* uart_active;

/* 启动单字节中断接收链 */
static void uart_start_rx(uart_base_t* uart)
{
    uart_active = uart;
    if (HAL_UART_Receive_IT(uart->huart, &uart_rx_byte, 1) != HAL_OK) {
        uart->rx_done = 0;
        uart->rx_len  = 0;
    }
}

/* HAL 接收回调：逐字节存入缓冲，溢出则丢弃 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (uart_active == NULL || huart != uart_active->huart) {
        return;
    }

    uart_base_t* uart = uart_active;

    if (uart->rx_len < uart->rx_buf_size) {
        uart->rx_buf[uart->rx_len] = uart_rx_byte;
        uart->rx_len++;
    }

    HAL_UART_Receive_IT(uart->huart, &uart_rx_byte, 1);
}

/* HAL 错误回调：重启接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (uart_active != NULL && huart == uart_active->huart) {
        uart_start_rx(uart_active);
    }
}

/* IDLE 中断预处理，供 USART1_IRQHandler 调用
 * 返回 1 表示已处理 IDLE，调用方跳过 HAL_UART_IRQHandler
 * 返回 0 表示非 IDLE 中断，调用方继续走 HAL_UART_IRQHandler
 *
 * 防止 RXNE+IDLE 竞态丢字节：末字节到达时两标志可能同时置位，
 * 若直接清 IDLE（读 DR）会丢失该字节。先判 RXNE：
 *   两标志均置位 → 交 HAL 统一处理（读 DR 存字节并清标志）
 *   仅 IDLE 置位 → 手动清除（此时读 DR 无害） */
int uart_idle_hook(UART_HandleTypeDef* huart)
{
    if (uart_active == NULL || huart != uart_active->huart) {
        return 0;
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET
        && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET) {

        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) {
            HAL_UART_IRQHandler(huart);
        } else {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
        }

        if (uart_active->rx_len > 0) {
            uart_active->rx_done = 1;
        }
        return 1;
    }

    return 0;
}

/* 初始化驱动：使能 IDLE 中断，启动接收
 * 前置条件：CubeMX 已完成 GPIO、UART、NVIC 初始化
 *           且已调用 MX_USART1_UART_Init() */
device_err_t uart_init(uart_base_t* uart)
{
    if (uart == NULL || uart->huart == NULL || uart->rx_buf == NULL) {
        return DRV_ERR_NULL;
    }

    UART_HandleTypeDef* huart = uart->huart;

    /* 使能 IDLE 中断（CubeMX/HAL 在 F1 上默认不开启） */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    uart->rx_len  = 0;
    uart->rx_done = 0;

    uart_start_rx(uart);

    return DRV_OK;
}

/* 阻塞发送 */
device_err_t uart_send(uart_base_t* uart, const uint8_t* data, uint16_t len)
{
    if (uart == NULL || data == NULL || len == 0) {
        return DRV_ERR_NULL;
    }

    if (HAL_UART_Transmit(uart->huart, (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK) {
        return DRV_ERR_IO;
    }

    return DRV_OK;
}

/* 读取已接收帧到用户缓冲区，读后清空 */
uint16_t uart_recv(uart_base_t* uart, uint8_t* buf, uint16_t len)
{
    if (uart == NULL || buf == NULL || len == 0) {
        return 0;
    }

    uint16_t copy_len = 0;

    __disable_irq();
    if (uart->rx_done && uart->rx_len > 0) {
        copy_len = (uart->rx_len < len) ? uart->rx_len : len;
        for (uint16_t i = 0; i < copy_len; i++) {
            buf[i] = uart->rx_buf[i];
        }
        uart->rx_len  = 0;
        uart->rx_done = 0;
    }
    __enable_irq();

    return copy_len;
}
