#ifndef DEVICE_UART_H
#define DEVICE_UART_H

#include "stm32f1xx_hal.h"
#include "device_err.h"

/* 驱动内部缓冲大小（与 CubeMX 配置无关） */
#define UART_RX_BUF_SIZE        256U

/* UART 硬件抽象层
 * huart 由 CubeMX 生成，驱动只负责缓冲和收发逻辑 */
typedef struct uart_base {
    UART_HandleTypeDef* huart;          /* CubeMX 生成的句柄 */
    uint8_t*            rx_buf;         /* 接收缓冲 */
    uint16_t            rx_buf_size;    /* 缓冲容量 */
    volatile uint16_t   rx_len;         /* 已接收字节数 */
    volatile uint8_t    rx_done;        /* 帧完成标志 */
} uart_base_t;

/* API */
device_err_t uart_init(uart_base_t* uart);
device_err_t uart_send(uart_base_t* uart, const uint8_t* data, uint16_t len);
uint16_t     uart_recv(uart_base_t* uart, uint8_t* buf, uint16_t len);
int          uart_idle_hook(UART_HandleTypeDef* huart);

#endif /* DEVICE_UART_H */
