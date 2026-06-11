#ifndef DEVICE_UART_H
#define DEVICE_UART_H

#include "stm32f1xx_hal.h"
#include "device.h"
// test1
/* 椹卞姩鍐呴儴缂撳啿澶у皬锛堜笌 CubeMX 閰嶇疆鏃犲叧锛?*/
#define UART_RX_BUF_SIZE 256U

/* UART 纭欢鎶借薄灞?
 * huart 鐢?CubeMX 鐢熸垚锛岄┍鍔ㄥ彧璐熻矗缂撳啿鍜屾敹鍙戦€昏緫 */
typedef struct uart_base
{
        UART_HandleTypeDef *huart; /* CubeMX 鐢熸垚鐨勫彞鏌?*/
        uint8_t *rx_buf;           /* 鎺ユ敹缂撳啿 */
        uint16_t rx_buf_size;      /* 缂撳啿瀹归噺 */
        volatile uint16_t rx_len;  /* 宸叉帴鏀跺瓧鑺傛暟 */
        volatile uint8_t rx_done;  /* 甯у畬鎴愭爣蹇?*/
        volatile uint32_t last_rx_tick; /* 最后字节到达的 tick */
} uart_base_t;

/* API */
device_err_t uart_init(uart_base_t *uart);

device_err_t uart_send(uart_base_t *uart, const uint8_t *data, uint16_t len);

uint16_t uart_recv(uart_base_t *uart, uint8_t *buf, uint16_t len);

int uart_idle_hook(UART_HandleTypeDef *huart);

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"

/* RTOS锛氳缃帴鏀跺畬鎴愭椂閫氱煡鐨勪换鍔?*/
void uart_set_notify_task(uart_base_t *uart, TaskHandle_t task);

/* RTOS锛氶樆濉炵瓑寰呬竴甯ф暟鎹埌杈撅紙涓嶅崰 CPU锛夛紝杩斿洖鍚庤皟鐢?uart_recv 璇绘暟鎹?*/
uint32_t uart_wait_rx(uint32_t timeout_ms);
#endif

#endif /* DEVICE_UART_H */
