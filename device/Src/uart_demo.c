/* uart_demo.c — 串口驱动使用例程
 * 需要配合 CubeMX 工程使用 */

#include "uart.h"
#include <string.h>

/* CubeMX 生成的句柄 */
extern UART_HandleTypeDef huart1;

/* 接收缓冲 */
static uint8_t rx_buf[UART_RX_BUF_SIZE];

/* 驱动实例 */
static uart_base_t uart1 = {
    .huart       = &huart1,
    .rx_buf      = rx_buf,
    .rx_buf_size = UART_RX_BUF_SIZE,
};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();  /* CubeMX 生成，必须在 uart_init 之前 */

    uart_init(&uart1);

    /* 发送字符串 */
    uart_send(&uart1, (uint8_t *)"hello\r\n", 7);

    while (1)
    {
        /* 收到一帧数据后回显 */
        if (uart1.rx_done) {
            uint8_t buf[256];
            uint16_t n = uart_recv(&uart1, buf, sizeof(buf));
            if (n > 0) {
                uart_send(&uart1, buf, n);
            }
        }
    }
}
