/**
 * @file    uart.h
 * @brief   UART 驱动 —— 单字节中断接收 + IDLE 帧检测
 *
 * 接收流程：
 *   1. HAL_UART_Receive_IT 逐字节中断接收，存入 rx_buf
 *   2. IDLE 中断（总线空闲）或 5ms 超时 → 标记 rx_done = 1
 *   3. 主循环/任务调用 uart_recv() 读取数据
 *
 * FreeRTOS 支持：
 *   - uart_set_notify_task() 注册任务句柄
 *   - IDLE 中断自动 vTaskNotifyGiveFromISR() 唤醒任务
 *   - uart_wait_rx() 阻塞等待接收完成
 */

#ifndef DEVICE_UART_H
#define DEVICE_UART_H

#include "device.h"

#ifdef USE_FREERTOS
	#if defined(USE_CMSIS_V2_OS)
		#include "cmsis_os2.h"
	#else
		#include "FreeRTOS.h"
		#include "task.h"
	#endif
#endif

/* ======================== 接收缓冲区大小 ======================== */
#define UART_RX_BUF_SIZE	256

/* ======================== UART 设备结构体 ======================== */
typedef struct uart_base {
	UART_HandleTypeDef* huart;		/* HAL UART 句柄 */
	uint8_t  rx_buf[UART_RX_BUF_SIZE];	/* 接收缓冲区 */
	uint16_t rx_buf_size;			/* 缓冲区大小 */
	volatile uint16_t rx_len;		/* 当前已接收字节数 */
	volatile uint8_t  rx_done;		/* 帧接收完成标志（IDLE 或超时触发） */
	volatile uint32_t last_rx_tick;		/* 最后一次接收的 HAL_GetTick 时间戳 */
} uart_base_t;

/* ======================== API ======================== */

/**
 * @brief  初始化 UART 驱动：使能 IDLE 中断，启动接收
 * @note   前置条件：CubeMX 已完成 GPIO、UART、NVIC 初始化
 */
device_err_t uart_init(uart_base_t* uart);

/**
 * @brief  阻塞发送（适用于低优先级任务）
 */
device_err_t uart_send(uart_base_t* uart, const uint8_t* data, uint16_t len);

/**
 * @brief  读取已接收到的帧到用户缓冲区，读后清空
 * @retval 实际读取字节数（0 表示无新数据）
 */
uint16_t uart_recv(uart_base_t* uart, uint8_t* buf, uint16_t len);

#ifdef USE_FREERTOS
/**
 * @brief  设置接收完成时通知的 FreeRTOS 任务
 */
void uart_set_notify_task(uart_base_t* uart, TaskHandle_t task);

/**
 * @brief  阻塞等待接收完成通知
 * @param  timeout_ms: 超时时间（ms），UINT32_MAX 表示永久等待
 * @retval 0 表示超时，>0 表示收到通知
 */
uint32_t uart_wait_rx(uint32_t timeout_ms);
#endif

/**
 * @brief  IDLE 中断预处理，供 USARTx_IRQHandler 调用
 * @retval 1 表示已处理 IDLE，调用方跳过 HAL_UART_IRQHandler
 * @retval 0 表示非 IDLE 中断，调用方继续走 HAL_UART_IRQHandler
 */
int uart_idle_hook(UART_HandleTypeDef* huart);

#endif /* DEVICE_UART_H */
