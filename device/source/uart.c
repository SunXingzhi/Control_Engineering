/**
 * @file    uart.c
 * @brief   UART 驱动实现：单字节中断接收 + IDLE 帧检测 + 5ms 超时
 *
 * 接收机制：
 *   - 每收到 1 字节触发 HAL_UART_RxCpltCallback，存入缓冲区
 *   - IDLE 中断（总线空闲）或 5ms 超时 → 标记一帧接收完成
 *   - 两种方式均可触发，避免只依赖 IDLE 时关中断期间丢失的问题
 */

#include "../include/uart.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* 单字节接收暂存 */
static uint8_t uart_rx_byte;

/* 当前活跃 UART，供中断回调使用 */
static uart_base_t* uart_active;

#ifdef USE_FREERTOS
/* 接收完成时通知的任务句柄 */
static TaskHandle_t uart_notify_task = NULL;
#endif

/**
 * @brief  启动单字节中断接收链
 */
static void uart_start_rx(uart_base_t* uart)
{
	uart_active = uart;
	if (HAL_UART_Receive_IT(uart->huart, &uart_rx_byte, 1) != HAL_OK){
		uart->rx_done = 0;
		uart->rx_len = 0;
	}
}

/**
 * @brief  HAL 接收回调：逐字节存入缓冲区，溢出则丢弃
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (uart_active == NULL || huart != uart_active->huart){
		return;
	}

	uart_base_t* uart = uart_active;

	if (uart->rx_len < uart->rx_buf_size){
		uart->rx_buf[uart->rx_len] = uart_rx_byte;
		uart->rx_len++;
		uart->last_rx_tick = HAL_GetTick();
	}

	/* 继续接收下一字节（即使缓冲区满也不中断接收链） */
	HAL_UART_Receive_IT(uart->huart, &uart_rx_byte, 1);
}

/**
 * @brief  HAL 错误回调：重启接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
	if (uart_active != NULL && huart == uart_active->huart){
		uart_start_rx(uart_active);
	}
}

/**
 * @brief  IDLE 中断预处理，供 USARTx_IRQHandler 调用
 * @retval 1 表示已处理 IDLE，调用方跳过 HAL_UART_IRQHandler
 * @retval 0 表示非 IDLE 中断，调用方继续走 HAL_UART_IRQHandler
 *
 * 防止 RXNE+IDLE 竞态丢字节：
 *   末字节到达时两标志可能同时置位，若直接清 IDLE（读 DR）会丢失该字节。
 *   先判 RXNE：
 *     两标志均置位 → 交给 HAL 统一处理（读 DR 存字节并清标志）
 *     仅 IDLE 置位 → 手动清除（此时读 DR 无害）
 */
int uart_idle_hook(UART_HandleTypeDef* huart)
{
	if (uart_active == NULL || huart != uart_active->huart){
		return 0;
	}

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET
	    && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET){

		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET){
			/* RXNE 也置位：交给 HAL 处理（读 DR 存字节） */
			HAL_UART_IRQHandler(huart);
		} else {
			/* 仅 IDLE：手动清标志 */
			__HAL_UART_CLEAR_IDLEFLAG(huart);
		}

		if (uart_active->rx_len > 0){
			uart_active->rx_done = 1;
#ifdef USE_FREERTOS
			if (uart_notify_task != NULL){
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				vTaskNotifyGiveFromISR(uart_notify_task, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
#endif
		}
		return 1;
	}

	return 0;
}

/**
 * @brief  初始化 UART 驱动：使能 IDLE 中断，启动接收
 * @note   前置条件：CubeMX 已完成 GPIO、UART、NVIC 初始化
 */
device_err_t uart_init(uart_base_t* uart)
{
	if (uart == NULL || uart->huart == NULL || uart->rx_buf == NULL){
		return DRV_ERR_NULL;
	}

	UART_HandleTypeDef* huart = uart->huart;

	/* 使能 IDLE 中断（CubeMX/HAL 在 F1 上默认不开启） */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	uart->rx_len = 0;
	uart->rx_done = 0;

	uart_start_rx(uart);

	return DRV_OK;
}

/**
 * @brief  阻塞发送
 * @note   适用于低优先级任务，高优先级任务应改用 DMA 发送
 */
device_err_t uart_send(uart_base_t* uart, const uint8_t* data, uint16_t len)
{
	if (uart == NULL || data == NULL || len == 0){
		return DRV_ERR_NULL;
	}

	if (HAL_UART_Transmit(uart->huart, (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK){
		return DRV_ERR_IO;
	}

	return DRV_OK;
}

/**
 * @brief  读取已接收到的帧到用户缓冲区，读后清空
 * @retval 实际读取字节数（0 表示无新数据）
 */
uint16_t uart_recv(uart_base_t* uart, uint8_t* buf, uint16_t len)
{
	if (uart == NULL || buf == NULL || len == 0){
		return 0;
	}

	uint16_t copy_len = 0;

#ifdef USE_FREERTOS
	taskENTER_CRITICAL();
#else
	__disable_irq();
#endif

	/* 帧就绪条件：IDLE 标记 或 5ms 超时 */
	uint8_t frame_ready = uart->rx_done
	    || (uart->rx_len > 0 && (HAL_GetTick() - uart->last_rx_tick) >= 5);

	if (frame_ready && uart->rx_len > 0){
		copy_len = (uart->rx_len < len) ? uart->rx_len : len;
		for (uint16_t i = 0; i < copy_len; i++){
			buf[i] = uart->rx_buf[i];
		}
		uart->rx_len = 0;
		uart->rx_done = 0;
	}

#ifdef USE_FREERTOS
	taskEXIT_CRITICAL();
#else
	__enable_irq();
#endif

	return copy_len;
}

#ifdef USE_FREERTOS
/**
 * @brief  设置接收完成时通知的 FreeRTOS 任务
 */
void uart_set_notify_task(uart_base_t* uart, TaskHandle_t task)
{
	(void)uart;
	uart_notify_task = task;
}

/**
 * @brief  阻塞等待接收完成通知
 * @param  timeout_ms: 超时时间（ms），UINT32_MAX 表示永久等待
 * @retval 0 表示超时，>0 表示收到通知
 */
uint32_t uart_wait_rx(uint32_t timeout_ms)
{
	TickType_t ticks = (timeout_ms == UINT32_MAX)
		                   ? portMAX_DELAY
		                   : pdMS_TO_TICKS(timeout_ms);

	return ulTaskNotifyTake(pdTRUE, ticks);
}
#endif
