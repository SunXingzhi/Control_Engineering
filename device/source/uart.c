#include "../include/uart.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* 鍗曞瓧鑺傛帴鏀舵殏瀛?*/
static uint8_t uart_rx_byte;

/* 褰撳墠娲昏穬 UART锛屼緵涓柇鍥炶皟浣跨敤 */
static uart_base_t* uart_active;

#ifdef USE_FREERTOS
/* 鎺ユ敹瀹屾垚鏃堕€氱煡鐨勪换鍔″彞鏌?*/
static TaskHandle_t uart_notify_task = NULL;
#endif

/* 鍚姩鍗曞瓧鑺備腑鏂帴鏀堕摼 */
static void uart_start_rx(uart_base_t* uart)
{
	uart_active = uart;
	if (HAL_UART_Receive_IT(uart->huart, &uart_rx_byte, 1) != HAL_OK){
		uart->rx_done = 0;
		uart->rx_len = 0;
	}
}

/* HAL 鎺ユ敹鍥炶皟锛氶€愬瓧鑺傚瓨鍏ョ紦鍐诧紝婧㈠嚭鍒欎涪寮?*/
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

	HAL_UART_Receive_IT(uart->huart, &uart_rx_byte, 1);
}

/* HAL 閿欒鍥炶皟锛氶噸鍚帴鏀?*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
	if (uart_active != NULL && huart == uart_active->huart){
		uart_start_rx(uart_active);
	}
}

/* IDLE 涓柇棰勫鐞嗭紝渚?USART1_IRQHandler 璋冪敤
 * 杩斿洖 1 琛ㄧず宸插鐞?IDLE锛岃皟鐢ㄦ柟璺宠繃 HAL_UART_IRQHandler
 * 杩斿洖 0 琛ㄧず闈?IDLE 涓柇锛岃皟鐢ㄦ柟缁х画璧?HAL_UART_IRQHandler
 *
 * 闃叉 RXNE+IDLE 绔炴€佷涪瀛楄妭锛氭湯瀛楄妭鍒拌揪鏃朵袱鏍囧織鍙兘鍚屾椂缃綅锛?
 * 鑻ョ洿鎺ユ竻 IDLE锛堣 DR锛変細涓㈠け璇ュ瓧鑺傘€傚厛鍒?RXNE锛?
 *   涓ゆ爣蹇楀潎缃綅 鈫?浜?HAL 缁熶竴澶勭悊锛堣 DR 瀛樺瓧鑺傚苟娓呮爣蹇楋級
 *   浠?IDLE 缃綅 鈫?鎵嬪姩娓呴櫎锛堟鏃惰 DR 鏃犲锛?*/
int uart_idle_hook(UART_HandleTypeDef* huart)
{
	if (uart_active == NULL || huart != uart_active->huart){
		return 0;
	}

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) !=
		RESET){
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET){
			HAL_UART_IRQHandler(huart);
		}
		else{
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

/* 鍒濆鍖栭┍鍔細浣胯兘 IDLE 涓柇锛屽惎鍔ㄦ帴鏀?
 * 鍓嶇疆鏉′欢锛欳ubeMX 宸插畬鎴?GPIO銆乁ART銆丯VIC 鍒濆鍖?
 *           涓斿凡璋冪敤 MX_USART1_UART_Init() */
device_err_t uart_init(uart_base_t* uart)
{
	if (uart == NULL || uart->huart == NULL || uart->rx_buf == NULL){
		return DRV_ERR_NULL;
	}

	UART_HandleTypeDef* huart = uart->huart;

	/* 浣胯兘 IDLE 涓柇锛圕ubeMX/HAL 鍦?F1 涓婇粯璁や笉寮€鍚級 */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	uart->rx_len = 0;
	uart->rx_done = 0;

	uart_start_rx(uart);

	return DRV_OK;
}

/* 闃诲鍙戦€?*/
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

/* 璇诲彇宸叉帴鏀跺抚鍒扮敤鎴风紦鍐插尯锛岃鍚庢竻绌?*/
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
void uart_set_notify_task(uart_base_t* uart, TaskHandle_t task)
{
	(void)uart;
	uart_notify_task = task;
}

uint32_t uart_wait_rx(uint32_t timeout_ms)
{
	TickType_t ticks = (timeout_ms == UINT32_MAX)
		                   ? portMAX_DELAY
		                   : pdMS_TO_TICKS(timeout_ms);

	return ulTaskNotifyTake(pdTRUE, ticks);
}
#endif
