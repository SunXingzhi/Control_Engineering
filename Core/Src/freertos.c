/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "device.h"
#include "mt6701.h"
#include "angle_sensor.h"
#include "cmd_parser.h"
#include "adc.h"
#include "lqr_controller.h"
#include "lqr_controller_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uart_base_t uart1;
osMessageQueueId_t controller_task_handle;
volatile sensor_state_t g_sensor; // 共享变量
extern mt6701_t* g_dev; // 磁编码器变量
extern AngleSensor sensor1;
extern AngleSensor sensor2;
extern step_motor_t motor;
volatile lqr_controller_output_t g_lqr_output;
/* USER CODE END Variables */
/* Definitions for usart_send_task */
osThreadId_t usart_send_taskHandle;
const osThreadAttr_t usart_send_task_attributes = {
	.name = "usart_send_task",
	.stack_size = 192 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for usart_recv_task */
osThreadId_t usart_recv_taskHandle;
const osThreadAttr_t usart_recv_task_attributes = {
	.name = "usart_recv_task",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for controller_task */
osThreadId_t controller_taskHandle;
const osThreadAttr_t controller_task_attributes = {
	.name = "controller_task",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for sensor_data_tas */
osThreadId_t sensor_data_tasHandle;
const osThreadAttr_t sensor_data_tas_attributes = {
	.name = "sensor_data_tas",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for usart_recv_queue */
osMessageQueueId_t usart_recv_queueHandle;
const osMessageQueueAttr_t usart_recv_queue_attributes = {
	.name = "usart_recv_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void usart_send_task_func(void* argument);
void usart_recv_task_func(void* argument);
void controller_task_func(void* argument);
void sensor_data_updater_func(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of usart_recv_queue */
	usart_recv_queueHandle = osMessageQueueNew(4, sizeof(cmd_t), &usart_recv_queue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of usart_send_task */
	usart_send_taskHandle = osThreadNew(usart_send_task_func, NULL, &usart_send_task_attributes);

	/* creation of usart_recv_task */
	usart_recv_taskHandle = osThreadNew(usart_recv_task_func, NULL, &usart_recv_task_attributes);

	/* creation of controller_task */
	controller_taskHandle = osThreadNew(controller_task_func, NULL, &controller_task_attributes);

	/* creation of sensor_data_tas */
	sensor_data_tasHandle = osThreadNew(sensor_data_updater_func, NULL, &sensor_data_tas_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	// 在这里注册控制任务句柄(优先级比控制任务高, 先初始化, 没有句柄变量未定义的问题)
	controller_task_handle = controller_taskHandle;
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_usart_send_task_func */
/**
  * @brief  Function implementing the usart_send_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_usart_send_task_func */
void usart_send_task_func(void* argument)
{
	/* USER CODE BEGIN usart_send_task_func */
	/* Infinite loop */
	for (;;){
		// 直接发送要显示的信息.

		osDelay(1);
	}
	/* USER CODE END usart_send_task_func */
}

/* USER CODE BEGIN Header_usart_recv_task_func */
/**
* @brief Function implementing the usart_recv_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usart_recv_task_func */
void usart_recv_task_func(void* argument)
{
	/* USER CODE BEGIN usart_recv_task_func */
	// 注册任务通知函数
	uart_set_notify_task(&uart1, osThreadGetId());

	/* Infinite loop */
	for (;;){
		// 当串口接收中断发送通知后, 该任务开始运行
		uart_wait_rx(UINT32_MAX);
		/**循环解析命令行命令
		 * 主要功能: 解析出不同命令后将数据发送到串口消息队列中, 控制器命令设置一定的阻塞时间来实现读取, 解析命令.
		 *
		 */
		cmd_parser_process();
		osDelay(1);
	}
	/* USER CODE END usart_recv_task_func */
}

/* USER CODE BEGIN Header_controller_task_func */
/**
* @brief Function implementing the controller_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controller_task_func */
void controller_task_func(void* argument)
{
	/* USER CODE BEGIN controller_task_func */
	cmd_t cmd_data = {0};
	lqr_controller_t controller;
	lqr_controller_init(&controller);
	/* Infinite loop */
	for (;;){
		/* 传感器通知定义控制周期，有限等待用于检测传感器任务超时。 */
		const uint32_t notification_count = ulTaskNotifyTake(
			pdTRUE,
			pdMS_TO_TICKS(LQR_SENSOR_TIMEOUT_MS));

		/* 扫描当前命令，停机与停用命令由高优先级队列优先返回。 */
		cmd_data.id = CMD_NONE;
		osMessageQueueGet(usart_recv_queueHandle, &cmd_data, 0, 0);
		switch (cmd_data.id){
		case CMD_LQR_ENABLE:
			lqr_controller_handle_command(&controller, LQR_COMMAND_ENABLE);
			break;
		case CMD_LQR_DISABLE:
			lqr_controller_handle_command(&controller, LQR_COMMAND_DISABLE);
			break;
		case CMD_STOP:
			lqr_controller_handle_command(&controller, LQR_COMMAND_DISABLE);
			step_motor_stop(&motor);
			break;
		case CMD_LQR_RESET:
			lqr_controller_handle_command(&controller, LQR_COMMAND_RESET_FAULT);
			break;
		default:
			break;
		}

		sensor_state_t sensor_snapshot;
		taskENTER_CRITICAL();
		sensor_snapshot = g_sensor;
		taskEXIT_CRITICAL();

		lqr_controller_output_t output = lqr_controller_update(
			&controller,
			&sensor_snapshot,
			notification_count != 0);
		g_lqr_output = output;
		lqr_controller_apply_motor_output(&output, &motor);
	}
	/* USER CODE END controller_task_func */
}

/* USER CODE BEGIN Header_sensor_data_updater_func */
/**
* @brief Function implementing the sensor_data_upd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_data_updater_func */
void sensor_data_updater_func(void* argument)
{
	/* USER CODE BEGIN sensor_data_updater_func */
	/* Infinite loop */
	TickType_t init_tick = xTaskGetTickCount();
	float theta1_prev = 0, theta2_prev = 0, x_prev = 0;
	float theta1_dot = 0, theta2_dot = 0, x_dot = 0;

	for (;;){
		// 设置固定更新周期:2ms, 与控制周期一致
		vTaskDelayUntil(&init_tick, pdMS_TO_TICKS(2));
		// 读取角度传感器（PA0=一级摆, PA1=二级摆，都在 ADC1 不同通道）
		float theta1 = AngleSensor_GetFilteredAngle(&sensor1);
		float theta2 = AngleSensor_GetFilteredAngle(&sensor2);

		float x_cart = g_dev->sensor.angle_total;
		float x_speed = g_dev->sensor.speed;

		// 没有获取角速度接口, 这里直接计算
		float dt = 0.002f;
		float alpha = 0.3f;
		float theta1_dot_raw = (theta1 - theta1_prev) / dt;
		float theta2_dot_raw = (theta2 - theta2_prev) / dt;

		theta1_dot = alpha * theta1_dot_raw + (1 - alpha) * theta1_dot;
		theta2_dot = alpha * theta2_dot_raw + (1 - alpha) * theta2_dot;

		theta1_prev = theta1;
		theta2_prev = theta2;

		// 写入共享结构体
		taskENTER_CRITICAL();
		g_sensor.theta1 = theta1;
		g_sensor.theta2 = theta2;
		g_sensor.theta1_dot = theta1_dot;
		g_sensor.theta2_dot = theta2_dot;
		g_sensor.x_cart = x_cart;
		g_sensor.x_dot = x_dot;
		taskEXIT_CRITICAL();

		// TODO 通知控制任务(这里freertos 和 CMSIS V2兼容曾混用不知道是否有问题)
		xTaskNotifyGive(controller_taskHandle);
	}
	/* USER CODE END sensor_data_updater_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
