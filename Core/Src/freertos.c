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

/* USER CODE END Variables */
/* Definitions for usart_send_task */
osThreadId_t usart_send_taskHandle;
const osThreadAttr_t usart_send_task_attributes = {
  .name = "usart_send_task",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for usart_recv_task */
osThreadId_t usart_recv_taskHandle;
const osThreadAttr_t usart_recv_task_attributes = {
  .name = "usart_recv_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controller_task */
osThreadId_t controller_taskHandle;
const osThreadAttr_t controller_task_attributes = {
  .name = "controller_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for sensor_data_tas */
osThreadId_t sensor_data_tasHandle;
const osThreadAttr_t sensor_data_tas_attributes = {
  .name = "sensor_data_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for usart_recv_queue */
osMessageQueueId_t usart_recv_queueHandle;
const osMessageQueueAttr_t usart_recv_queue_attributes = {
  .name = "usart_recv_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void usart_send_task_func(void *argument);
void usart_recv_task_func(void *argument);
void controller_task_func(void *argument);
void sensor_data_updater_func(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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
  usart_recv_queueHandle = osMessageQueueNew (64, sizeof(uint8_t), &usart_recv_queue_attributes);

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
void usart_send_task_func(void *argument)
{
  /* USER CODE BEGIN usart_send_task_func */
	/* Infinite loop */
	for (;;){
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
void usart_recv_task_func(void *argument)
{
  /* USER CODE BEGIN usart_recv_task_func */
	/* Infinite loop */
	for (;;){

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
void controller_task_func(void *argument)
{
  /* USER CODE BEGIN controller_task_func */
	/* Infinite loop */
	for (;;){
		osDelay(1);
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
void sensor_data_updater_func(void *argument)
{
  /* USER CODE BEGIN sensor_data_updater_func */
	/* Infinite loop */
	for (;;){
		osDelay(1);
	}
  /* USER CODE END sensor_data_updater_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

