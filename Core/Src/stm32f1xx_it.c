/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../device/include/uart.h"
#include "driver_step_motor.h"
#include "PID.h"
#include "mt6701.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1){
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	HAL_SYSTICK_Callback();
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if (uart_idle_hook(&huart1)) return;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
extern mt6701_t* encoder;
// 创建TIM回调对应功能实例
static tim_callback_entry_t callback_table[8] = {0}; // 最多8个TIM实例, TIM1-TIM8

extern mt6701_t* g_dev;

void tim_register_motor(TIM_HandleTypeDef* htim, step_motor_t* motor)
{
	uint8_t index = TIM_TO_TABLE_INDEX(htim);
	if (index != TIM_TABLE_ERROR_INDEX){
		callback_table[index].motor = motor;
	}
}

step_motor_t* find_step_motor_from_tim_table(TIM_HandleTypeDef* htim)
{
	// 获取对应索引
	uint8_t index = TIM_TO_TABLE_INDEX(htim);
	return callback_table[index].motor;
}

/**
 * @brief TIM中断回调函数. 目前步进电机用到了TIM3 CH1作为PWM输出, 同时编码器使用TIM4普通计时器模式来计算编码器捕获到的角速度.
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	angle_sensor_t* s = NULL;
	step_motor_t* motor = find_step_motor_from_tim_table(htim);
	// 电机部分: TIM4 更新中断 → 步数限位计数（开环/闭环通用）
	if (htim->Instance == TIM4){
		if (motor != NULL){
			step_motor_information_t* info = &motor->step_motor_information;

			if (info->step_remaining > 0){
				info->step_remaining--;
			}

			if (info->step_remaining == 0){
				// 目标步数到达：关中断 + 关 PWM
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
				step_motor_pwm_off(motor);
			}
		}
	}
	// 编码器部分
	else if (htim->Instance == TIM3){
		if (g_dev == NULL){
			return;
		}
		if (htim->Instance == g_dev->sensor.htim->Instance){
			s = &g_dev->sensor;

			if (s->first_sample){
				s->first_sample = 0;
				s->angle_last = s->angle;
			}

			float diff = cycle_diff(s->angle - s->angle_last, MT6701_ANGLE_MAX);
			__disable_irq(); // 禁用中断
			s->angle_last = s->angle;
			s->speed = diff * speed_calc_freq;
			s->speed_raw = (int32_t)(s->speed * 100.0f * (180.0f / 3.1415926f));
			__enable_irq(); // 使能中断
		}

		// PID部分
		// 计算PID参数
		if (s != NULL || motor != NULL){
			const float output = PID_calc(&motor->motor_pid, s->speed);

			if (output == 0){
				motor->step_motor_information.dir = STOP_DIR;
			}

			else if (output < 0){
				motor->motor_pid.Output = -output;

				motor->step_motor_information.dir = NEGATIVE_DIR;
			}

			step_motor_set_speed(motor, motor->motor_pid.Output, motor->step_motor_information.dir);
	}


	}
}

#if !defined(USE_FREE_RTOS)
	#if USE_MOTOR_PID_CONTROL==0
		/**
		 * @brief  SysTick 每 1ms 回调 → 每 5ms 驱动一次斜坡状态机
		 *         HAL_IncTick() → HAL_SYSTICK_Callback() 由中断自动调用
		 */
		extern step_motor_t motor;
		void HAL_SYSTICK_Callback(void)
		{
			static uint8_t tick_cnt = 0;
			if (++tick_cnt >= 5){
				tick_cnt = 0;
				ramp_step_motor_tick(&motor.ramp, &motor);
			}
		}
	#elif USE_MOTOR_PID_CONTROL==1
		void HAL_SYSTICK_Callback(void)
		{
			// 相关裸机...

		}
	#endif
#endif
/* USER CODE END 1 */
