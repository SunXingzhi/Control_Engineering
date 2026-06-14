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
#include "auto_tune.h"
#include "device.h"
#include "tim.h"
#include "pendulum.h"
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
volatile uint8_t g_limit_right_flag = 0;
volatile uint8_t g_limit_left_flag = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern PID_AutoTune_t tuner;

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

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MOTOR_LIMIT_CLOSE_Pin);
  HAL_GPIO_EXTI_IRQHandler(MOTOR_LIMIT_REMOTE_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
extern mt6701_t encoder;
static tim_callback_entry_t callback_table[8] = {0};

extern mt6701_t* g_dev;

/**
 * @brief  EXTI 中断回调（HAL_GPIO_EXTI_IRQHandler 触发后自动调用）
 *         限位开关压下时 IO 为 LOW（下降沿），松开时为 HIGH（上升沿）
 *         仅在下降沿（按下）时置标志，上升沿（松开）忽略
 *         带 20ms 消抖，兼容裸机和 FreeRTOS（HAL_GetTick() 两者通用）
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_tick_close  = 0;
    static uint32_t last_tick_remote = 0;
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == MOTOR_LIMIT_CLOSE_Pin) {
        // PA11 右限位：按下时为 LOW
        if (now - last_tick_close < LIMIT_DEBOUNCE_MS) return;  // 消抖
        last_tick_close = now;
        if (HAL_GPIO_ReadPin(MOTOR_LIMIT_CLOSE_GPIO_Port, MOTOR_LIMIT_CLOSE_Pin) == GPIO_PIN_RESET) {
            g_limit_right_flag = 1;
        }
    }
    else if (GPIO_Pin == MOTOR_LIMIT_REMOTE_Pin) {
        // PA12 左限位：按下时为 LOW
        if (now - last_tick_remote < LIMIT_DEBOUNCE_MS) return;  // 消抖
        last_tick_remote = now;
        if (HAL_GPIO_ReadPin(MOTOR_LIMIT_REMOTE_GPIO_Port, MOTOR_LIMIT_REMOTE_Pin) == GPIO_PIN_RESET) {
            g_limit_left_flag = 1;
        }
    }
}

/* ======================== 注册 / 查找 ======================== */

void tim_register_motor(TIM_HandleTypeDef* htim, step_motor_t* motor)
{
	uint8_t index = TIM_TO_TABLE_INDEX(htim);
	if (index != TIM_TABLE_ERROR_INDEX){
		callback_table[index].motor = motor;
	}
}

static step_motor_t* find_motor_by_tim(TIM_HandleTypeDef* htim)
{
	uint8_t index = TIM_TO_TABLE_INDEX(htim);
	return callback_table[index].motor;
}

/* ======================== TIM4: 步数限位 ======================== */

/**
 * @brief  TIM4 更新中断处理（步数限位计数）
 * @note   仅在 move_angle 设置 step_remaining 后才计数
 */
static void tim4_step_counter_isr(TIM_HandleTypeDef* htim)
{
	step_motor_t* motor = find_motor_by_tim(htim);
	if (motor == NULL) return;

	step_motor_information_t* info = &motor->step_motor_information;
	if (info->step_remaining > 0){
		info->step_remaining--;
		if (info->step_remaining == 0){
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
			step_motor_pwm_off(motor);
		}
	}
}

/* ======================== HAL 回调入口 ======================== */

/**
 * @brief  TIM 更新中断回调（由 HAL_TIM_IRQHandler 调用）
 * @param  htim: 触发中断的定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// TIM4 → 步数限位
	if (htim->Instance == TIM4){
		tim4_step_counter_isr(htim);
	}
	// TIM3 → 编码器采样(周期1ms) + PID控制(周期2ms)
	else if (htim->Instance == TIM3){
		encoder_update_speed();
		step_motor_t* motor = find_motor_by_tim(htim);
		if (motor == NULL) return;
		// 更新电机当前频率
		motor->step_motor_information.current_frequency	= motor_speed_to_freq(g_dev->sensor.speed,
											motor->step_motor_information.step_model);
#if USE_MOTOR_PID_CONTROL==1
		pid_control_tick(find_motor_by_tim(htim));
#endif
		// 波形输出：打印目标值和实际值
		extern volatile uint8_t g_wave_ready;
		extern volatile float g_wave_target;
		extern volatile float g_wave_actual;
		// extern volatile uint8_t auto_tune_active;
		static volatile uint8_t tick = 0;
		if (++tick >= 5){
			tick = 0;
			// extern PID_AutoTune_t tuner;
			// g_wave_target = auto_tune_active ? tuner.setpoint : motor->motor_pid.Target;
			g_wave_actual = g_dev->sensor.speed;
			g_wave_ready = 1;
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
