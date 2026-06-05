/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_step_motor.h"
#include "mt6701.h"
#define TIM_TABLE_ERROR_INDEX		0XFF
#define TIM_TO_TABLE_INDEX(htim)	(((htim)->Instance == TIM1)? 0: \
((htim)->Instance == TIM2)? 1: \
((htim)->Instance == TIM3)? 2: \
((htim)->Instance == TIM4)? 3: TIM_TABLE_ERROR_INDEX)

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  step_motor_t*   motor;
  // mt6701_t*       sensor;
} tim_callback_entry_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void USART1_IRQHandler(void);
/* USER CODE BEGIN EFP */
step_motor_t* find_step_motor_from_tim_table(TIM_HandleTypeDef* htim);
void tim_register_motor(TIM_HandleTypeDef* htim, step_motor_t* motor);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
