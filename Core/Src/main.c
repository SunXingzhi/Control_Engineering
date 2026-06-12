/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../device/include/driver_step_motor.h"
#include "../../device/include/test_step_motor.h"
#include "../../device/include/test_cmd_motor.h"
#include "../../device/include/auto_tune.h"
#include "../../device/include/mt6701.h"
#include "../../device/include/uart.h"
#include "mt6701.h"
#include "angle_sensor.h"
#include "pendulum.h"
#include "cmd_parser.h"
#include "stm32f1xx_it.h"
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

/* USER CODE BEGIN PV */

static uart_base_t uart1 = {
	.huart = &huart1,
	.rx_buf_size = UART_RX_BUF_SIZE,
	.rx_buf = {0},
};
static tim_callback_entry_t callback_table[8] = {0};
extern mt6701_t* g_dev;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
step_motor_t motor = {
	HR4988,
	{
		.dir_gpio_pin = GPIO_PIN_5,
		.dir_gpio_port = GPIOB,
		.step_gpio_port = MOTOR_STEP_PIN_GPIO_Port,
		.step_gpio_pin = MOTOR_STEP_PIN_Pin,
		.ms1_gpio_port = MOTOR_MS1_PIN_GPIO_Port,
		.ms1_gpio_pin = MOTOR_MS1_PIN_Pin,
		.ms2_gpio_port = MOTOR_MS2_PIN_GPIO_Port,
		.ms2_gpio_pin = MOTOR_MS2_PIN_Pin,
		.ms3_gpio_port = MOTOR_MS3_PIN_GPIO_Port,
		.ms3_gpio_pin = MOTOR_MS3_PIN_Pin,
		.tim_handle = &htim4,
		.tim_channel = TIM_CHANNEL_1
	},
	{
		.current_frequency = 0,
		.step_model = DEFAULT_STEP,
		.dir = POSITIVE_DIR,
	}
};

// MT6701 磁编码器实例
mt6701_t encoder = {
	.sensor = {
		.hspi = &hspi1,
		.cs_gpiox = MT6701_CSN_GPIO_Port,
		.cs_gpio_pin = MT6701_CSN_Pin,
		.htim = &htim3,
	},
};

/* 角度传感器实例（文件作用域，供 pendulum.c extern 引用）*/
AngleSensor sensor1 = {0};

// 自动调参实例（ISR 读写，需通过临界区保护多字节访问）
volatile PID_AutoTune_t tuner;
volatile uint8_t auto_tune_active = 0; // 1=调参模式, 0=正常PID模式

// 波形数据共享变量（ISR 写，主循环读）
volatile float g_wave_target = 0;
volatile float g_wave_actual = 0;
volatile uint8_t g_wave_ready = 0;

/* 起摆上下文 */
pendulum_ctx_t pendulum = {0};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	uart_init(&uart1);

	if (step_motor_init(&motor) != DRV_OK){
		Error_Handler();
	}

	// 初始化 MT6701 磁编码器
	if (angle_sensor_init(&encoder) != DRV_OK){
		Error_Handler();
	}

	// 注册电机到 TIM 回调表（TIM4 中断能找到 motor 实例）
	// tim_register_motor(&htim4, &motor);

	// 初始化自动调参（默认关闭，通过串口命令启动）
	PID_AutoTune_Init((PID_AutoTune_t*)&tuner,
	                  PRESET_AUTOTUNE_AMPLITUDE,
	                  PRESET_AUTOTUNE_HYSTERESIS,
	                  PRESET_AUTOTUNE_SETPOINT,
	                  PRESET_AUTOTUNE_CYCLES);

	// 初始化串口命令测试
	test_cmd_motor_init(&motor, &uart1);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1){
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// 		test_cmd_motor_loop();
		//
		// 		// 波形输出（主循环打印，不阻塞 ISR）
		// 		if (g_wave_ready){
		// 			g_wave_ready = 0;
		// #if (USE_MOTOR_PID_CONTROL)==1
		// 			// 调试：打印 PID 输出、编码器速度、误差、实际频率
		// 			extern volatile float g_pid_debug_output;
		// 			extern volatile float g_pid_debug_actual;
		// 			extern volatile float g_pid_debug_error;
		// 			extern volatile uint16_t g_pid_debug_freq;
		// 			printf("%.1f,%.1f,%.1f,%.1f\r\n",
		// 			       g_wave_target,
		// 			       (double)g_pid_debug_actual,
		// 			       g_pid_debug_output,
		// 			       g_pid_debug_error);
		// #else
		//
		//
		// #endif
		// 		}
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
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


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1){
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
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
		motor->step_motor_information.current_frequency = motor_speed_to_freq(g_dev->sensor.speed,
			motor->step_motor_information.step_model);
#if USE_MOTOR_PID_CONTROL==1
		pid_control_tick(find_motor_by_tim(htim));
#endif
		// 波形输出：打印目标值和实际值
		extern volatile uint8_t g_wave_ready;
		extern volatile float g_wave_target;
		extern volatile float g_wave_actual;
		extern volatile uint8_t auto_tune_active;
		static volatile uint8_t tick = 0;
		if (++tick >= 5){
			tick = 0;
			// extern PID_AutoTune_t tuner;
			// g_wave_target = auto_tune_active ? tuner.setpoint : motor->motor_pid.Target;
			g_wave_actual = g_dev->sensor.speed;
			g_wave_ready = 1;
		}
	}
	/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	CRITICAL_ENTER();
	while (1){
		// 串口发送
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
