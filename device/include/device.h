#ifndef TWO_LINK_PUBLIC_RULE_H
#define TWO_LINK_PUBLIC_RULE_H
#include "stm32f1xx.h"
#include <stdio.h>
#define EPSION				0.001	// 浮点精度
#define FLOAT_COMPARATIVE_ACCURACY	4
#define PI				3.1415926f

// 程序配置相关
#define	DEBUG	1	// 是否进入DEBUG模式

// 设备处理类
typedef enum device_err{
	DRV_OK = 0,
	DRV_ERR_NULL,
	DRV_ERR_INIT,
	DRV_ERR_TIMEOUT,
	DRV_ERR_PARAM,
	DRV_ERR_BUSY,
	DRV_ERR_IO,
} device_err_t;

// 电机相关配置信息(比如是否使用rtos, 定时器等等)
#define USE_MOTOR_PID_CONTROL	0
// #define USE_FREERTOS			// 判断系统是否使用FreeRTOS


// 电机步进方式枚举（多处共用，放在公共头文件避免循环依赖）
typedef enum motor_step_model{
	DEFAULT_STEP = 1,
	FULL_STEP = 1,
	HALF_STEP = 2,
	ONE_FOURTH_STEP = 4,
	ONE_EIGHTH_STEP = 8,
	ONE_SIXTEENTH_STEP = 16
} motor_step_model_t;


// 根据是否加入OS决定进入临界区的API使用
#ifdef USE_FREERTOS
	#include "FreeRTOS.h"
	#include "task.h"
	#define CRITICAL_ENTER()    taskENTER_CRITICAL()
	#define CRITICAL_EXIT()     taskEXIT_CRITICAL()
	#define CRITICAL_ENTER_FROM_ISR()  taskENTER_CRITICAL_FROM_ISR()
	#define CRITICAL_EXIT_FROM_ISR(x)  taskEXIT_CRITICAL_FROM_ISR(x)
#else
	#define CRITICAL_ENTER()    __disable_irq()
	#define CRITICAL_EXIT()     __enable_irq()
	// ISR 版本裸机下退化为同上
	#define CRITICAL_ENTER_FROM_ISR()  __disable_irq()
	#define CRITICAL_EXIT_FROM_ISR(x)  __enable_irq()
#endif

// 常见数值运算方法
/**
 * @brief  快速浮点数求绝对值（位操作，不经过软浮点库）
 * @param  x: 输入浮点数
 * @retval |x|
 * @note   IEEE 754: 符号位在 bit31，清零即可。Cortex-M3 无 FPU 时比 fabsf 快 5~10 倍
 */
static inline float self_fabs(float x)
{
	union{
		float f;
		uint32_t i;
	} u;
	u.f = x;
	u.i &= 0x7FFFFFFFu;
	return u.f;
}

/**
 * @brief  弧度转角度
 * @param  rad: 弧度值
 * @retval 角度值（°）
 */
static inline float rad_to_deg(float rad)
{
	return rad * (180.0f / PI);
}

/**
 * @brief  角速度(rad/s)转转速(rpm)
 * @param  omega: 角速度（rad/s）
 * @retval 转速（rpm）
 */
static inline float omega_to_rpm(float omega)
{
	return omega * (60.0f / (2.0f * PI));
}

/**
 * @brief  转速(rpm)转角速度(rad/s)
 * @param  rpm: 转速（rpm）
 * @retval 角速度（rad/s）
 */
static inline float rpm_to_omega(float rpm)
{
	return rpm * (2.0f * PI / 60.0f);
}

#endif //TWO_LINK_PUBLIC_RULE_H
