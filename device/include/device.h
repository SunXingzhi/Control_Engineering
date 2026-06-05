#ifndef TWO_LINK_PUBLIC_RULE_H
#define TWO_LINK_PUBLIC_RULE_H
#include "stm32f1xx.h"
#include <stdio.h>
#define EPSION				0.001	// 浮点精度
#define FLOAT_COMPARATIVE_ACCURACY	4

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
#define USE_MOTOR_PID_CONTROL	1
// #define USE_FREERTOS			// 判断系统是否使用FreeRTOS


// 电机步进方式枚举（多处共用，放在公共头文件避免循环依赖）
typedef enum motor_step_model{
	DEFAULT_STEP		= 1,
	FULL_STEP		= 1,
	HALF_STEP		= 2,
	ONE_FOURTH_STEP		= 4,
	ONE_EIGHTH_STEP		= 8,
	ONE_SIXTEENTH_STEP	= 16
} motor_step_model_t;

// 常见数值运算方法
/**
 * @brief  快速浮点数求绝对值（位操作，不经过软浮点库）
 * @param  x: 输入浮点数
 * @retval |x|
 * @note   IEEE 754: 符号位在 bit31，清零即可。Cortex-M3 无 FPU 时比 fabsf 快 5~10 倍
 */
static inline float self_fabs(float x)
{
	union { float f; uint32_t i; } u;
	u.f = x;
	u.i &= 0x7FFFFFFFu;
	return u.f;
}

//
#endif //TWO_LINK_PUBLIC_RULE_H
