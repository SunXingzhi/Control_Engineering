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
 * @brief  轻量字符串转浮点（替代 strtof，只处理十进制，不链接 strtod 库）
 * @param  s  输入字符串，格式: [+-]digits[.digits]
 * @retval float 值
 */
static inline float strtof_lite(const char* s)
{
	float sign = 1.0f;
	float val = 0.0f;
	float frac = 0.0f;
	float div = 1.0f;

	if (*s == '-')      { sign = -1.0f; s++; }
	else if (*s == '+') { s++; }

	while (*s >= '0' && *s <= '9') {
		val = val * 10.0f + (float)(*s - '0');
		s++;
	}
	if (*s == '.') {
		s++;
		while (*s >= '0' && *s <= '9') {
			frac = frac * 10.0f + (float)(*s - '0');
			div *= 10.0f;
			s++;
		}
	}
	return sign * (val + frac / div);
}

/**
 * @brief  轻量浮点转字符串（纯整数运算，不链接 printf %f）
 * @param  buf    输出缓冲区（至少 16 字节）
 * @param  val    浮点值
 * @param  dec    小数位数（1~4）
 * @retval buf 指针（可直接传给 printf %s）
 */
static inline char* ftoa_lite(char* buf, float val, int dec)
{
	if (dec < 1) dec = 1;
	if (dec > 4) dec = 4;

	/* 处理负数 */
	char* p = buf;
	if (val < 0.0f) { *p++ = '-'; val = -val; }

	/* 四舍五入 */
	float round = 0.5f;
	for (int i = 0; i < dec; i++) round *= 0.1f;
	val += round;

	/* 整数部分 */
	int int_part = (int)val;
	float frac_part = val - (float)int_part;

	/* 整数转字符串（递归写法，避免数组） */
	char int_buf[12];
	int int_len = 0;
	if (int_part == 0) {
		int_buf[int_len++] = '0';
	} else {
		while (int_part > 0) {
			int_buf[int_len++] = '0' + (int_part % 10);
			int_part /= 10;
		}
	}
	for (int i = int_len - 1; i >= 0; i--)
		*p++ = int_buf[i];

	/* 小数部分 */
	*p++ = '.';
	for (int i = 0; i < dec; i++) {
		frac_part *= 10.0f;
		int digit = (int)frac_part;
		*p++ = '0' + digit;
		frac_part -= (float)digit;
	}
	*p = '\0';
	return buf;
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


/**
 *
 * @param left_limited_rad 滑台左侧限位弧度值(total_angle)
 * @param right_limted_rad 滑台右侧限位弧度值
 * @return
 */
float get_positional_scale(float left_limited_rad, float right_limted_rad, float slide_table_safety_stroke);

/**
 *
 * @param current_absolute_angle_rad 当前绝对角度弧度值(total_angle)
 * @param midnight_angle_rad 相对坐标零点坐标值
 * @return 基于该坐标系下的位置坐标
 */
float get_linear_position(float current_absolute_angle_rad, float midnight_angle_rad, float position_scale);

#endif //TWO_LINK_PUBLIC_RULE_H
