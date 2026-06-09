/**
 * @file    mt6701.h
 * @brief   MT6701 磁编码器驱动（SPI + DMA 方式）
 */

#ifndef __MT6701_H
#define __MT6701_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "device.h"

/* ---- 速度计算频率（供 MX_TIM3_Init 使用）---- */
#define speed_calc_freq     1000   /* Hz */

#define MT6701_ANGLE_MAX    (2.0f * 3.1415926f)
#define MT6701_RAW_MAX      ((1 << 14) - 1)   /* 16383 */

/* ================================================================
 *  角度传感器基类（硬件抽象）
 * ================================================================ */
typedef struct{
	SPI_HandleTypeDef* hspi; /* SPI 句柄指针                   */
	GPIO_TypeDef* cs_gpiox; /* CS 引脚 GPIO 端口              */
	uint16_t cs_gpio_pin; /* CS 引脚号                      */
	TIM_HandleTypeDef* htim; /* 速度计算定时器句柄             */

	uint32_t angle_raw; /* 原始角度值（14-bit, 0~16383）  */
	int32_t speed_raw; /* 角速度（度/秒 × 100）          */

	float angle; /* 当前角度，弧度 (0 ~ 2pi)       */
	float speed; /* 当前角速度，弧度/秒            */
	float angle_last; /* 上一次角度（速度计算用）       */
	float angle_prev; /* 上一次角度（累积角度过零检测用） */
	float angle_total; /* 累计角度，正转累加，反转递减     */
	uint8_t first_sample; /* 首次采样标志                   */
} angle_sensor_t;

/* ================================================================
 *  MT6701 设备实例（继承 angle_sensor_t）
 * ================================================================ */
typedef struct{
	angle_sensor_t sensor; /* 传感器基类（必须为第一个成员） */

	uint8_t tx_buf[3]; /* SPI DMA 发送缓冲区（dummy）   */
	uint8_t rx_buf[3]; /* SPI DMA 接收缓冲区            */
} mt6701_t;

/* ================================================================
 *  辅助函数(外部无需调用)
 * ================================================================ */
float cycle_diff(float diff, float cycle);

/* ================================================================
 *  API 函数
 * ================================================================ */

/**
 * @brief  初始化角度传感器：启动 SPI DMA 接收 + 定时器中断
 * @param  dev  MT6701 设备指针，调用前需填好 sensor 中的 hspi / cs_gpiox / cs_gpio_pin / htim
 * @retval DRV_OK / DRV_ERR_NULL / DRV_ERR_INIT
 */
device_err_t angle_sensor_init(mt6701_t* dev);

/**
 * @brief  读取当前角度
 * @param  dev      MT6701 设备指针
 * @param  angle    [out] 角度值（弧度制，0~2π）
 * @retval DRV_OK / DRV_ERR_NULL
 */
device_err_t angle_sensor_read_angle(mt6701_t* dev, float* angle);

/**
 * @brief  读取当前角速度
 * @param  dev      MT6701 设备指针
 * @param  speed    [out] 角速度（度/秒 × 100），正值正转，负值反转
 * @retval DRV_OK / DRV_ERR_NULL
 */
device_err_t angle_sensor_read_speed(mt6701_t* dev, float* speed);

/**
 * @brief  读取累计角度
 * @param  dev      MT6701 设备指针
 * @param  angle    [out] 累计角度（弧度制，0~2π），正转累加，反转递减
 * @retval DRV_OK / DRV_ERR_NULL
 */
device_err_t angle_sensor_read_total_angle(mt6701_t* dev, float* angle);

/**
 * @brief  编码器速度更新（TIM3 中断调用，1ms 周期）
 * @note   带一阶低通滤波，结果存入 g_dev->sensor.speed（rpm）
 */
void encoder_update_speed(void);

#endif /* __MT6701_H */
