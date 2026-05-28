#ifndef __ANGLE_SENSOR_H
#define __ANGLE_SENSOR_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 角度传感器对象结构体
 */
typedef struct {
    ADC_HandleTypeDef* hadc;       /* 已初始化的ADC句柄指针        */
    uint32_t           Channel;    /* ADC通道号，如ADC_CHANNEL_0   */
    uint32_t           SampleTime; /* 采样时间，如ADC_SAMPLETIME_55CYCLES_5 */
    float              k;          /* 线性换算斜率 (angle = raw * k + b) */
    float              b;          /* 线性换算截距                 */

    /* 一阶低通滤波相关 */
    float              filter_coeff; /* 滤波系数 (0.0~1.0)         */
    float              filtered_value; /* 上一次滤波后的角度值    */
    uint8_t            filter_initialized; /* 初始化标志          */
} AngleSensor;

/**
 * @brief 初始化角度传感器对象（带默认滤波系数）
 * @param sensor     传感器对象指针
 * @param hadc       已初始化的ADC句柄指针（例如 &hadc1）
 * @param Channel    ADC通道宏，如 ADC_CHANNEL_0
 * @param SampleTime 采样时间，如 ADC_SAMPLETIME_55CYCLES_5
 * @param k          角度斜率 (单位/ADC读数)
 * @param b          角度截距
 * @param filter_coeff 一阶滤波系数 (0.0~1.0)，如 0.15；传 1.0 则等于无滤波
 */
void AngleSensor_Init(AngleSensor* sensor,
                      ADC_HandleTypeDef* hadc,
                      uint32_t Channel,
                      uint32_t SampleTime,
                      float k, float b,
                      float filter_coeff);

/**
 * @brief 读取ADC原始值（单次转换，阻塞等待）
 * @param  sensor 传感器对象指针
 * @return ADC转换结果（0~4095）
 */
uint32_t AngleSensor_ReadRaw(AngleSensor* sensor);

/**
 * @brief 获取未经滤波的瞬时角度值
 * @param  sensor 传感器对象指针
 * @return 瞬时角度值
 */
float AngleSensor_GetAngle(AngleSensor* sensor);

/**
 * @brief 获取经过一阶低通滤波后的角度值
 * @param  sensor 传感器对象指针
 * @return 滤波后的角度值
 * @note   第一次调用时直接用瞬时值初始化，之后按照指数移动平均更新
 */
float AngleSensor_GetFilteredAngle(AngleSensor* sensor);

/**
 * @brief 重新设置校准系数
 * @param sensor 传感器对象指针
 * @param k     新斜率
 * @param b     新截距
 */
void AngleSensor_SetCalibration(AngleSensor* sensor, float k, float b);

/**
 * @brief 重新设置滤波系数
 * @param sensor 传感器对象指针
 * @param coeff 新的滤波系数 (0.0~1.0)
 */
void AngleSensor_SetFilterCoeff(AngleSensor* sensor, float coeff);

#ifdef __cplusplus
}
#endif

#endif

