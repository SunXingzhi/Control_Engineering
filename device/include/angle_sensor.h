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
    float              offset;     /* 角度偏移量：最终角度 = 原始角度 - offset */

    /* 一阶低通滤波相关 */
    float              filter_coeff;      /* 角度滤波系数 (0.0~1.0)      */
    float              filtered_value;    /* 上一次滤波后的角度值        */
    uint8_t            filter_initialized;/* 角度滤波器初始化标志        */

    /* 角速度计算相关 */
    float              prev_angle;        /* 上一次用于计算速度的角度值 */
    uint32_t           prev_time_ms;      /* 上一次角度采样的时间戳(ms) */
    uint8_t            vel_initialized;   /* 速度微分初始化标志          */

    /* 角速度一阶低通滤波 */
    float              vel_filter_coeff;       /* 速度滤波系数 (0.0~1.0) */
    float              filtered_velocity;      /* 滤波后的角速度 */
    uint8_t            vel_filter_initialized; /* 速度滤波器初始化标志 */

    /* 角度量程范围，用于处理环绕微分 */
    float              range;              /* 传感器量程，默认 360.0° */
} AngleSensor;

/**
 * @brief 初始化角度传感器对象（带默认滤波系数）
 * @param sensor        传感器对象指针
 * @param hadc          已初始化的ADC句柄指针（例如 &hadc1）
 * @param Channel       ADC通道宏，如 ADC_CHANNEL_0
 * @param SampleTime    采样时间，如 ADC_SAMPLETIME_55CYCLES_5
 * @param k             角度斜率 (单位/ADC读数)
 * @param b             角度截距
 * @param filter_coeff  一阶滤波系数 (0.0~1.0)，如 0.15；传 1.0 则等于无滤波
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

float angle_sensor_get_unwrapped(AngleSensor* sensor);

/**
 * @brief 获取未经滤波的瞬时角度值（已扣除偏移量）
 * @param  sensor 传感器对象指针
 * @return 瞬时角度值
 */
float AngleSensor_GetAngle(AngleSensor* sensor);


/**
 * @brief 获取经过一阶低通滤波后的角度值（已扣除偏移量）
 * @param  sensor 传感器对象指针
 * @return 滤波后的角度值
 * @note   第一次调用时直接用瞬时值初始化，之后按照指数移动平均更新
 */
float AngleSensor_GetFilteredAngle(AngleSensor* sensor);


/**
 * @brief 获取未归一化的滤波角度（连续值，无 0°/360° 跳变）
 * @note  用于控制算法，避免角度归一化导致 PID 输入突变
 */
float AngleSensor_GetFilteredAngleUnwrapped(AngleSensor* sensor);


/**
 * @brief 获取角速度（基于滤波后角度的微分）
 * @param  sensor 传感器对象指针
 * @return 角速度，单位：度/秒
 * @note   首次调用返回 0；之后根据两次采样的角度差与时间差计算。
 *         该函数内部会调用 GetFilteredAngle 以获得最新角度值。
 *         在 SetCurrentAngle 或 SetOffset 后，速度计算状态会被重置，下一次调用返回 0。
 */
float AngleSensor_GetAngularVelocity(AngleSensor* sensor);


/**
 * @brief 重新设置线性校准系数
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


/**
 * @brief 将当前物理角度设置为指定角度值（自动计算偏移量）
 * @param sensor       传感器对象指针
 * @param target_angle 希望当前位置显示的角度值（例如摆下垂时设为 0°）
 * @note  调用后立即生效，GetAngle / GetFilteredAngle 都会减去新计算的偏移量。
 *        内部会重置滤波器状态和速度计算状态，使输出立即跳变并避免虚假速度尖峰。
 */
void AngleSensor_SetCurrentAngle(AngleSensor* sensor, float target_angle);


/**
 * @brief 直接设置角度偏移量
 * @param sensor 传感器对象指针
 * @param offset 偏移量，最终角度 = 原始角度 - offset
 * @note  调用后即时更新滤波值，并重置速度计算状态。
 */
void AngleSensor_SetOffset(AngleSensor* sensor, float offset);


void AngleSensor_SetRange(AngleSensor* sensor, float range);


#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_SENSOR_H */
