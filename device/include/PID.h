/**
 * 速度PID
 *
 */
#ifndef __PID_H__
#define __PID_H__
#include "device.h"

typedef struct PID{
	float	Kp;            // 比例系数
	float	Ki;            // 积分系数
	float	Kd;            // 微分系数
	float	Target;        // 目标值
	float	Output;        // PID计算输出值
	float	PrevError;     // 上次误差，用于微分项
	float	Integral;      // 误差积分累积
	float	IntegralLimit; // 积分限幅
	// float OutputMax;     // 输出最大值
	// float OutputMin;     // 输出最小值
} PID_t;


device_err_t PID_init(PID_t *pid, float kp, float ki, float kd);
float PID_update(PID_t *pid, float feedback);
#endif