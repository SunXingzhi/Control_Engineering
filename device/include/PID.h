/**
 * 速度PID
 *
 */
#ifndef __PID_H__
#define __PID_H__
#include <stdint.h>
// #include "driver_step_motor.h"

typedef enum PID_TYPE{
	PID_MOTOR_POSITION,	// 步进电机位置环
	PID_MOTOR_SPEED		// 步进电机速度环
} PID_type_t;

/**
 * PID具体参数联合体, 在不同场景下PID的配置参数可能不同,
 * 例如步进电机参数需要了解步进模式是X细分. 角度传感器了解XX...
 */
typedef union PID_TYPE_ARGS{
	int32_t	motor_step_model;
} PID_type_args_t;

typedef struct PID{
	PID_type_t	pid_type;	// PID类型. 电机/角度传感器等
	PID_type_args_t	pid_args;	// 如果是步进电机就是细分模式
	float	Kp;		// 比例系数
	float	Ki;		// 积分系数
	float	Kd;		// 微分系数

	float	Target;		// 目标值
	float	Output;		// PID计算输出值

	float 	PrevError ;	//  Error[-2]
	float 	LastError;	//  Error[-1]
	float 	Error;    	//  Error[0 ]
	float 	DError;  	//pid->Error - pid->LastError
	float 	SumError; 	//  Sums of Errors

	float	IntegralMax;	// 积分项最大值
	float	OutputMax;	// 输出最大值
	float	OutputMin;	// 输出最小值
} PID_t;

PID_t* PID_init(PID_t *pid,
				PID_type_t pid_type,
				PID_type_args_t pid_args,
				float kp,
				float ki,
				float kd,
				float output_max,
				float output_min);
PID_t* PID_incremental_calc(PID_t* pid, float actual_val);
PID_t* PID_update(PID_t *pid, float feedback);

#endif