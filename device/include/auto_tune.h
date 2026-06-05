/**
 * @file   auto_tune.h
 * @brief  PID继电器自动调参模块
 * @note   通过继电器反馈法使系统产生等幅振荡，测量振荡周期 Tu 和振幅 Au，
 *         再根据 Ziegler-Nichols 公式计算出 Kp, Ki, Kd.
 *
 *  使用流程:
 *    1. PID_AutoTune_Init()  — 初始化调参器
 *    2. PID_AutoTune_Calc()  — 每个控制周期调用，返回继电器输出
 *    3. PID_AutoTune_IsDone() — 检查是否完成
 *    4. PID_AutoTune_GetResult() — 获取调参结果
 */

#ifndef DEVICE_AUTO_TUNE_H
#define DEVICE_AUTO_TUNE_H

#include "device.h"

// ======================== 类型定义 ========================

/** 调参状态 */
typedef enum AUTOTUNE_STATE {
	AUTOTUNE_IDLE = 0,		// 空闲（未启动或已复位）
	AUTOTUNE_RELAY_HIGH,	// 继电器输出 +d
	AUTOTUNE_RELAY_LOW,		// 继电器输出 -d
	AUTOTUNE_DONE			// 调参完成
} autotune_state_t;

/** 调参结果 */
typedef struct AUTOTUNE_RESULT {
	float Ku;			// 继电器增益 = 4d / (π * Au)
	float Tu;			// 振荡周期 (s)
	float Kp;			// 比例系数
	float Ki;			// 积分系数
	float Kd;			// 微分系数
} autotune_result_t;

/** 继电器调参器实例 */
typedef struct PID_AUTOTUNE {
	// ---- 用户配置（Init 时设置） ----
	float	relay_amplitude;	// 继电器幅值 d（输出 ±d）
	float	hysteresis;		// 继电器滞环宽度（防抖，推荐 d 的 5%~10%）
	float	setpoint;		// 目标值（调参时的振荡中心）
	uint8_t	cycles_required;	// 需要测到的完整振荡周期数（推荐 5~10）

	// ---- 运行时状态 ----
	autotune_state_t state;		// 当前状态机状态
	float	output;			// 当前继电器输出（+d 或 -d）

	// ---- 振荡测量 ----
	uint8_t	half_cycle_count;	// 已记录的半周期计数
	float	peak_high;		// 当前半周期的峰值
	float	peak_low;		// 当前半周期的谷值
	float	last_cross_time;	// 上次过零时刻 (s)
	float	period_sum;		// 多个完整周期的时间累加 (s)
	float	amplitude_sum;		// 多个完整周期的振幅累加
	uint8_t	cycles_count;		// 已测到的完整周期数
	float	timestamp;		// 当前时间戳 (s)

	// ---- 结果 ----
	autotune_result_t result;	// 调参结果
} PID_AutoTune_t;


// ======================== 公共 API ========================

/**
 * @brief  初始化继电器调参器
 * @param  at:         调参器实例指针
 * @param  relay_d:    继电器幅值（越大振荡越明显，但系统冲击也越大）
 * @param  hysteresis: 滞环宽度（推荐 relay_d 的 5%~10%）
 * @param  setpoint:   调参时的目标值
 * @param  cycles:     需要测量的完整周期数（推荐 5~10）
 * @retval device_err_t
 */
device_err_t PID_AutoTune_Init(PID_AutoTune_t* at,
                                float relay_d, float hysteresis,
                                float setpoint, uint8_t cycles);

/**
 * @brief  继电器调参主循环（每个控制周期调用一次）
 * @param  at:     调参器实例指针
 * @param  actual: 当前实际值（传感器反馈）
 * @param  dt:     时间步长 (s)
 * @retval 继电器输出值（直接送给执行器）
 * @note   调参完成后继续调用会返回最后一次输出，不会重复计算
 */
float PID_AutoTune_Calc(PID_AutoTune_t* at, float actual, float dt);

/**
 * @brief  检查调参是否完成
 * @param  at: 调参器实例指针
 * @retval 1 = 完成，0 = 未完成
 */
uint8_t PID_AutoTune_IsDone(const PID_AutoTune_t* at);

/**
 * @brief  获取调参结果
 * @param  at: 调参器实例指针
 * @retval 调参结果结构体指针；未完成时返回 NULL
 */
const autotune_result_t* PID_AutoTune_GetResult(const PID_AutoTune_t* at);

/**
 * @brief  复位调参器（可重新开始新一轮调参）
 * @param  at: 调参器实例指针
 * @retval device_err_t
 */
device_err_t PID_AutoTune_Reset(PID_AutoTune_t* at);

/**
 * @brief  运行时更新目标值（调参过程中调整振荡中心）
 * @param  at:       调参器实例指针
 * @param  setpoint: 新的目标值
 * @retval device_err_t
 */
device_err_t PID_AutoTune_SetSetpoint(PID_AutoTune_t* at, float setpoint);

#endif //DEVICE_AUTO_TUNE_H
