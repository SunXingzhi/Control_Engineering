/**
 * 位置式PID（带时间自适应的梯形积分 + 积分抗饱和）
 *
 * 公式:
 *   err = Target - actual
 *   SumError += (err + prev_error) * deltaT / 2   // 梯形积分
 *   DError = (err - prev_error) / deltaT           // 差分近似微分
 *   Output = Kp*err + Ki*SumError + Kd*DError
 *
 * 特点:
 *   - 使用 DWT 微秒计时，自适应调用频率
 *   - 积分抗饱和: 输出饱和时冻结积分累加
 *   - 首次调用跳过微分和积分（prev_error=0, deltaT 不可信）
 */
#include <stdint.h>

#include "PID.h"
#include <stdlib.h>

/* DWT 微秒时间戳，定义在 control.c 中 */
extern uint32_t DWT_GetTick_us(void);

typedef struct{
	float SumError; // 积分累加值
	float DError; // 上一次的微分值（缓存供调试用）
	float prev_error; // 上一次误差
	uint32_t t_k_1; // 上次计算时间 (us)
	uint8_t initialized; // 首次调用标志
} PID_PosState_t;

static void* pos_init(void)
{
	return calloc(1, sizeof(PID_PosState_t));
}

/**
 * @brief  位置式 PID 计算（带梯形积分 + 积分抗饱和）
 * @param  pid:    PID 实例（Kp/Ki/Kd/Target/OutputMax/OutputMin 已设置）
 * @param  actual: 当前实际值（传感器反馈）
 * @retval PID 输出值
 */
static float pos_calc(PID_t* pid, float actual)
{
	PID_PosState_t* st = (PID_PosState_t*)pid->algo_state;

	uint32_t now_us = DWT_GetTick_us();
	float err = pid->Target - actual;

	/* 首次调用: 只记录时间和误差，不输出 */
	if (!st->initialized){
		st->t_k_1 = now_us;
		st->prev_error = err;
		st->initialized = 1;
		pid->Error = err;
		pid->LastError = 0;
		return 0.0f;
	}

	/* 计算时间间隔 (秒) */
	float deltaT = (float)(now_us - st->t_k_1) * 1e-6f;
	if (deltaT <= 0.0f) deltaT = 1e-6f; // 防护: 最小 1us

	/* 梯形积分 */
	st->SumError += (err + st->prev_error) * deltaT * 0.5f;

	/* 积分限幅 (抗饱和) —— 量纲为 误差×时间，独立于输出限幅 OutputMax/Min
	 * BUG 修复: 原代码误用 OutputMax/OutputMin（其量纲是输出量，如 RPM 或 rad）
	 *          去限幅 SumError（量纲是误差×时间），导致积分几乎永不饱和、抗饱和失效。
	 * 若用户未显式设置 IntegralMax/Min（<=0），回退到一个保守的上限：
	 *   使得积分项 Ki·SumError 不超过输出量程的 90%，即 |SumError| <= 0.9·|Output|/|Ki|。
	 *   这样在 Ki 极小或为 0 时不会误伤（上限趋于无穷），Ki 较大时也能有效防饱和。 */
	float imax = pid->IntegralMax;
	float imin = pid->IntegralMin;
	if (imax <= 0.0f || imin >= 0.0f) {
		float ki = pid->Ki;
		float ki_abs = (ki >= 0.0f) ? ki : -ki;
		float out_abs = (pid->OutputMax >= 0.0f) ? pid->OutputMax : -pid->OutputMax;
		if (out_abs < 1e-9f) out_abs = 420.0f;  /* 兜底，避免除零 */
		float bound = (ki_abs > 1e-9f) ? (0.9f * out_abs / ki_abs) : 1e6f;
		if (imax <= 0.0f)  imax = bound;
		if (imin >= 0.0f)  imin = -bound;
	}
	if (st->SumError > imax) st->SumError = imax;
	if (st->SumError < imin) st->SumError = imin;

	/* 微分 (差分近似) */
	st->DError = (err - st->prev_error) / deltaT;

	/* PID 输出 */
	float output = pid->Kp * err
		+ pid->Ki * st->SumError
		+ pid->Kd * st->DError;

	/* 输出限幅 + 饱和时冻结积分 */
	if (output > pid->OutputMax){
		output = pid->OutputMax;
		/* 如果误差方向与饱和方向一致，撤销本次积分累加 */
		if (err > 0)
			st->SumError -= (err + st->prev_error) * deltaT * 0.5f;
	}
	else if (output < pid->OutputMin){
		output = pid->OutputMin;
		if (err < 0)
			st->SumError -= (err + st->prev_error) * deltaT * 0.5f;
	}

	/* 更新历史 */
	st->t_k_1 = now_us;
	st->prev_error = err;
	pid->Error = err;
	pid->LastError = err;
	pid->Output = output;

	return output;
}

static void pos_reset(PID_t* pid)
{
	PID_PosState_t* st = (PID_PosState_t*)pid->algo_state;
	st->SumError = 0;
	st->DError = 0;
	st->prev_error = 0;
	st->t_k_1 = 0;
	st->initialized = 0;
	pid->Error = 0;
	pid->LastError = 0;
	pid->Output = 0;
}

/* 导出接口 */
const PID_AlgoInterface_t PID_POSITIONAL_OPS = {
	.init = pos_init,
	.calc = pos_calc,
	.reset = pos_reset,
	.destroy = free,
};
