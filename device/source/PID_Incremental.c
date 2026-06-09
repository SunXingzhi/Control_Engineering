/**
 * 增量式PID
 *
 * 公式:
 *   ΔOutput = Kp×(e[k]-e[k-1]) + Ki×e[k] + Kd×(e[k]-2×e[k-1]+e[k-2])
 *   Output  += ΔOutput
 *
 * 特点:
 *   - 增量式本身具有抗饱和能力：输出限幅后，下次计算基于实际输出（非饱和值）
 *   - 因此不需要像位置式那样额外做积分抗饱和
 */
#include "PID.h"
#include <stdlib.h>

// 增量式私有状态
typedef struct {
	float PrevError;   // e[k-2]
} PID_IncState_t;

static void* inc_init(void)
{
	return calloc(1, sizeof(PID_IncState_t));
}

/**
 * @brief  增量式 PID 计算
 * @param  pid:    PID 实例
 * @param  actual: 当前实际值（传感器反馈）
 * @retval PID 输出值
 */
static float inc_calc(PID_t* pid, float actual)
{
	PID_IncState_t* st = (PID_IncState_t*)pid->algo_state;

	pid->Error = pid->Target - actual;

	// 增量式公式: ΔOutput = Kp×(e[k]-e[k-1]) + Ki×e[k] + Kd×(e[k]-2e[k-1]+e[k-2])
	float delta = pid->Kp * (pid->Error - pid->LastError)
	            + pid->Ki * pid->Error
	            + pid->Kd * (pid->Error + st->PrevError - 2 * pid->LastError);

	pid->Output += delta;

	// 更新历史误差（必须在限幅之前，否则限幅后的 Output 会影响下次 delta 计算）
	st->PrevError  = pid->LastError;
	pid->LastError = pid->Error;

	// 输出限幅: 使用 OutputMax 和 OutputMin
	if (pid->Output > pid->OutputMax) pid->Output = pid->OutputMax;
	if (pid->Output < pid->OutputMin) pid->Output = pid->OutputMin;

	return pid->Output;
}

static void inc_reset(PID_t* pid)
{
	PID_IncState_t* st = (PID_IncState_t*)pid->algo_state;
	st->PrevError  = 0;
	pid->LastError = 0;
	pid->Error     = 0;
	pid->Output    = 0;
}


// 导出接口
const PID_AlgoInterface_t PID_INCREMENTAL_OPS = {
	.init    = inc_init,
	.calc    = inc_calc,
	.reset   = inc_reset,
	.destroy = free,
};

