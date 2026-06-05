#include "PID.h"
#include <stdlib.h>


// 增量式私有状态
typedef struct {
	float PrevError;
} PID_IncState_t;

static void* inc_init(void)
{
	return calloc(1, sizeof(PID_IncState_t));
}

/**
 *
 * @param pid 传入的PID数据
 * @param actual 当前PID控制实际值
 * @return PID控制器后的得出值
 */
static float inc_calc(PID_t* pid, float actual)
{
	PID_IncState_t* st = (PID_IncState_t*)pid->algo_state;

	pid->Error    = pid->Target - actual;
	pid->Output  += pid->Kp * (pid->Error - pid->LastError)
		      +  pid->Ki * pid->Error
		      +  pid->Kd * (pid->Error + st->PrevError - 2*pid->LastError);

	st->PrevError  = pid->LastError;
	pid->LastError = pid->Error;

	if (pid->Output > pid->OutputMax)  pid->Output = pid->OutputMax;
	if (pid->Output < -pid->OutputMax) pid->Output = -pid->OutputMax;

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

