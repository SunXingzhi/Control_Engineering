#include "PID.h"
#include <stddef.h>


PID_t*	pid	= NULL;
extern const PID_AlgoInterface_t PID_INCREMENTAL_OPS;
extern const PID_AlgoInterface_t PID_POSITIONAL_OPS;

// 算法注册表
static const PID_AlgoInterface_t* algo_table[] = {
	[PID_POSITIONAL]  = &PID_POSITIONAL_OPS,
	[PID_INCREMENTAL] = &PID_INCREMENTAL_OPS,
};

/**
 * @brief PID控制器初始化
 * @param pid
 * @param algo
 * @param args
 * @param kp
 * @param ki
 * @param kd
 * @param output_max
 * @param output_min
 * @return
 */
PID_t* PID_init(PID_t* pid, PID_algo_t algo, void* args,
                float kp, float ki, float kd,
                float output_max, float output_min)
{
	if (pid == NULL) return NULL;

	pid->pid_algo   = algo;
	pid->interface  = algo_table[algo];
	pid->algo_state = pid->interface->init();
	pid->args       = args;
	pid->Kp		= kp;
	pid->Ki		= ki;
	pid->Kd		= kd;
	pid->OutputMax = output_max;
	pid->OutputMin = output_min;

	// 初始化运行时状态（防止首次 calc 时使用未定义值）
	pid->Target   = 0;
	pid->Output   = 0;
	pid->Error    = 0;
	pid->LastError = 0;


	return pid;
}

float PID_calc(PID_t* pid, float actual)
{
	return pid->interface->calc(pid, actual);
}

void PID_reset(PID_t* pid)
{
	return pid->interface->reset(pid);
}

void PID_destroy(PID_t* pid)
{
	pid->interface->destroy(pid->algo_state);
}