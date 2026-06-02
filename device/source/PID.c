#include "PID.h"
#include "driver_step_motor.h"

/**
 * @brief  从PID参数联合体中获取步进电机细分模式
 * @param  pid: PID结构体指针，其 pid_type 必须为 PID_MOTOR_SPEED 或 PID_MOTOR_POSITION
 * @retval motor_step_model_t 细分模式枚举值；若 pid_type 不匹配，返回 FULL_STEP 作为默认值
 */

static inline motor_step_model_t PID_get_step_model(const PID_t* pid)
{
	if (pid->pid_type == PID_MOTOR_SPEED ||
	    pid->pid_type == PID_MOTOR_POSITION) {
		return pid->pid_args.motor_step_model;
	    }
	return FULL_STEP; // 默认值
}

/**
 * @brief PID控制器初始化.
 * @param pid PID参数结构
 * @param pid_type 具体PID用途, 控制电机or角度传感器
 * @param pid_args 根据PID用途获取实际的配置参数, 如果用途是控制电机, 则表示细分模式
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_max 最大输出限幅
 * @param output_min 最小输出限幅
 * @return 构造完成的PID数据
 * @note 当用途是控制电机时, target以及actual表示的都是电机转速(单位RPM), output代表更新的频率
 */
PID_t*  PID_init(PID_t *pid,
                 PID_type_t pid_type,
                 PID_type_args_t pid_args,
                 float kp,
                 float ki,
                 float kd,
                 float output_max,
                 float output_min)
{
	if (pid==NULL)	return NULL;

	pid->pid_type		= pid_type;
	pid->pid_args		= pid_args;	// 如果是电机就是步进模式.
	pid->Kp			= kp;
	pid->Ki			= ki;
	pid->Kd			= kd;
	pid->IntegralMax	= 0;	// TODO 积分项的最大值如何确定
	pid->Output		= 0;
	pid->PrevError		= 0;
	pid->LastError		= 0;
	pid->SumError		= 0;
	pid->DError		= 0;
	pid->Error		= 0;
	pid->Target		= 0;
	pid->OutputMax		= output_max;
	pid->OutputMin		= output_min;

	return pid;
}

/**
 * @brief
 *
 */
PID_t* PID_incremental_calc(PID_t* pid, float actual_val)
{
	if (pid==NULL)	return NULL;

	pid->Error = pid->Target - actual_val;

	pid->Output  +=  pid->Kp* ( pid->Error - pid->LastError )+
					 pid->Ki* pid->Error +
					 pid->Kd* ( pid->Error +  pid->PrevError - 2*pid->LastError);

	pid->PrevError = pid->LastError;
	pid->LastError = pid->Error;

	// 设置输出限度
	if(pid->Output > pid->OutputMax )    pid->Output = pid->OutputMax;
	if(pid->Output < - pid->OutputMax )  pid->Output = -pid->OutputMax;



	// 计算实际要的频率及最后的ARR值(暂时用不到)
	// const uint16_t freq = motor_speed_to_freq(pid->Output, PID_get_step_model(pid));
	// uint16_t auto_reload	= motor_freq_to_arr(freq);

	return pid;
}



/**
 * @brief PID控制器更新函数, 根据句柄中的pid参数(kp, ki, kd)决定实际输出值
 * 特别的, 如果是电机PID, 则输入输出的是速度, 单位m/s.
 * @param pid 传入的PID参数信息
 * @param feedback 输入的当前系统值(与pid数据中的target进行比较获取心值)
 * @return
 */
PID_t* PID_update(PID_t *pid, float feedback)
{
	if (pid==NULL) return NULL;
	float result	= 0;



	return pid;
}