//
// Created by q1325 on 2026/5/16.
//

#include "../include/driver_step_motor.h"

#include "auto_tune.h"
#include "main.h"
#include "device.h"
#include "stm32f1xx_it.h"

#if (USE_MOTOR_PID_CONTROL==1)
#include "PID.h"
#include "auto_tune.h"
#endif


// 灵活配置tim_clock_freq, 方便后续更改
uint32_t tim_clock_freq	= 0;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#if (USE_MOTOR_PID_CONTROL==1)
	extern volatile uint8_t auto_tune_active;
	extern volatile PID_AutoTune_t tuner;
	extern volatile float g_wave_target;
	extern volatile float g_wave_actual;
	extern volatile uint8_t g_wave_ready;
	extern mt6701_t* g_dev;
#endif

// 内部函数定义
#if USE_MOTOR_PID_CONTROL==0
	static device_err_t ramp_step_motor_init(motor_ramp_t* ramp,
									uint32_t target_freq,
									uint32_t step_freq,
									uint32_t hold_ms);
	static device_err_t ramp_step_motor_start(motor_ramp_t* ramp, step_motor_t* motor);
	device_err_t ramp_step_motor_tick(motor_ramp_t* ramp, step_motor_t* motor);
#endif

void step_motor_pwm_off(step_motor_t* motor);
device_err_t step_motor_set_direction(step_motor_t* motor, motor_direction_t dir);

#if USE_MOTOR_PID_CONTROL==1
// PID 限速器状态（文件级，方便 step_motor_stop/set_speed 重置）
static float clamp_last_output = 0.0f;
#endif

// ===============================工具函数==============================
/**
 * @brief  电机转速转换成对应频率
 * @param  motor_speed_rpm: 转速值（单位: rpm）
 * @param  step_model: 步进细分模式，决定每转对应的脉冲数
 * @retval 对应的电机频率值
 */
uint16_t motor_speed_to_freq(float motor_speed_rpm, motor_step_model_t step_model)
{
	uint16_t multiple	= 1;
	uint16_t frequency	= 0;
	switch (step_model){
	case FULL_STEP:
		break;
	case HALF_STEP:
		multiple	= 2;
		break;
	case ONE_FOURTH_STEP:
		multiple	= 4;
		break;
	case ONE_EIGHTH_STEP:
		multiple	= 8;
		break;
	case ONE_SIXTEENTH_STEP:
		multiple	= 16;
		break;
	default:
		break;
	}

	frequency	= (uint16_t)(motor_speed_rpm * multiple * 10/3);
	// 如果超过最大电机频率限制, 则限制为最大频率
	if (frequency>MAX_PWM_FREQUENCY_HZ) return MAX_PWM_FREQUENCY_HZ;

	return frequency;
}

/**
 * @brief  电机频率转换实际的TIM AutoReload 寄存器值
 * @param  pulse_freq_hz: 此时的脉冲频率
 * @retval 返回对应ARR值
 */
uint16_t motor_freq_to_arr(uint16_t pulse_freq_hz)
{
	if (pulse_freq_hz==0 || tim_clock_freq==0) return 1;
	return (tim_clock_freq / (htim4.Init.Prescaler + 1) /
		pulse_freq_hz) - 1;
}

/**
 * @brief  电机频率转换为转速
 * @param  freq:       要转换的频率 (单位: Hz)
 * @param  step_model: 步进细分模式
 * @retval 转速值 (单位: rpm)
 */
uint16_t motor_freq_to_speed(const uint16_t freq, const motor_step_model_t step_model)
{
	uint16_t multiple	= 0;

	switch (step_model){
	case FULL_STEP:
		multiple	= 1;
		break;
	case HALF_STEP:
		multiple	= 2;
		break;
	case ONE_FOURTH_STEP:
		multiple	= 4;
		break;
	case ONE_EIGHTH_STEP:
		multiple	= 8;
		break;
	case ONE_SIXTEENTH_STEP:
		multiple	= 16;
		break;
	default:
		multiple	= 1;
		break;
	}

	return  (uint16_t)(freq*3/multiple/10);

}
// =================================================================

/**
 * @brief  初始化步进电机(用到什么内容可以改进电机对象成员变量)
 * @param  motor: 步进电机结构体指针，必须先填充 motor_base_info
 * @retval device_err_t
 */
device_err_t step_motor_init(step_motor_t* motor)
{
	// 参数检查
	if (motor == NULL) return DRV_ERR_NULL;
	if (motor->motor_base_info.tim_handle == NULL) return DRV_ERR_PARAM;


	// 初始化电机信息
	motor->step_motor_information.dir = POSITIVE_DIR;
	motor->step_motor_information.current_frequency = 0;
	// motor->step_motor_information.target_frequency = 0;

	// 设置步进模式
	step_motor_set_step_model(motor);

	// 如果使用闭环控制则无需设置斜坡加速, 而是采用PID+输出限幅的操作
#if (USE_MOTOR_PID_CONTROL==1)
	motor->motor_pid	= *PID_init(&motor->motor_pid,
				PID_INCREMENTAL,
				&(motor->step_motor_information.step_model),
				0.05f,		// Kp — 快速响应
				0.02f,		// Ki — 消除稳态误差
				0.002f,		// Kd — 抑制超调
				MOTOR_PID_OUTPUT_MAX(motor->step_motor_information.step_model),
				MOTOR_PID_OUTPUT_MIN(motor->step_motor_information.step_model)
			);

#elif (USE_MOTOR_PID_CONTROL==0)
	// 初始化斜坡参数
	ramp_step_motor_init(&motor->ramp, 0, MOTOR_STEP_LENGTH_FREQUENCY_HZ, 0);

#endif

	// 注册实例, 不注册会导致TIM采样失效.
	tim_register_motor(&htim4, motor);
	tim_register_motor(&htim3, motor);
	return DRV_OK;
}

/**
 * @brief  反初始化步进电机
 * @param  motor: 步进电机结构体指针
 * @retval device_err_t
 */
device_err_t step_motor_deinit(step_motor_t* motor)
{
	if (motor == NULL) return DRV_ERR_NULL;

	step_motor_stop(motor);

	/* 复位电机信息 */
	motor->step_motor_information.dir = STOP_DIR;
	motor->step_motor_information.dir_state	= DIR_NORMAL;
	motor->step_motor_information.current_frequency = 0;
	// motor->step_motor_information.target_frequency = 0;

	return DRV_OK;
}

/**
 * @brief 通过GPIO口操作更改步进电机步进模式(full/half/quarter, etc)
 * @param motor: 电机信息结构体
 * @return device_err_t: 操作结果
 */
device_err_t step_motor_set_step_model(step_motor_t* motor)
{
	if (motor==NULL) return DRV_ERR_NULL;

	switch (motor->step_motor_information.step_model){
	case FULL_STEP:
		HAL_GPIO_WritePin(MOTOR_MS1_PIN_GPIO_Port, MOTOR_MS1_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_MS2_PIN_GPIO_Port, MOTOR_MS2_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_MS3_PIN_GPIO_Port, MOTOR_MS3_PIN_Pin, GPIO_PIN_RESET);

		break;
	case HALF_STEP:
		HAL_GPIO_WritePin(MOTOR_MS1_PIN_GPIO_Port, MOTOR_MS1_PIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_MS2_PIN_GPIO_Port, MOTOR_MS2_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_MS3_PIN_GPIO_Port, MOTOR_MS3_PIN_Pin, GPIO_PIN_RESET);

		break;
	case ONE_FOURTH_STEP:
		HAL_GPIO_WritePin(MOTOR_MS1_PIN_GPIO_Port, MOTOR_MS1_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_MS2_PIN_GPIO_Port, MOTOR_MS2_PIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_MS3_PIN_GPIO_Port, MOTOR_MS3_PIN_Pin, GPIO_PIN_RESET);

		break;
	case ONE_EIGHTH_STEP:
		HAL_GPIO_WritePin(MOTOR_MS1_PIN_GPIO_Port, MOTOR_MS1_PIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_MS2_PIN_GPIO_Port, MOTOR_MS2_PIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_MS3_PIN_GPIO_Port, MOTOR_MS3_PIN_Pin, GPIO_PIN_RESET);

		break;
	case ONE_SIXTEENTH_STEP:
		HAL_GPIO_WritePin(MOTOR_MS1_PIN_GPIO_Port, MOTOR_MS1_PIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_MS2_PIN_GPIO_Port, MOTOR_MS2_PIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_MS3_PIN_GPIO_Port, MOTOR_MS3_PIN_Pin, GPIO_PIN_SET);

		break;
	default:
		HAL_GPIO_WritePin(MOTOR_MS1_PIN_GPIO_Port, MOTOR_MS1_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_MS2_PIN_GPIO_Port, MOTOR_MS2_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_MS3_PIN_GPIO_Port, MOTOR_MS3_PIN_Pin, GPIO_PIN_RESET);

		break;
	}

	return DRV_OK;
}

/**
 * @brief  启动步进电机PWM输出
 * @param  motor: 步进电机结构体指针
 * @retval device_err_t
 */
device_err_t step_motor_start(step_motor_t* motor)
{
	if (motor == NULL){
		return DRV_ERR_NULL;
	}

	/* 开启 PWM 输出（仅启动计数器和输出通道，不自动开中断）
	 * 中断由上层（step_motor_move_steps / ramp 状态机）显式控制
	 */
	if (HAL_TIM_PWM_Start(motor->motor_base_info.tim_handle,
		motor->motor_base_info.tim_channel) == HAL_OK){
		return DRV_OK;
	} else{
		return DRV_ERR_IO;
	}

}

/**
 * @brief  立即关闭电机（不更改实际状态，再次调用 step_motor_start 会恢复之前的速度）
 * @param  motor: 步进电机结构体指针
 * @retval device_err_t
 * @note   调用后立即返回，不阻塞。减速完成后 ramp 状态机会自动切断 PWM
 */
device_err_t step_motor_stop(step_motor_t* motor)
{

	if (motor==NULL) return DRV_ERR_NULL;
	step_motor_pwm_off(motor);

	motor->step_motor_information.current_frequency	= 0;
	motor->step_motor_information.dir		= STOP_DIR;
	motor->step_motor_information.dir_state		= DIR_NORMAL;
	motor->step_motor_information.step_remaining	= 0;

#if USE_MOTOR_PID_CONTROL==0
	// 清除斜坡状态机，防止 SysTick 回调继续用旧状态操作定时器
	motor->ramp.state        = RAMP_IDLE;
	motor->ramp.freq_current = 0;
	motor->ramp.freq_target  = 0;
	// motor->ramp.step_number  = 0;
#elif USE_MOTOR_PID_CONTROL==1
	// 重置限速器状态
	clamp_last_output = 0.0f;
#endif

	return DRV_OK;
}


/**
 * @brief  设置步进电机速度（非阻塞, 可被打断）
 * @param  motor: 步进电机结构体指针
 * @param  speed: 电机转速（单位: rpm，支持浮点数，但最后转化频率时小数会舍去）
 * @param  dir:   电机运动方向
 * @retval device_err_t
 * @note   如果传入速度与步进模式计算出的频率大于步进电机支持的最大频率，
 *         则强制转换为最大频率（MAX_PWM_FREQUENCY_HZ）
 */
device_err_t step_motor_set_speed(step_motor_t* motor, const float speed, motor_direction_t dir)
{
	float converted_speed	= 0;
	if (motor == NULL) return DRV_ERR_NULL;
	if (dir>STOP_DIR) return DRV_ERR_PARAM;

	// 如果需要换向，先停止电机（防止电流冲击），方向由 set_direction 在停车后设置
	if (motor->step_motor_information.dir != dir) step_motor_stop(motor);

	// 如果目标速度为0, 停止电机, 并且不设计斜坡/PID控制
	if (speed==0){
		step_motor_stop(motor);
		motor->step_motor_information.dir		= STOP_DIR;
		motor->step_motor_information.current_frequency	= 0;
#if USE_MOTOR_PID_CONTROL==1
		// 清除PID相关数据
		motor->motor_pid.interface->reset(&motor->motor_pid);
#endif

		return DRV_OK;
	}
	if (speed<0){
		converted_speed	= -speed;
		dir				= -motor->step_motor_information.dir;
	} else{
		converted_speed	= speed;
	}

	// 设置电机方向（step_motor_stop 后 current_frequency=0，方向已空闲）
	step_motor_set_direction(motor, dir);

	// 根据转速获取实际转动频率
	uint16_t target_freq = motor_speed_to_freq(converted_speed,
						motor->step_motor_information.step_model);
	if (target_freq == 0) return DRV_ERR_PARAM;
	// 如果频率过大, 设置为支持的电机最大频率
	if (target_freq>MAX_PWM_FREQUENCY_HZ) target_freq	= MAX_PWM_FREQUENCY_HZ;

#if USE_MOTOR_PID_CONTROL==0
	// 如果不使用闭环控制, 则需要设置
	// 配置斜坡参数（ACCEL/DECEL 判断交给 start 根据 ramp 内部 freq 完成.
	ramp_step_motor_set(&motor->ramp,
	                    0,		// current_freq: stop 后已归零，从 0 加速到 target
	                    target_freq, 0,
	                    0, RAMP_IDLE);	// 此时ramp->state不重要

	// 启动斜坡（start 内部根据 freq_current/target 自动选 ACCEL/DECEL）
	return ramp_step_motor_start(&motor->ramp, motor);
#elif USE_MOTOR_PID_CONTROL==1
	// 增量式 PID 从当前频率 + 最小启动频率开始
	// 三者（Output / clamp_last_output / 硬件频率）必须一致，否则 PID 第一周期 delta 计算出错
	CRITICAL_ENTER();
	motor->motor_pid.interface->reset(&motor->motor_pid);
	motor->motor_pid.Target = speed;

	// 同方向切换：cur_freq 保留，平滑过渡（避免断崖）
	// 反向切换：step_motor_stop 已将 cur_freq 清零，从 MIN_START_FREQ 重新起步
	uint16_t cur_freq = motor->step_motor_information.current_frequency;
	uint16_t start_freq = cur_freq + MIN_START_FREQ;
	if (start_freq > target_freq) start_freq = target_freq;

	float start_rpm = motor_freq_to_speed(start_freq, motor->step_motor_information.step_model);
	// 反向时
	if (dir == NEGATIVE_DIR) start_rpm = -start_rpm;
	motor->motor_pid.Output     = start_rpm;
	clamp_last_output           = start_rpm;
	CRITICAL_EXIT();
	step_motor_set_pulse_freq(motor, start_freq);
	step_motor_start(motor);
#endif
	return  DRV_OK;
}

/**
 * @brief  运动指定角度后自动停止（非阻塞）
 * @note   通过 TIM4 更新中断对每个 STEP 脉冲计数，
 *         计数完毕后在中断回调中自动关 PWM + 关中断.
 *         函数默认会先将PWM输出关闭.
 * @param  motor: 步进电机指针（调用方需传非 const 指针）
 * @param  dir:   运动方向
 * @param  angle: 旋转角度（°）
 * @retval device_err_t
 */
device_err_t step_motor_move_angle(step_motor_t* motor,
                                   motor_direction_t dir,
                                   float angle)
{
	if (motor==NULL || angle<=0.0f) return DRV_ERR_NULL;

	// 计算脉冲数
	const uint16_t microstep = (uint16_t)motor->step_motor_information.step_model;

	const float step_angle	= FULL_UP_STEP_LENGTH_ANGLE / (float)microstep;
	uint32_t pulses		= (uint32_t)(angle / step_angle + 0.5f);
	if (pulses==0) pulses	= 1;
	// 获取当前电机频率
	uint16_t freq		= motor->step_motor_information.current_frequency;
	if (freq==0)	freq	= DEFAULT_MOTOR_FREQUENCY_HZ;

	// 设置方向
	step_motor_set_direction(motor, dir);

	// 设置步数限位（开环/闭环通用）
	motor->step_motor_information.step_remaining = pulses;

	// 设置脉冲频率（内部 EGR=UG 会置位 UIF 并清零计数器）
	step_motor_set_pulse_freq(motor, freq);
	// 启动 PWM（CEN=1, 计数器从 0 开始跑）
	if (step_motor_start(motor)!=DRV_OK) {
		return DRV_ERR_IO;
	}

	// 先清除标志位, 防止开始计时立即触发一次中断
	__HAL_TIM_CLEAR_FLAG(motor->motor_base_info.tim_handle, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(motor->motor_base_info.tim_handle, TIM_IT_UPDATE);

	return DRV_OK;
}

/**
 * @brief  实时设置脉冲频率（非阻塞）
 * @note   通过修改 TIM ARR 寄存器实现调速, 立即返回不阻塞
 * @param  motor:        步进电机结构体指针
 * @param  pulse_freq_hz: 目标脉冲频率 (Hz)
 * @retval device_err_t
 */
device_err_t step_motor_set_pulse_freq(step_motor_t* motor, uint16_t pulse_freq_hz)
{
	if (motor == NULL) {
		return DRV_ERR_NULL;
	}

	TIM_HandleTypeDef* htim = motor->motor_base_info.tim_handle;
	if (htim == NULL || pulse_freq_hz == 0) {
		return DRV_ERR_PARAM;
	}

	uint32_t new_arr = motor_freq_to_arr(pulse_freq_hz);
	if (new_arr < 1) {
		new_arr = 1;
	}
	// 更新 ARR 影子寄存器 + CCR 活动寄存器
	__HAL_TIM_SET_AUTORELOAD(htim, new_arr);
	__HAL_TIM_SET_COMPARE(htim, motor->motor_base_info.tim_channel, new_arr / 2);

	htim->Instance->EGR = TIM_EGR_UG;
	motor->step_motor_information.current_frequency = pulse_freq_hz;
	return DRV_OK;
}

/**
 * @brief  设置电机运动方向
 * @param  motor: 步进电机结构体指针
 * @param  dir:   目标运动方向
 * @retval device_err_t
 */
device_err_t step_motor_set_direction(step_motor_t* motor, motor_direction_t dir)
{
	if (motor==NULL) return DRV_ERR_NULL;
	if (dir > STOP_DIR) return DRV_ERR_PARAM;
	if (motor->step_motor_information.dir==dir) return DRV_OK;

	/* 仅操作硬件 DIR 引脚 + 更新内部方向标记
	 * 速度 / 换向逻辑由上层 step_motor_set_speed() 负责 */
	if (dir == POSITIVE_DIR) {
		HAL_GPIO_WritePin(motor->motor_base_info.dir_gpio_port,
		                  motor->motor_base_info.dir_gpio_pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(motor->motor_base_info.dir_gpio_port,
		                  motor->motor_base_info.dir_gpio_pin, GPIO_PIN_SET);
	}

	motor->step_motor_information.dir = dir;

	return DRV_OK;
}

/**
 * @brief  内部函数：立即关闭 PWM 并复位电机状态（硬停止）
 * @param  motor: 步进电机结构体指针
 * @retval 无
 * @note   仅供 ramp tick 状态机在减速到 0 后调用，不应从外部 API 直接调用
 */
void step_motor_pwm_off(step_motor_t* motor)
{
	// 先关 TIM 更新中断，再停 PWM（避免空转中断占资源）
	__HAL_TIM_DISABLE_IT(motor->motor_base_info.tim_handle, TIM_IT_UPDATE);
	HAL_TIM_PWM_Stop(motor->motor_base_info.tim_handle, motor->motor_base_info.tim_channel);
	motor->step_motor_information.current_frequency = 0;
	motor->step_motor_information.dir_state	= DIR_NORMAL;
}


#if USE_MOTOR_PID_CONTROL==0
// ======================== 非阻塞加速斜坡（中断驱动, 步进电机驱动内部使用） ========================
/**
 * @brief  设置斜坡参数（不启动电机）
 * @param  ramp:         斜坡状态机指针
 * @param  current_freq: 当前频率 (Hz)
 * @param  target_freq:  目标频率 (Hz)
 * @param  step_number:  限位步数（RAMP_STEP 模式下使用，其他模式传 0）
 * @param  hold_ms:      到达目标后保持时间 (ms)，0 = 不停留（持续运行）
 * @param  state:        初始斜坡状态
 */
void ramp_step_motor_set(motor_ramp_t* ramp,
                         uint32_t current_freq, uint32_t target_freq,
                         uint16_t step_number, uint32_t hold_ms,
                         ramp_state_t state)
{

	ramp->state		= state;
	ramp->freq_current	= current_freq;
	ramp->freq_target	= target_freq;
	ramp->freq_step		= MOTOR_STEP_LENGTH_FREQUENCY_HZ;
	ramp->hold_target	= hold_ms!=0 ? (hold_ms/5):0;   // 5ms 一个 tick
	ramp->hold_ticks	= 0;
	// ramp->step_number	= step_number;  // 步数限位用
}

/**
 * @brief  初始化斜坡参数（仅在 step_motor_init 时调用一次）
 * @param  ramp:         斜坡状态机指针
 * @param  target_freq:  目标频率 (Hz)
 * @param  step_freq:    每 tick 频率增量 (Hz)，tick 间隔 5ms
 * @param  hold_ms:      到达目标后保持时间 (ms)，0 = 不保持直接结束
 * @retval device_err_t
 */

static device_err_t ramp_step_motor_init(motor_ramp_t* ramp,
                                         uint32_t target_freq, uint32_t step_freq, uint32_t hold_ms)
{
	if (ramp==NULL){
		return DRV_ERR_NULL;
	}
	ramp->state       = RAMP_IDLE;
	ramp->freq_current = 0;
	ramp->freq_target  = target_freq;
	ramp->freq_step    = step_freq;
	ramp->hold_target  = 0;   // 5ms 一个 tick
	ramp->hold_ticks   = 0;

	return DRV_OK;
}


/**
 * @brief  启动斜坡状态机（加速 → 保持 → 减速 → 停止）
 * @param  ramp:  斜坡状态机指针
 * @param  motor: 步进电机指针
 * @retval device_err_t
 * @note   本质上只是切换 ramp 的状态，具体逻辑由 systick 回调中的
 *         ramp_step_motor_tick() 每 5ms 驱动执行
 * @note   target_freq == 0 且 current_freq > 0 → 直接进入减速到 0
 * @note   函数假设电机运动方向未改变
 */
static device_err_t ramp_step_motor_start(motor_ramp_t* ramp, step_motor_t* motor)
{
	if (motor == NULL) {
		return DRV_ERR_NULL;
	}
	if (ramp->freq_target == 0 && ramp->freq_current == 0) {
		ramp->state = RAMP_IDLE;
		return DRV_OK;
	}
	if (ramp->freq_target == 0 && ramp->freq_current > 0) {
		// 减速到 0，需要先确保 PWM 已开启
		step_motor_start(motor);
		ramp->state = RAMP_DECEL;
		return DRV_OK;
	}

	// 开启PWM输出
	step_motor_start(motor);
	// 设置状态：根据 freq_target 和 freq_current 确定是加速还是减速
	if (ramp->freq_target > ramp->freq_current) {
		ramp->state = RAMP_ACCEL;
	} else if (ramp->freq_target < ramp->freq_current) {
		ramp->state = RAMP_DECEL;
	} else {
		// freq_target == freq_current 且 > 0：保持速度（不再走进 ACCEL/DECEL）
		ramp->state = RAMP_HOLD;
	}

	return DRV_OK;
}

/**
 * @brief  斜坡状态机 tick（每 5ms 调用一次，从中断/回调中调用）
 * @param  ramp:  斜坡状态机指针
 * @param  motor: 步进电机指针
 * @retval device_err_t
 * @note   状态转换:
 *   RAMP_IDLE  → (无操作)
 *   RAMP_ACCEL → freq += step，直到到达 target → RAMP_HOLD
 *   RAMP_HOLD  → 计数倒计时                    → RAMP_DECEL
 *   RAMP_DECEL → freq -= step，直到 0          → RAMP_IDLE
 */
device_err_t ramp_step_motor_tick(motor_ramp_t* ramp, step_motor_t* motor)
{
	if (ramp->state == RAMP_IDLE || ramp->state == RAMP_DONE) {
		return DRV_OK;
	}

	switch (ramp->state) {
	case RAMP_ACCEL:
		ramp->freq_current += ramp->freq_step;
		if (ramp->freq_current >= ramp->freq_target) {
			ramp->freq_current = ramp->freq_target;
			if (ramp->hold_target > 0) {
				ramp->state = RAMP_HOLD;
				ramp->hold_ticks = ramp->hold_target;

			}
			// TODO: 此时一直是加速状态.
			// hold_target==0 → 保持当前速度，不再自动进入 DECEL
		}
		step_motor_set_pulse_freq(motor, ramp->freq_current);
		break;

	case RAMP_HOLD:
		if (ramp->hold_ticks > 0) {
			ramp->hold_ticks--;
			if (ramp->hold_ticks == 0) {
				ramp->state = RAMP_DECEL;  // 倒数结束，开始减速
			}
		}
		break;

	case RAMP_DECEL:
		if (ramp->freq_current > ramp->freq_step) {
			ramp->freq_current -= ramp->freq_step;
		} else {
			ramp->freq_current = 0;
			ramp->state = RAMP_IDLE;
		}
		step_motor_set_pulse_freq(motor, ramp->freq_current);
		if (ramp->freq_current == 0) {
			// 关闭硬件输出, 省资源
			step_motor_pwm_off(motor);
			motor->step_motor_information.dir_state	= DIR_NORMAL;
		}
		break;
	case RAMP_STEP:
		break;
	default:
		return DRV_ERR_PARAM;

	}


	return DRV_OK;
}
#endif

/* ======================== PID 控制 ======================== */

#if USE_MOTOR_PID_CONTROL==1

/**
 * @brief  限速 + 限幅
 * @param  raw: PID 原始输出
 * @retval 限速限幅后的输出
 */
float pid_output_clamp(float raw)
{

	float delta = raw - clamp_last_output;
	if (delta >  MOTOR_ACCEL_LIMIT) delta =  MOTOR_ACCEL_LIMIT;
	if (delta < -MOTOR_ACCEL_LIMIT) delta = -MOTOR_ACCEL_LIMIT;
	float output = clamp_last_output + delta;
	clamp_last_output = output;

	if (output >  MOTOR_MAX_RPM) output =  MOTOR_MAX_RPM;
	if (output < -MOTOR_MAX_RPM) output = -MOTOR_MAX_RPM;

	return output;
}

/**
 * @brief  将 PID 输出写入电机硬件（方向 + 频率）
 * @note   不经过 step_motor_set_speed，避免覆盖 PID Target
 */
void pid_apply_output(step_motor_t* motor, float output)
{
	if (output > 0.0f){
		step_motor_set_direction(motor, POSITIVE_DIR);
		uint16_t freq = motor_speed_to_freq(output,
			motor->step_motor_information.step_model);
		step_motor_set_pulse_freq(motor, freq);
	} else if (output < 0.0f){
		step_motor_set_direction(motor, NEGATIVE_DIR);
		uint16_t freq = motor_speed_to_freq(-output,
			motor->step_motor_information.step_model);
		step_motor_set_pulse_freq(motor, freq);
	}
}

/**
 * @brief  更新波形共享数据（主循环负责打印）
 */
static void wave_data_update(float target, float actual)
{
	static uint8_t tick = 0;
	if (++tick >= 5){
		tick = 0;
		g_wave_target = target;
		g_wave_actual = actual;
		g_wave_ready = 1;
	}
}

// ---- PID 调试共享变量（ISR 写，主循环读）----
volatile float g_pid_debug_output = 0;    // PID 计算输出
volatile float g_pid_debug_actual = 0;    // 编码器实际速度
volatile float g_pid_debug_error = 0;     // 当前误差
volatile uint16_t g_pid_debug_freq = 0;   // 实际写入的脉冲频率

/**
 * @brief  PID 控制 / 自动调参主逻辑（TIM3 中断，每 PID_DIVIDER 次(Default:2)采样执行一次）
 *
 *  增加失步检测与软重启机制：
 *  当 PID 输出已饱和（推到最大频率）但编码器反馈速度持续很低时，
 *  判定为步进电机失步，自动将频率降到起步频率让电机重新建立同步。
 */
void pid_control_tick(step_motor_t* motor)
{
	static uint8_t pid_tick = 0;
	if (++pid_tick < PID_DIVIDER) return;
	pid_tick = 0;

	if (motor == NULL || g_dev == NULL) return;

	float actual = g_dev->sensor.speed;

	// ---- 自动调参模式 ----
	if (auto_tune_active && !PID_AutoTune_IsDone((PID_AutoTune_t*)&tuner)){
		float output = PID_AutoTune_Calc((PID_AutoTune_t*)&tuner, actual, 0.002f);
		motor_direction_t dir = (output >= 0) ? POSITIVE_DIR : NEGATIVE_DIR;
		step_motor_set_speed(motor, self_fabs(output), dir);
		return;
	}

	// ---- 正常 PID 模式 ----

	// 失步检测状态变量
	static uint8_t stall_count = 0;    // 连续失步计数
	static uint8_t cooldown    = 0;    // 软重启冷却计数

	// 冷却期内：强制使用起步频率，让电机在低速下重新同步
	if (cooldown > 0){
		cooldown--;
		// 保持起步频率，不执行 PID 计算（符号与 Target 一致）
		float cooldown_rpm = (motor->motor_pid.Target >= 0) ? MIN_START_RPM : -MIN_START_RPM;
		pid_apply_output(motor, cooldown_rpm);

		g_pid_debug_output = cooldown_rpm;
		g_pid_debug_actual = actual;
		g_pid_debug_error  = motor->motor_pid.Error;
		g_pid_debug_freq   = motor->step_motor_information.current_frequency;
		return;
	}

	float raw = PID_calc(&motor->motor_pid, actual);

	// 限速 + 限幅（20rpm/周期），防止 Output 飙升太快导致电机失步
	float output = pid_output_clamp(raw);

	// ---- 失步检测与软重启 ----
	// 条件：PID 输出已饱和（接近 OutputMax），但实际速度远低于目标
	// 连续满足条件时触发软重启：回到起步频率让电机重新建立同步
	{
		float target_abs  = self_fabs(motor->motor_pid.Target);
		float stall_limit = (target_abs > 50.0f)
			? target_abs * 0.3f          // 高速目标：实际<目标30%即可能失步
			: STALL_SPEED_THRESHOLD;     // 低速目标：用绝对阈值 30rpm

		if (output >= motor->motor_pid.OutputMax - 1.0f
		    && self_fabs(actual) < stall_limit){
			stall_count++;
			if (stall_count >= STALL_DETECT_CYCLES){
				// 确认失步：触发软重启
				stall_count = 0;
				cooldown    = STALL_RESTART_COOLDOWN;

				float saved_target = motor->motor_pid.Target;
				CRITICAL_ENTER();
				motor->motor_pid.interface->reset(&motor->motor_pid);
				motor->motor_pid.Target = saved_target;
				float restart_rpm = (saved_target >= 0) ? MIN_START_RPM : -MIN_START_RPM;
				motor->motor_pid.Output = restart_rpm;
				clamp_last_output       = restart_rpm;
				CRITICAL_EXIT();

				step_motor_set_pulse_freq(motor, MIN_START_FREQ);
				g_pid_debug_output = restart_rpm;
				g_pid_debug_actual = actual;
				g_pid_debug_error  = motor->motor_pid.Error;
				g_pid_debug_freq   = motor->step_motor_information.current_frequency;
				return;
			}
		} else {
			stall_count = 0;
		}
	}

	// 调试：记录 PID 状态
	g_pid_debug_output = output;
	g_pid_debug_actual = actual;
	g_pid_debug_error  = motor->motor_pid.Error;
	g_pid_debug_freq   = motor->step_motor_information.current_frequency;

	pid_apply_output(motor, output);

}
#endif
