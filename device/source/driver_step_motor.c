//
// Created by q1325 on 2026/5/16.
//

#include "../include/driver_step_motor.h"

#include "main.h"
#include "device.h"
// 灵活配置tim_clock_freq, 方便后续更改
uint32_t tim_clock_freq	= 0;
extern TIM_HandleTypeDef htim4;
motor_ramp_t g_ramp = {0};

// 内部函数定义
static device_err_t ramp_step_motor_init(motor_ramp_t* ramp,
								step_motor_t* motor,
								uint32_t target_freq,
								uint32_t step_freq,
								uint32_t hold_ms);
static device_err_t ramp_step_motor_start(motor_ramp_t* ramp);
void step_motor_pwm_off(step_motor_t* motor);
static device_err_t step_motor_set_direction(step_motor_t* motor, motor_direction_t dir);

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
inline uint16_t motor_freq_to_arr(uint16_t pulse_freq_hz)
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
	// TODO 初始化PID参数
	// if (PID_init(&motor->motor_pid,
	// 	motor->motor_pid.Kp,
	// 	motor->motor_pid.Ki,
	// 	motor->motor_pid.,
	// 	)==NULL) return DRV_ERR_NULL;
#elif (USE_MOTOR_PID_CONTROL==0)
	// 初始化斜坡参数
	ramp_step_motor_init(&g_ramp, motor, 0, MOTOR_STEP_LENGH_FREQUENCY_HZ, 0);

#endif

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

	// 清除斜坡状态机，防止 SysTick 回调继续用旧状态操作定时器
	g_ramp.state        = RAMP_IDLE;
	g_ramp.freq_current = 0;
	g_ramp.freq_target  = 0;
	g_ramp.step_number  = 0;

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
device_err_t step_motor_set_speed(step_motor_t* motor, const float speed, const motor_direction_t dir)
{
	if (motor == NULL) return DRV_ERR_NULL;
	if (dir>STOP_DIR) return DRV_ERR_PARAM;

	// 如果需要换向，先停止电机（防止电流冲击），方向由 set_direction 在停车后设置
	if (motor->step_motor_information.dir != dir) step_motor_stop(motor);

	// 设置电机方向（step_motor_stop 后 current_frequency=0，方向已空闲）
	step_motor_set_direction(motor, dir);

	// 根据转速获取实际转动频率
	uint16_t target_freq = motor_speed_to_freq(speed,
						motor->step_motor_information.step_model);
	if (target_freq == 0) return DRV_ERR_PARAM;
	// 如果频率过大, 设置为支持的电机最大频率
	if (target_freq>MAX_PWM_FREQUENCY_HZ) target_freq	= MAX_PWM_FREQUENCY_HZ;

	// 配置斜坡参数（ACCEL/DECEL 判断交给 start 根据 ramp 内部 freq 完成.

	ramp_step_motor_set(&g_ramp, motor,
	                    0,		// current_freq: stop 后已归零，从 0 加速到 target
	                    target_freq, 0,
	                    0, RAMP_IDLE);	// 此时ramp->state不重要

	// 启动斜坡（start 内部根据 freq_current/target 自动选 ACCEL/DECEL）
	return ramp_step_motor_start(&g_ramp);
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

	const float step_angle	= FULL_UP_STEP_LENTH_ANGLE / (float)microstep;
	uint32_t pulses		= (uint32_t)(angle / step_angle + 0.5f);
	if (pulses==0) pulses	= 1;
	// 获取当前电机频率
	uint16_t freq		= motor->step_motor_information.current_frequency;
	if (freq==0)	freq	= DEFAULT_MOTOR_FREQUENCY_HZ;

	// 设置方向
	step_motor_set_direction(motor, dir);
	// 配置ramp参数
	ramp_step_motor_set(&g_ramp, motor,
	                    freq, freq,
	                    (uint16_t)pulses,  // step_number
	                    0, RAMP_STEP);

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
 * @brief  更新角度——根据目标角度运动（倒立摆控制用）
 * @note   当前实现为 move_angle 的快捷封装
 * @param  motor: 步进电机指针（const，内部通过全局 g_ramp 访问）
 * @param  dir:   运动方向
 * @param  angle: 目标角度（°）
 * @retval device_err_t
 */
inline device_err_t step_motor_update_angle(const step_motor_t* motor,
                                     motor_direction_t dir,
                                     const float angle)
{
	if (motor == NULL || angle <= 0.0f) return DRV_ERR_NULL;
	if (g_ramp.motor == NULL || g_ramp.motor != motor) return DRV_ERR_PARAM;

	return step_motor_move_angle(g_ramp.motor, dir, angle);
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
static device_err_t step_motor_set_direction(step_motor_t* motor, motor_direction_t dir)
{
	if (motor==NULL) return DRV_ERR_NULL;
	if (dir > STOP_DIR) return DRV_ERR_PARAM;
	if (motor->step_motor_information.dir==dir) return DRV_OK;

	/* 仅操作硬件 DIR 引脚 + 更新内部方向标记
	 * 速度 / 换向逻辑由上层 step_motor_set_speed() 负责 */
	if (dir == POSITIVE_DIR) {
		HAL_GPIO_WritePin(motor->motor_base_info.dir_gpio_port,
		                  motor->motor_base_info.dir_gpio_pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(motor->motor_base_info.dir_gpio_port,
		                  motor->motor_base_info.dir_gpio_pin, GPIO_PIN_RESET);
	}

	motor->step_motor_information.dir = dir;

	return DRV_OK;
}


/**
 * @brief  设置电机绝对位置（不产生实际脉冲, 仅校正位置计数器）
 * @param  motor:             步进电机结构体指针
 * @param  absolute_position: 绝对位置步数值
 * @retval device_err_t
 */
// device_err_t step_motor_set_absolute_position(step_motor_t* motor,
//                                               const uint32_t absolute_position)
// {
// 	if (motor == NULL) {
// 		return DRV_ERR_NULL;
// 	}
//
// 	// TODO: 在 step_motor_information 中增加 position 字段后替换
// 	motor->step_motor_information.current_frequency = absolute_position;
// 	return DRV_OK;
// }

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



// ======================== 非阻塞加速斜坡（中断驱动, 步进电机驱动内部使用） ========================
/**
 * @brief  设置斜坡参数（不启动电机）
 * @param  ramp:         斜坡状态机指针
 * @param  motor:        步进电机指针
 * @param  current_freq: 当前频率 (Hz)
 * @param  target_freq:  目标频率 (Hz)
 * @param  step_number:  限位步数（RAMP_STEP 模式下使用，其他模式传 0）
 * @param  hold_ms:      到达目标后保持时间 (ms)，0 = 不停留（持续运行）
 * @param  state:        初始斜坡状态
 */
inline void ramp_step_motor_set(motor_ramp_t* ramp, step_motor_t* motor,
                         uint32_t current_freq, uint32_t target_freq,
                         uint16_t step_number, uint32_t hold_ms,
                         ramp_state_t state)
{

	ramp->state		= state;
	ramp->motor		= motor;
	ramp->freq_current	= current_freq;
	ramp->freq_target	= target_freq;
	ramp->freq_step		= MOTOR_STEP_LENGH_FREQUENCY_HZ;
	ramp->hold_target	= hold_ms!=0 ? (hold_ms/5):0;   // 5ms 一个 tick
	ramp->hold_ticks	= 0;
	ramp->step_number	= step_number;  // 步数限位用
}

/**
 * @brief  初始化斜坡参数（仅在 step_motor_init 时调用一次）
 * @param  ramp:         斜坡状态机指针
 * @param  motor:        步进电机指针
 * @param  target_freq:  目标频率 (Hz)
 * @param  step_freq:    每 tick 频率增量 (Hz)，tick 间隔 5ms
 * @param  hold_ms:      到达目标后保持时间 (ms)，0 = 不保持直接结束
 * @retval device_err_t
 */
static device_err_t ramp_step_motor_init(motor_ramp_t* ramp, step_motor_t* motor,
                                         uint32_t target_freq, uint32_t step_freq, uint32_t hold_ms)
{
	if (ramp==NULL){
		return DRV_ERR_NULL;
	}
	ramp->state       = RAMP_IDLE;
	ramp->motor       = motor;
	ramp->freq_current = 0;
	ramp->freq_target  = target_freq;
	ramp->freq_step    = step_freq;
	ramp->hold_target  = 0;   // 5ms 一个 tick
	ramp->hold_ticks   = 0;

	return DRV_OK;
}

/**
 * @brief  启动斜坡状态机（加速 → 保持 → 减速 → 停止）
 * @param  ramp: 斜坡状态机指针
 * @retval device_err_t
 * @note   本质上只是切换 ramp 的状态，具体逻辑由 systick 回调中的
 *         ramp_step_motor_tick() 每 5ms 驱动执行
 * @note   target_freq == 0 且 current_freq > 0 → 直接进入减速到 0
 * @note   函数假设电机运动方向未改变
 */
static device_err_t ramp_step_motor_start(motor_ramp_t* ramp)
{
	if (ramp->motor == NULL) {
		return DRV_ERR_NULL;
	}
	if (ramp->freq_target == 0 && ramp->freq_current == 0) {
		ramp->state = RAMP_IDLE;
		return DRV_OK;
	}
	if (ramp->freq_target == 0 && ramp->freq_current > 0) {
		// 减速到 0，需要先确保 PWM 已开启
		step_motor_start(ramp->motor);
		ramp->state = RAMP_DECEL;
		return DRV_OK;
	}

	// 开启PWM输出
	step_motor_start(ramp->motor);
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
 * @brief  非阻塞停止：设置减速斜坡到 0，由 ramp tick 驱动减速 → 自动关 PWM
 * @note   调用后立即返回，不阻塞。减速完成后 ramp 状态机会自动切断 PWM
 * @param  motor: 步进电机结构体指针
 * @retval device_err_t
 */
// static device_err_t ramp_step_motor_stop(motor_ramp_t* ramp)
// {
// 	if (ramp==NULL || ramp->motor==NULL) return DRV_ERR_NULL;
//
// 	ramp->motor->step_motor_information.dir_state	= DIR_STOPPING;
//
// 	// 设置减速斜坡：从当前频率降到 0
// 	ramp_step_motor_set(&g_ramp,
// 		ramp->motor,
// 		ramp->motor->step_motor_information.current_frequency,
// 		0,
// 		0,
// 		0, RAMP_DECEL);
//
// 	return ramp_step_motor_start(&g_ramp);
// }

/**
 * @brief  斜坡状态机 tick（每 5ms 调用一次，从中断/回调中调用）
 * @param  ramp: 斜坡状态机指针
 * @retval device_err_t
 * @note   状态转换:
 *   RAMP_IDLE  → (无操作)
 *   RAMP_ACCEL → freq += step，直到到达 target → RAMP_HOLD
 *   RAMP_HOLD  → 计数倒计时                    → RAMP_DECEL
 *   RAMP_DECEL → freq -= step，直到 0          → RAMP_IDLE
 */
static device_err_t ramp_step_motor_tick(motor_ramp_t* ramp)
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
		step_motor_set_pulse_freq(ramp->motor, ramp->freq_current);
		break;

	case RAMP_HOLD:
		if (ramp->hold_ticks > 0) {
			ramp->hold_ticks--;
		}
		// 继续保持速度
		else if (ramp->hold_ticks==0){
			break;
		} else {
			ramp->state = RAMP_DECEL;
		}
		break;

	case RAMP_DECEL:
		if (ramp->freq_current > ramp->freq_step) {
			ramp->freq_current -= ramp->freq_step;
		} else {
			ramp->freq_current = 0;
			ramp->state = RAMP_IDLE;
		}
		step_motor_set_pulse_freq(ramp->motor, ramp->freq_current);
		if (ramp->freq_current == 0) {
			// 关闭硬件输出, 省资源
			step_motor_pwm_off(ramp->motor);
			ramp->motor->step_motor_information.dir_state	= DIR_NORMAL;
		}
		break;
	case RAMP_STEP:
		break;
	default:
		return DRV_ERR_PARAM;

	}


	return DRV_OK;
}

#if !defined(USE_FREE_RTOS)
	/**
	 * @brief  SysTick 每 1ms 回调 → 每 5ms 驱动一次斜坡状态机
	 *         HAL_IncTick() → HAL_SYSTICK_Callback() 由中断自动调用
	 */
	void HAL_SYSTICK_Callback(void)
	{
		static uint8_t tick_cnt = 0;
		if (++tick_cnt >= 5){
			tick_cnt = 0;
			ramp_step_motor_tick(&g_ramp);
		}
	}
#endif
