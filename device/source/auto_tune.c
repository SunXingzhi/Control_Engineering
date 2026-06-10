/**
 * @file   auto_tune.c
 * @brief  PID继电器自动调参模块实现
 * @note   继电器反馈法（Åström-Hägglund）：
 *         1. 用继电器（bang-bang）控制使系统在 setpoint 附近产生等幅振荡
 *         2. 测量振荡周期 Tu 和振幅 Au
 *         3. 计算继电器增益 Ku = 4d / (π·Au)
 *         4. 用 Ziegler-Nichols 公式算出 PID 参数
 *
 *  继电器输出波形:
 *    +d ─┐     ┌───┐     ┌───
 *        └─────┘   └─────┘
 *    -d        ↑ 振荡周期 Tu
 */

#include "../include/auto_tune.h"

// ======================== 内部函数前向声明 ========================
static void _record_half_cycle(PID_AutoTune_t* at);
static void _calculate_params(PID_AutoTune_t* at);


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
                                float setpoint, uint8_t cycles)
{
	if (at == NULL) return DRV_ERR_NULL;
	if (relay_d <= 0.0f || hysteresis <= 0.0f) return DRV_ERR_PARAM;
	if (cycles == 0) return DRV_ERR_PARAM;

	at->relay_amplitude  = relay_d;
	at->hysteresis       = hysteresis;
	at->setpoint         = setpoint;
	at->cycles_required  = cycles;

	at->state            = AUTOTUNE_RELAY_HIGH;
	at->output           = relay_d;

	at->half_cycle_count = 0;
	at->peak_high        = -1e9f;
	at->peak_low         = 1e9f;
	at->last_cross_time  = 0.0f;
	at->period_sum       = 0.0f;
	at->amplitude_sum    = 0.0f;
	at->cycles_count     = 0;
	at->timestamp        = 0.0f;

	at->result.Ku = 0.0f;
	at->result.Tu = 0.0f;
	at->result.Kp = 0.0f;
	at->result.Ki = 0.0f;
	at->result.Kd = 0.0f;

	return DRV_OK;
}

/**
 * @brief  继电器调参主循环（每个控制周期调用一次）
 * @param  at:     调参器实例指针
 * @param  actual: 当前实际值（传感器反馈）
 * @param  dt:     时间步长 (s)
 * @retval 继电器输出值（直接送给执行器）
 * @note   调参完成后继续调用会返回最后一次输出，不会重复计算
 */
float PID_AutoTune_Calc(PID_AutoTune_t* at, float actual, float dt)
{
	if (at == NULL) return 0.0f;

	// 调参已完成，不再处理
	if (at->state == AUTOTUNE_DONE || at->state == AUTOTUNE_IDLE) {
		return at->output;
	}

	at->timestamp += dt;

	// --- 记录极值 ---
	if (actual > at->peak_high) at->peak_high = actual;
	if (actual < at->peak_low)  at->peak_low  = actual;

	// --- 继电器滞环逻辑 ---
	float error = at->setpoint - actual;

	switch (at->state) {
	case AUTOTUNE_RELAY_HIGH:
		at->output = at->relay_amplitude;
		// actual 超过 setpoint + hysteresis → 切换到 -d
		if (error < -at->hysteresis) {
			at->state  = AUTOTUNE_RELAY_LOW;
			at->output = -at->relay_amplitude;
			_record_half_cycle(at);
		}
		break;

	case AUTOTUNE_RELAY_LOW:
		at->output = -at->relay_amplitude;
		// actual 低于 setpoint - hysteresis → 切换到 +d
		if (error > at->hysteresis) {
			at->state  = AUTOTUNE_RELAY_HIGH;
			at->output = at->relay_amplitude;
			_record_half_cycle(at);
		}
		break;

	default:
		break;
	}

	// --- 检查是否收集够了 ---
	if (at->cycles_count >= at->cycles_required) {
		_calculate_params(at);
		at->state = AUTOTUNE_DONE;
	}

	return at->output;
}

/**
 * @brief  检查调参是否完成
 * @param  at: 调参器实例指针
 * @retval 1 = 完成，0 = 未完成
 */
uint8_t PID_AutoTune_IsDone(const PID_AutoTune_t* at)
{
	if (at == NULL) return 0;
	return (at->state == AUTOTUNE_DONE) ? 1 : 0;
}

/**
 * @brief  获取调参结果
 * @param  at: 调参器实例指针
 * @retval 调参结果结构体指针；未完成时返回 NULL
 */
const autotune_result_t* PID_AutoTune_GetResult(const PID_AutoTune_t* at)
{
	if (at == NULL) return NULL;
	if (at->state != AUTOTUNE_DONE) return NULL;
	return &at->result;
}

/**
 * @brief  复位调参器（可重新开始新一轮调参）
 * @param  at: 调参器实例指针
 * @retval device_err_t
 */
device_err_t PID_AutoTune_Reset(PID_AutoTune_t* at)
{
	if (at == NULL) return DRV_ERR_NULL;

	// 保留用户的配置参数，只清除运行时状态
	float relay_d    = at->relay_amplitude;
	float hyst       = at->hysteresis;
	float sp         = at->setpoint;
	uint8_t cycles   = at->cycles_required;

	at->state            = AUTOTUNE_RELAY_HIGH;
	at->output           = relay_d;
	at->half_cycle_count = 0;
	at->peak_high        = -1e9f;
	at->peak_low         = 1e9f;
	at->last_cross_time  = 0.0f;
	at->period_sum       = 0.0f;
	at->amplitude_sum    = 0.0f;
	at->cycles_count     = 0;
	at->timestamp        = 0.0f;

	// 清除旧结果
	at->result.Ku = 0.0f;
	at->result.Tu = 0.0f;
	at->result.Kp = 0.0f;
	at->result.Ki = 0.0f;
	at->result.Kd = 0.0f;

	(void)relay_d; (void)hyst; (void)sp; (void)cycles;

	return DRV_OK;
}

/**
 * @brief  运行时更新目标值（调参过程中调整振荡中心）
 * @param  at:       调参器实例指针
 * @param  setpoint: 新的目标值
 * @retval device_err_t
 */
device_err_t PID_AutoTune_SetSetpoint(PID_AutoTune_t* at, float setpoint)
{
	if (at == NULL) return DRV_ERR_NULL;
	at->setpoint = setpoint;
	return DRV_OK;
}


// ======================== 内部函数 ========================

/**
 * @brief  记录半个振荡周期的数据
 * @note   每检测到一次继电器切换（HIGH→LOW 或 LOW→HIGH）调用一次。
 *         每两个半周期 = 一个完整周期，累加 period 和 amplitude。
 */
static void _record_half_cycle(PID_AutoTune_t* at)
{
	at->half_cycle_count++;

	// 每两个半周期才算一个完整周期
	if (at->half_cycle_count % 2 == 0) {
		float period    = at->timestamp - at->last_cross_time;
		float amplitude = (at->peak_high - at->peak_low) / 2.0f;

		at->period_sum    += period;
		at->amplitude_sum += amplitude;
		at->cycles_count++;

		// 重置极值记录，准备下一个周期
		at->peak_high = -1e9f;
		at->peak_low  = 1e9f;
	}

	at->last_cross_time = at->timestamp;
}

/**
 * @brief  用 Ziegler-Nichols 公式计算 PID 参数
 *
 *  继电器增益:  Ku = 4d / (π * Au)
 *  振荡周期:   Tu = 平均周期
 *
 *  ┌──────────┬────────────┬──────────────┬───────────────┐
 *  │ 控制器   │    Kp      │     Ki       │     Kd        │
 *  ├──────────┼────────────┼──────────────┼───────────────┤
 *  │ P        │ 0.5·Ku     │ —            │ —             │
 *  │ PI       │ 0.45·Ku    │ 0.54·Ku/Tu   │ —             │
 *  │ PID      │ 0.6·Ku     │ 1.2·Ku/Tu    │ 0.075·Ku·Tu   │
 *  └──────────┴────────────┴──────────────┴───────────────┘
 *
 * @param  at: 调参器实例指针
 */
static void _calculate_params(PID_AutoTune_t* at)
{
	float d  = at->relay_amplitude;
	float Au = at->amplitude_sum / (float)at->cycles_count;
	float Tu = at->period_sum    / (float)at->cycles_count;

	// 防止除零
	if (Au <= 0.0f || Tu <= 0.0f) {
		at->result.Ku = 0.0f;
		at->result.Tu = 0.0f;
		at->result.Kp = 0.0f;
		at->result.Ki = 0.0f;
		at->result.Kd = 0.0f;
		return;
	}

	// 继电器增益
	float Ku = (4.0f * d) / (3.14159f * Au);

	at->result.Ku = Ku;
	at->result.Tu = Tu;

	// Ziegler-Nichols PID 公式
	at->result.Kp = 0.6f  * Ku;
	at->result.Ki = 1.2f  * Ku / Tu;
	at->result.Kd = 0.075f * Ku * Tu;
}
