/**
 * @brief 设备公共信息头, 主程序使用驱动直接包含该头即可, 另外配置了公共函数内容.
 * 同时在device.h中定义了相关驱动的配置信息. 比如是否使用freertos, 是否使用PID闭环控制等等.
 */

#include "../include/device.h"
#include "mt6701.h"
#include "driver_step_motor.h"

// 全局变量定义
extern mt6701_t* g_dev;
extern motor_ramp_t g_ramp;

/**
 * @brief TIM中断回调函数. 目前步进电机用到了TIM3 CH1作为PWM输出, 同时编码器使用TIM4普通计时器模式来计算编码器捕获到的角速度.
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// 电机部分: 只处理 TIM4 的步数限位模式
	if (htim->Instance==TIM4){
		if (g_ramp.state==RAMP_STEP && g_ramp.motor != NULL){
			step_motor_t *motor = g_ramp.motor;

			if (g_ramp.step_number > 0) {
				g_ramp.step_number--;
			}

			if (g_ramp.step_number == 0) {
				// 目标步数到达
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);

				// 关闭PWM
				step_motor_pwm_off(motor);

				// 状态机回到空闲
				g_ramp.state        = RAMP_IDLE;
				g_ramp.freq_current = 0;
				g_ramp.freq_target  = 0;
			}
		}
	}
	// 编码器部分
	else{
		if (g_dev == NULL){
			return;
		}
		if (htim->Instance == g_dev->sensor.htim->Instance){
			angle_sensor_t* s = &g_dev->sensor;

			if (s->first_sample){
				s->first_sample = 0;
				s->angle_last = s->angle;
			}

			float diff = cycle_diff(s->angle - s->angle_last, MT6701_ANGLE_MAX);
			s->angle_last = s->angle;
			s->speed = diff * speed_calc_freq;
			s->speed_raw = (int32_t)(s->speed * 100.0f * (180.0f / 3.1415926f));
		}
	}
}