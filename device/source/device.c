/**
 * @brief 设备公共信息头, 主程序使用驱动直接包含该头即可, 另外配置了公共函数内容.
 * 同时在device.h中定义了相关驱动的配置信息. 比如是否使用freertos, 是否使用PID闭环控制等等.
 */

#include "../include/device.h"
#include "mt6701.h"
#include "driver_step_motor.h"

/**
 * @brief 获取位置比例尺
 * @return
 */
float get_positional_scale(const float left_limited_rad, const float right_limted_rad, float slide_table_safety_stroke)
{

	return slide_table_safety_stroke/self_fabs(right_limted_rad - left_limited_rad);;
}


/**
 * @brief 获取平台直线位移 (m).
 * 这里设左边导轨边沿为0点(实际是磁编码器获取到的到达左侧限位时的total_angle值, 向右逐渐递增)
 */
float get_linear_position(float current_absolute_angle_rad, float midnight_angle_rad, float position_scale)
{
	// 当前角度值一定大于左侧限位值

	// 返回以左侧为坐标零点的实际坐标值.
	return (current_absolute_angle_rad - midnight_angle_rad)*position_scale;
}