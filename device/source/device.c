/**
 * @brief 设备公共信息头, 主程序使用驱动直接包含该头即可, 另外配置了公共函数内容.
 * 同时在device.h中定义了相关驱动的配置信息. 比如是否使用freertos, 是否使用PID闭环控制等等.
 */

#include "../include/device.h"
#include "mt6701.h"
#include "driver_step_motor.h"

