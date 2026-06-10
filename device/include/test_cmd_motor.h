/**
 * @file   test_cmd_motor.h
 * @brief  串口命令控制电机测试用例
 *
 * 测试流程:
 *   1. 系统启动后初始化串口 + 电机 + 命令解析器
 *   2. 发送 "H" 查看帮助
 *   3. 发送 "S:300,1" → 电机正转 300rpm
 *   4. 发送 "Q" → 查询当前状态
 *   5. 发送 "P" → 停止
 *   6. 发送 "A:90,1" → 正转 90°
 *   7. 发送 "M:8" → 设置 1/8 细分
 *   8. 重复测试
 *
 * 使用方式:
 *   串口工具（如 SSCOM、PuTTY）连接 USART1，波特率 115200
 *   输入命令后回车发送
 */

#ifndef DEVICE_TEST_CMD_MOTOR_H
#define DEVICE_TEST_CMD_MOTOR_H

#include "driver_step_motor.h"
#include "uart.h"

/**
 * @brief  初始化测试环境（串口 + 电机 + 命令解析器）
 * @param  motor: 电机实例指针
 * @param  uart:  串口实例指针
 * @retval device_err_t
 */
device_err_t test_cmd_motor_init(step_motor_t* motor, uart_base_t* uart);

/**
 * @brief  测试主循环（在 main while(1) 中调用）
 * @note   非阻塞，检查串口命令并执行
 */
void test_cmd_motor_loop(void);

#endif //DEVICE_TEST_CMD_MOTOR_H
