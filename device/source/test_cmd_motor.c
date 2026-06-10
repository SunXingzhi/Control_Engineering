/**
 * @file   test_cmd_motor.c
 * @brief  串口命令控制电机测试实现
 */

#include "../include/test_cmd_motor.h"
#include "../include/cmd_parser.h"

/**
 * @brief  初始化测试环境
 */
device_err_t test_cmd_motor_init(step_motor_t* motor, uart_base_t* uart)
{
	if (motor == NULL || uart == NULL) return DRV_ERR_NULL;

	// 初始化命令解析器
	cmd_parser_init(uart, motor);

	// 发送欢迎信息和帮助
	uart_send(uart, (uint8_t*)"\r\n=== Motor CMD Test ===\r\n", 26);
	cmd_send_help(uart);

	return DRV_OK;
}

/**
 * @brief  测试主循环
 */
void test_cmd_motor_loop(void)
{
	cmd_parser_process();
}
