/**
 * @file   cmd_parser.h
 * @brief  串口命令解析器 —— 电机控制测试用
 *
 * 命令协议（文本格式，以 \n 结尾）:
 *   S:<rpm>             设置速度（负数=反向，如 S:-100）
 *   A:<angle>           走指定角度（°，负数=反向，如 A:-90）
 *   P                   停止电机
 *   Q                   查询电机状态
 *   M:<mode>            设置细分模式（1/2/4/8/16）
 *   T                   启动自动调参（继电器法）
 *   R                   查询自动调参结果
 *   X:<target>,<kp>,<ki>,<kd>  设置PID参数+目标（target可负）
 *   H                   显示帮助
 *
 * 应答:
 *   OK                  命令执行成功
 *   ERR:<reason>        命令执行失败
 *   <数据>              查询结果
 */

#ifndef DEVICE_CMD_PARSER_H
#define DEVICE_CMD_PARSER_H

#include "device.h"
#include "driver_step_motor.h"
#include "uart.h"

/** 命令 ID */
typedef enum CMD_ID {
	CMD_NONE = 0,
	CMD_SPEED,		// S:<rpm>,<dir>
	CMD_ANGLE,		// A:<angle>,<dir>
	CMD_STOP,		// P
	CMD_QUERY,		// Q
	CMD_STEP_MODEL,		// M:<mode>
	CMD_AUTOTUNE_START,	// T
	CMD_AUTOTUNE_RESULT,	// R
	CMD_PID_SETTING,	// X
	CMD_HELP,		// H
} cmd_id_t;

/** 解析后的命令结构 */
typedef struct CMD {
	cmd_id_t	id;
	float		param1;		// rpm 或 angle 或 target
	uint8_t		param2;		// direction 或 step_model 或
	float		param3;		// Kp
	float		param4;		// Ki
	float		param5;		// Kd
} cmd_t;

/**
 * @brief  初始化命令解析器
 * @param  uart: 串口实例指针
 * @param  motor: 电机实例指针
 */
void cmd_parser_init(uart_base_t* uart, step_motor_t* motor);

/**
 * @brief  命令处理主循环（在 main while(1) 中调用）
 * @note   非阻塞，检查 rx_done 标志后解析并执行命令
 */
void cmd_parser_process(void);

/**
 * @brief  发送帮助信息
 */
void cmd_send_help(uart_base_t* uart);

#endif //DEVICE_CMD_PARSER_H
