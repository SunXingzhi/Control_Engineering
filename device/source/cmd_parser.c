/**
 * @file   cmd_parser.c
 * @brief  串口命令解析器实现
 */

#include "../include/cmd_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 模块内部状态 */
static uart_base_t*   s_uart  = NULL;
static step_motor_t*  s_motor = NULL;

/* 发送字符串的便捷宏 */
#define SEND_STR(s)   uart_send(s_uart, (uint8_t*)(s), strlen(s))

/* ======================== 内部函数声明 ======================== */
static cmd_t  parse_cmd(const uint8_t* data, uint16_t len);
static void   execute_cmd(const cmd_t* cmd);
static void   send_ok(void);
static void   send_err(const char* reason);
static void   send_float(float val);

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化命令解析器
 * @param  uart:  串口实例指针
 * @param  motor: 电机实例指针
 */
void cmd_parser_init(uart_base_t* uart, step_motor_t* motor)
{
	s_uart  = uart;
	s_motor = motor;
}

/**
 * @brief  命令处理主循环（在 main while(1) 中调用）
 */
void cmd_parser_process(void)
{
	if (s_uart == NULL || s_motor == NULL) return;
	if (!s_uart->rx_done) return;

	uint8_t buf[256];
	uint16_t n = uart_recv(s_uart, buf, sizeof(buf) - 1);
	if (n == 0) return;

	buf[n] = '\0';  // 确保字符串结尾

	cmd_t cmd = parse_cmd(buf, n);
	execute_cmd(&cmd);
}

/**
 * @brief  发送帮助信息
 */
void cmd_send_help(uart_base_t* uart)
{
	uart_base_t* prev = s_uart;
	s_uart = uart;

	SEND_STR("=== Motor Control Commands ===\r\n");
	SEND_STR(" S:<rpm>,<dir>    Set speed (dir: 1=FWD, 2=REV)\r\n");
	SEND_STR(" A:<angle>,<dir>  Move angle (degree)\r\n");
	SEND_STR(" P                Stop motor\r\n");
	SEND_STR(" Q                Query status\r\n");
	SEND_STR(" M:<mode>         Set step mode (1/2/4/8/16)\r\n");
	SEND_STR(" H                Show this help\r\n");
	SEND_STR("==============================\r\n");

	s_uart = prev;
}

/* ======================== 命令解析 ======================== */

/**
 * @brief  解析接收到的字符串为命令结构
 * @param  data: 接收缓冲区
 * @param  len:  数据长度
 * @retval 解析后的命令结构
 */
static cmd_t parse_cmd(const uint8_t* data, uint16_t len)
{
	cmd_t cmd = {CMD_NONE, 0, 0};

	if (data == NULL || len == 0) return cmd;

	// 跳过前导空格和 \r
	while (len > 0 && (*data == ' ' || *data == '\r')) {
		data++;
		len--;
	}
	if (len == 0) return cmd;

	// 第一个字节是命令字母
	char type = (char)data[0];

	switch (type) {
	case 'S':  // S:<rpm>,<dir>
	case 's':
		cmd.id = CMD_SPEED;
		if (len > 2 && data[1] == ':') {
			// 查找逗号分隔 rpm 和 dir
			const uint8_t* comma = memchr(data + 2, ',', len - 2);
			if (comma != NULL) {
				char rpm_str[16] = {0};
				uint16_t rpm_len = comma - data - 2;
				if (rpm_len < sizeof(rpm_str)) {
					memcpy(rpm_str, data + 2, rpm_len);
					cmd.param1 = strtof(rpm_str, NULL);  // rpm
				}
				cmd.param2 = (uint8_t)atoi((const char*)(comma + 1));  // dir
			}
		}
		break;

	case 'A':  // A:<angle>,<dir>
	case 'a':
		cmd.id = CMD_ANGLE;
		if (len > 2 && data[1] == ':') {
			const uint8_t* comma = memchr(data + 2, ',', len - 2);
			if (comma != NULL) {
				char angle_str[16] = {0};
				uint16_t angle_len = comma - data - 2;
				if (angle_len < sizeof(angle_str)) {
					memcpy(angle_str, data + 2, angle_len);
					cmd.param1 = strtof(angle_str, NULL);  // angle
				}
				cmd.param2 = (uint8_t)atoi((const char*)(comma + 1));  // dir
			}
		}
		break;

	case 'P':  // 停止
	case 'p':
		cmd.id = CMD_STOP;
		break;

	case 'Q':  // 查询
	case 'q':
		cmd.id = CMD_QUERY;
		break;

	case 'M':  // M:<mode>
	case 'm':
		cmd.id = CMD_STEP_MODEL;
		if (len > 2 && data[1] == ':') {
			cmd.param2 = (uint8_t)atoi((const char*)(data + 2));
		}
		break;

	case 'H':  // 帮助
	case 'h':
		cmd.id = CMD_HELP;
		break;

	default:
		break;
	}

	return cmd;
}

/* ======================== 命令执行 ======================== */

/**
 * @brief  执行解析后的命令
 * @param  cmd: 命令结构指针
 */
static void execute_cmd(const cmd_t* cmd)
{
	if (cmd == NULL) return;

	switch (cmd->id) {
	case CMD_SPEED: {
		float rpm = cmd->param1;
		motor_direction_t dir = (cmd->param2 == 2) ? NEGATIVE_DIR : POSITIVE_DIR;

		if (rpm <= 0) {
			send_err("invalid rpm");
			return;
		}

		device_err_t ret = step_motor_set_speed(s_motor, rpm, dir);
		if (ret == DRV_OK) {
			send_ok();
		} else {
			send_err("set_speed failed");
		}
		break;
	}

	case CMD_ANGLE: {
		float angle = cmd->param1;
		motor_direction_t dir = (cmd->param2 == 2) ? NEGATIVE_DIR : POSITIVE_DIR;

		if (angle <= 0) {
			send_err("invalid angle");
			return;
		}

		device_err_t ret = step_motor_move_angle(s_motor, dir, angle);
		if (ret == DRV_OK) {
			send_ok();
		} else {
			send_err("move_angle failed");
		}
		break;
	}

	case CMD_STOP: {
		device_err_t ret = step_motor_stop(s_motor);
		if (ret == DRV_OK) {
			send_ok();
		} else {
			send_err("stop failed");
		}
		break;
	}

	case CMD_QUERY: {
		char status[128];
		step_motor_information_t* info = &s_motor->step_motor_information;

		const char* dir_str = (info->dir == POSITIVE_DIR) ? "FWD" :
		                      (info->dir == NEGATIVE_DIR) ? "REV" : "STOP";

		const char* model_str = (info->step_model == FULL_STEP) ? "FULL" :
		                        (info->step_model == HALF_STEP) ? "HALF" :
		                        (info->step_model == ONE_FOURTH_STEP) ? "1/4" :
		                        (info->step_model == ONE_EIGHTH_STEP) ? "1/8" :
		                        (info->step_model == ONE_SIXTEENTH_STEP) ? "1/16" : "?";

		snprintf(status, sizeof(status),
		         "DIR=%s FREQ=%u MODE=%s REMAIN=%lu\r\n",
		         dir_str,
		         info->current_frequency,
		         model_str,
		         (unsigned long)info->step_remaining);

		SEND_STR(status);
		break;
	}

	case CMD_STEP_MODEL: {
		uint8_t mode = cmd->param2;
		if (mode != 1 && mode != 2 && mode != 4 && mode != 8 && mode != 16) {
			send_err("invalid mode (1/2/4/8/16)");
			return;
		}

		s_motor->step_motor_information.step_model = (motor_step_model_t)mode;
		step_motor_set_step_model(s_motor);
		send_ok();
		break;
	}

	case CMD_HELP:
		cmd_send_help(s_uart);
		break;

	default:
		send_err("unknown cmd (H for help)");
		break;
	}
}

/* ======================== 应答函数 ======================== */

static void send_ok(void)
{
	SEND_STR("OK\r\n");
}

static void send_err(const char* reason)
{
	SEND_STR("ERR:");
	SEND_STR(reason);
	SEND_STR("\r\n");
}

static void send_float(float val)
{
	char buf[32];
	snprintf(buf, sizeof(buf), "%.3f\r\n", val);
	SEND_STR(buf);
}
