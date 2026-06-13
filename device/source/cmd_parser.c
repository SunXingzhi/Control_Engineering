/**
 * @file   cmd_parser.c
 * @brief  串口命令解析器实现
 */

#include "../include/cmd_parser.h"
#include "../include/auto_tune.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 模块内部状态 */
static uart_base_t*   s_uart  = NULL;
static step_motor_t*  s_motor = NULL;

/* 自动调参实例（定义在 main.c 中） */
extern volatile PID_AutoTune_t tuner;
extern volatile uint8_t auto_tune_active;
#if defined(USE_FREERTOS)
extern osMessageQueueId_t usart_recv_queueHandle;
#endif

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

#if !defined(USE_FREERTOS)
/**
 * @brief  命令处理主循环
 */
void cmd_parser_process(void)
{
	if (s_uart == NULL || s_motor == NULL) return;
	if (!s_uart->rx_done) return;

	uint8_t buf[128];
	uint16_t n = uart_recv(s_uart, buf, sizeof(buf) - 1);
	if (n == 0) return;

	// 逐字节累积，遇到 \n 就解析一行
	static uint8_t line_buf[64];
	static uint16_t pos = 0;

	for (uint16_t i = 0; i < n; i++){
		if (buf[i] == '\n' || buf[i] == '\r'){
			if (pos > 0){
				line_buf[pos] = '\0';
				cmd_t cmd = parse_cmd(line_buf, pos);
				execute_cmd(&cmd);
				pos = 0;
			}
		} else {
			if (pos < sizeof(line_buf) - 1){
				line_buf[pos++] = buf[i];
			}
		}
	}
}
#else
/**
 * @brief  命令处理主循环
 */
void cmd_parser_process(void)
{
	if (s_uart == NULL || s_motor == NULL) return;
	if (!s_uart->rx_done) return;

	uint8_t buf[128];
	uint16_t n = uart_recv(s_uart, buf, sizeof(buf) - 1);
	if (n == 0) return;

	// 逐字节累积，遇到 \n 就解析一行
	static uint8_t line_buf[64];
	static uint16_t pos = 0;

	for (uint16_t i = 0; i < n; i++){
		if (buf[i] == '\n' || buf[i] == '\r'){
			if (pos > 0){
				line_buf[pos] = '\0';


				cmd_t cmd = parse_cmd(line_buf, pos);
#if defined(USE_FREERTOS)
				// 发送命令到任务队列中(直到发送完成)
				osStatus_t ret	= osMessageQueuePut(usart_recv_queueHandle, &cmd, 0, 0);
				if (ret!=osOK){
					// 占位.
				}
#else
				// 如果不是freertos, 直接执行任务
				execute_cmd(&cmd);
#endif
				pos = 0;
			}
		} else {
			if (pos < sizeof(line_buf) - 1){
				line_buf[pos++] = buf[i];
			}
		}
	}
}
#endif

/**
 * @brief  发送帮助信息
 */
void cmd_send_help(uart_base_t* uart)
{
	uart_base_t* prev = s_uart;
	s_uart = uart;

	SEND_STR("===================== Motor Control Commands ==================\r\n");
	SEND_STR("| S:<rpm>          		Set speed (negative=REV)	|\r\n");
	SEND_STR("| A:<angle>        		Move angle (negative=REV)	|\r\n");
	SEND_STR("| P                		Stop motor			|\r\n");
	SEND_STR("| Q                		Query status			|\r\n");
	SEND_STR("| M:<mode>         		Set step mode (1/2/4/8/16)	|\r\n");
	SEND_STR("| T                		Start auto-tune (relay method)	|\r\n");
	SEND_STR("| R                		Query auto-tune result		|\r\n");
	SEND_STR("| X:<target>,<kp>,<ki>,<kd>	Set PID params + target		|\r\n");
	SEND_STR("| H				Show this help			|\r\n");
	SEND_STR("===============================================================\r\n");

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
	case 'S':  // S:<rpm>  负数表示反向
	case 's':
		cmd.id = CMD_SPEED;
		if (len > 2 && data[1] == ':') {
			char rpm_str[16] = {0};
			uint16_t rpm_len = len - 2;
			if (rpm_len >= sizeof(rpm_str)) rpm_len = sizeof(rpm_str) - 1;
			memcpy(rpm_str, data + 2, rpm_len);
			cmd.param1 = strtof(rpm_str, NULL);
		}
		break;

	case 'A':  // A:<angle>  负数表示反向
	case 'a':
		cmd.id = CMD_ANGLE;
		if (len > 2 && data[1] == ':') {
			char angle_str[16] = {0};
			uint16_t angle_len = len - 2;
			if (angle_len >= sizeof(angle_str)) angle_len = sizeof(angle_str) - 1;
			memcpy(angle_str, data + 2, angle_len);
			cmd.param1 = strtof(angle_str, NULL);
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

	case 'T':  // 启动自动调参
	case 't':
		cmd.id = CMD_AUTOTUNE_START;
		break;

	case 'R':  // 查询调参结果
	case 'r':
		cmd.id = CMD_AUTOTUNE_RESULT;
		break;

	case 'H':  // 帮助
	case 'h':
		cmd.id = CMD_HELP;
		break;

	case 'X':  // X:<target>,<kp>,<ki>,<kd>  target 可以为负数
	case 'x':
		cmd.id = CMD_PID_SETTING;
		if (len > 2 && data[1] == ':') {
			const uint8_t* p = data + 2;
			uint16_t remaining = len - 2;
			float* params[] = {&cmd.param1, &cmd.param3, &cmd.param4, &cmd.param5};

			for (int i = 0; i < 4 && remaining > 0; i++) {
				char tmp[16] = {0};
				const uint8_t* comma = memchr(p, ',', remaining);
				uint16_t field_len = comma ? (uint16_t)(comma - p) : remaining;
				if (field_len >= sizeof(tmp)) field_len = sizeof(tmp) - 1;
				memcpy(tmp, p, field_len);
				*params[i] = strtof(tmp, NULL);
				if (comma) {
					p = comma + 1;
					remaining -= (field_len + 1);
				} else {
					break;
				}
			}
		}
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
	// 设置电机速度命令（负数=反向）
	case CMD_SPEED: {
		float rpm = cmd->param1;

		if (rpm == 0) {
			send_err("invalid rpm (nonzero)");
			return;
		}

		motor_direction_t dir = (rpm > 0) ? POSITIVE_DIR : NEGATIVE_DIR;

		CRITICAL_ENTER();
		auto_tune_active = 0;
		CRITICAL_EXIT();
		step_motor_start(s_motor);
		device_err_t ret = step_motor_set_speed(s_motor, rpm, dir);
		if (ret == DRV_OK) {
			send_ok();
			extern volatile float g_wave_target;
			g_wave_target = rpm;
		} else {
			send_err("set_speed failed");
		}
		break;
	}
	// 指定电机运动角度命令（负数=反向）
	case CMD_ANGLE: {
		float angle = cmd->param1;

		if (angle == 0) {
			send_err("invalid angle (nonzero)");
			return;
		}

		motor_direction_t dir = (angle > 0) ? POSITIVE_DIR : NEGATIVE_DIR;
		if (angle < 0) angle = -angle;

		device_err_t ret = step_motor_move_angle(s_motor, dir, angle);
		if (ret == DRV_OK) {
			send_ok();
		} else {
			send_err("move_angle failed");
		}
		break;
	}
	// 停止命令
	case CMD_STOP: {
		device_err_t ret = step_motor_stop(s_motor);
		if (ret == DRV_OK) {
			send_ok();
		} else {
			send_err("stop failed");
		}
		break;
	}
	// 查询系统状态
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
	// 设置步进模式
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
	// 自动调参开始
	case CMD_AUTOTUNE_START: {
		step_motor_stop(s_motor);
		CRITICAL_ENTER();
		PID_AutoTune_Reset((PID_AutoTune_t*)&tuner);
		auto_tune_active = 1;
		CRITICAL_EXIT();
		SEND_STR("AUTO_TUNE STARTED\r\n");
		SEND_STR("relay=300rpm hyst=50rpm target=200rpm cycles=8\r\n");
		break;
	}
	// 自动调参查询结果
	case CMD_AUTOTUNE_RESULT: {
		CRITICAL_ENTER();
		uint8_t done = PID_AutoTune_IsDone((PID_AutoTune_t*)&tuner);
		const autotune_result_t* r = done ? PID_AutoTune_GetResult((PID_AutoTune_t*)&tuner) : NULL;
		autotune_result_t result_copy = {0};
		if (r != NULL) result_copy = *r;
		CRITICAL_EXIT();

		if (!done) {
			send_err("auto_tune not done yet");
			return;
		}
		if (r == NULL) {
			send_err("no result");
			return;
		}
		char buf[128];
		snprintf(buf, sizeof(buf),
		         "Tu=%.3fs Ku=%.2f Kp=%.4f Ki=%.4f Kd=%.6f\r\n",
		         result_copy.Tu, result_copy.Ku, result_copy.Kp, result_copy.Ki, result_copy.Kd);
		SEND_STR(buf);
		break;
	}
#if USE_MOTOR_PID_CONTROL==1
	// PID 设置目标和参数
	case CMD_PID_SETTING: {
		float target = cmd->param1;
		float kp = cmd->param3;
		float ki = cmd->param4;
		float kd = cmd->param5;

		CRITICAL_ENTER();
		s_motor->motor_pid.Kp = kp;
		s_motor->motor_pid.Ki = ki;
		s_motor->motor_pid.Kd = kd;
		auto_tune_active = 0;
		extern volatile float g_wave_target;
		g_wave_target = target;
		CRITICAL_EXIT();

		step_motor_start(s_motor);
		step_motor_set_speed(s_motor,
		                     target,
		                     target > 0 ? POSITIVE_DIR : NEGATIVE_DIR);

		char buf[96];
		snprintf(buf, sizeof(buf),
		         "SET target=%.1f Kp=%.4f Ki=%.4f Kd=%.6f\r\n",
		         target, kp, ki, kd);
		SEND_STR(buf);
		break;
	}
#endif

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
