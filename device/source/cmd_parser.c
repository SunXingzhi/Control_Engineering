/**
 * @file   cmd_parser.c
 * @brief  涓插彛鍛戒护瑙ｆ瀽鍣ㄥ疄鐜?
 */

#include "../include/cmd_parser.h"
#include "../include/auto_tune.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 妯″潡鍐呴儴鐘舵€?*/
static uart_base_t*   s_uart  = NULL;
static step_motor_t*  s_motor = NULL;

/* 鑷姩璋冨弬瀹炰緥锛堝畾涔夊湪 main.c 涓級 */
extern volatile PID_AutoTune_t tuner;
extern volatile uint8_t auto_tune_active;

/* 鍙戦€佸瓧绗︿覆鐨勪究鎹峰畯 */
#define SEND_STR(s)   uart_send(s_uart, (uint8_t*)(s), strlen(s))

/* ======================== 鍐呴儴鍑芥暟澹版槑 ======================== */
static cmd_t  parse_cmd(const uint8_t* data, uint16_t len);
static void   execute_cmd(const cmd_t* cmd);
static void   send_ok(void);
static void   send_err(const char* reason);
static void   send_float(float val);

/* ======================== 鍏叡 API ======================== */

/**
 * @brief  鍒濆鍖栧懡浠よВ鏋愬櫒
 * @param  uart:  涓插彛瀹炰緥鎸囬拡
 * @param  motor: 鐢垫満瀹炰緥鎸囬拡
 */
void cmd_parser_init(uart_base_t* uart, step_motor_t* motor)
{
	s_uart  = uart;
	s_motor = motor;
}

/**
 * @brief  鍛戒护澶勭悊涓诲惊鐜紙鍦?main while(1) 涓皟鐢級
 */
void cmd_parser_process(void)
{
	if (s_uart == NULL || s_motor == NULL) return;
	// rx_done 判断移入 uart_recv 内部（支持超时帧结束检测）

	uint8_t buf[256];
	uint16_t n = uart_recv(s_uart, buf, sizeof(buf) - 1);
	if (n == 0) return;
	// 纭繚瀛楃涓茬粨灏?
	buf[n] = '\0';

	cmd_t cmd = parse_cmd(buf, n);
	execute_cmd(&cmd);
}

/**
 * @brief  鍙戦€佸府鍔╀俊鎭?
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

/* ======================== 鍛戒护瑙ｆ瀽 ======================== */

/**
 * @brief  瑙ｆ瀽鎺ユ敹鍒扮殑瀛楃涓蹭负鍛戒护缁撴瀯
 * @param  data: 鎺ユ敹缂撳啿鍖?
 * @param  len:  鏁版嵁闀垮害
 * @retval 瑙ｆ瀽鍚庣殑鍛戒护缁撴瀯
 */
static cmd_t parse_cmd(const uint8_t* data, uint16_t len)
{
	cmd_t cmd = {CMD_NONE, 0, 0};

	if (data == NULL || len == 0) return cmd;

	// 璺宠繃鍓嶅绌烘牸鍜?\r
	while (len > 0 && (*data == ' ' || *data == '\r')) {
		data++;
		len--;
	}
	if (len == 0) return cmd;

	// 绗竴涓瓧鑺傛槸鍛戒护瀛楁瘝
	char type = (char)data[0];

	switch (type) {
	case 'S':  // S:<rpm>  璐熸暟琛ㄧず鍙嶅悜
	case 's':
		cmd.id = CMD_SPEED;
		if (len > 2 && data[1] == ':') {
			char rpm_str[16] = {0};
			uint16_t rpm_len = len - 2;
			if (rpm_len >= sizeof(rpm_str)) rpm_len = sizeof(rpm_str) - 1;
			memcpy(rpm_str, data + 2, rpm_len);
			cmd.param1 = strtof(rpm_str, NULL);  // rpm锛堝彲姝ｅ彲璐燂級
		}
		break;

	case 'A':  // A:<angle>  璐熸暟琛ㄧず鍙嶅悜
	case 'a':
		cmd.id = CMD_ANGLE;
		if (len > 2 && data[1] == ':') {
			char angle_str[16] = {0};
			uint16_t angle_len = len - 2;
			if (angle_len >= sizeof(angle_str)) angle_len = sizeof(angle_str) - 1;
			memcpy(angle_str, data + 2, angle_len);
			cmd.param1 = strtof(angle_str, NULL);  // angle锛堝彲姝ｅ彲璐燂級
		}
		break;

	case 'P':  // 鍋滄
	case 'p':
		cmd.id = CMD_STOP;
		break;

	case 'Q':  // 鏌ヨ
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

	case 'T':  // 鍚姩鑷姩璋冨弬
	case 't':
		cmd.id = CMD_AUTOTUNE_START;
		break;

	case 'R':  // 鏌ヨ璋冨弬缁撴灉
	case 'r':
		cmd.id = CMD_AUTOTUNE_RESULT;
		break;

	case 'H':  // 甯姪
	case 'h':
		cmd.id = CMD_HELP;
		break;
	case 'X':  // X:<target>,<kp>,<ki>,<kd>. 娉? target鍙互涓鸿礋鏁?
	case 'x':
		cmd.id = CMD_PID_SETTING;
		if (len > 2 && data[1] == ':') {
			// 瑙ｆ瀽閫楀彿鍒嗛殧鐨?4 涓诞鐐瑰弬鏁?
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

/* ======================== 鍛戒护鎵ц ======================== */

/**
 * @brief  鎵ц瑙ｆ瀽鍚庣殑鍛戒护
 * @param  cmd: 鍛戒护缁撴瀯鎸囬拡
 */
static void execute_cmd(const cmd_t* cmd)
{
	if (cmd == NULL) return;

	switch (cmd->id) {
	// 璁剧疆鐢垫満閫熷害鍛戒护锛堣礋鏁?鍙嶅悜锛?
	case CMD_SPEED: {
		float rpm = cmd->param1;

		if (rpm == 0) {
			send_err("invalid rpm (nonzero)");
			return;
		}

		motor_direction_t dir = (rpm > 0) ? POSITIVE_DIR : NEGATIVE_DIR;

		CRITICAL_ENTER();
		auto_tune_active = 0;  // 閫€鍑鸿嚜鍔ㄨ皟鍙傛ā寮?
		CRITICAL_EXIT();
		step_motor_start(s_motor);  // 纭繚 PWM 宸插惎鍔?
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
	// 鎸囧畾鐢垫満杩愬姩瑙掑害鍛戒护锛堣礋鏁?鍙嶅悜锛?
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
	// 璇锋眰鍋滄鍛戒护
	case CMD_STOP: {
		device_err_t ret = step_motor_stop(s_motor);
		if (ret == DRV_OK) {
			send_ok();
		} else {
			send_err("stop failed");
		}
		break;
	}
	// 璇锋眰褰撳墠绯荤粺淇℃伅鍛戒护
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
	// 璁剧疆姝ヨ繘妯″紡鍛戒护
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
	// 鑷姩璋冨弬寮€濮嬪懡浠?
	case CMD_AUTOTUNE_START: {
		// 鍋滄鐢垫満锛岄噸缃皟鍙傚櫒锛屽惎鍔ㄨ皟鍙傛ā寮?
		step_motor_stop(s_motor);
		// 涓寸晫鍖哄唴鍏?Reset 鍐嶇疆浣嶏紝闃叉 ISR 鍦?Reset 瀹屾垚鍓嶈鍒?active=1
		CRITICAL_ENTER();
		PID_AutoTune_Reset((PID_AutoTune_t*)&tuner);
		auto_tune_active = 1;
		CRITICAL_EXIT();
		SEND_STR("AUTO_TUNE STARTED\r\n");
		SEND_STR("relay=300rpm hyst=50rpm target=200rpm cycles=8\r\n");
		break;
	}
	// 鑷姩璋冨弬鑾峰彇缁撴灉鍛戒护
	case CMD_AUTOTUNE_RESULT: {
		// 涓寸晫鍖轰繚鎶わ紝闃叉 ISR 姝ｅ湪鏇存柊 tuner 鏃朵富寰幆璇诲埌鍗婂啓鍏ョ姸鎬?
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
	// PID 璁剧疆鐩爣鍜屽弬鏁拌繘琛岃皟鍙?
	case CMD_PID_SETTING: {
		// X:<target>,<kp>,<ki>,<kd>
		// param1=target, param3=Kp, param4=Ki, param5=Kd
		float target = cmd->param1;
		float kp = cmd->param3;
		float ki = cmd->param4;
		float kd = cmd->param5;

		// 涓寸晫鍖哄唴鏇存柊 PID 鍙傛暟 + 閫€鍑鸿嚜鍔ㄨ皟鍙傦紝闃叉 ISR 璇诲埌涓嶄竴鑷寸殑涓棿鐘舵€?
		CRITICAL_ENTER();
		s_motor->motor_pid.Kp = kp;
		s_motor->motor_pid.Ki = ki;
		s_motor->motor_pid.Kd = kd;
		auto_tune_active = 0;
		extern volatile float g_wave_target;
		g_wave_target	= target;
		CRITICAL_EXIT();

		// 鍚姩鐢垫満
		step_motor_start(s_motor);
		step_motor_set_speed(s_motor,
						target,
						target>0? POSITIVE_DIR: NEGATIVE_DIR);

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

/* ======================== 搴旂瓟鍑芥暟 ======================== */

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
