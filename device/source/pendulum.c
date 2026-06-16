/**
 * @file   pendulum.c
 * @brief  倒立摆起摆逻辑实现
 *
 * 状态流程：IDLE → CALIBRATE → CALIB_DONE → MOVE_MID → DISTURB → SWING → IDLE
 *
 * 串口命令："001" = 校准限位, "002" = 执行起摆
 */

#include "pendulum.h"
#include "mt6701.h"
#include "angle_sensor.h"
#include "uart.h"
#include "main.h"
#include "device.h"
#include <stdio.h>
#include <string.h>

#include "cmd_parser.h"
// #include "control.h"

/* 外部变量（定义在 main.c 中）*/
extern step_motor_t motor;
extern mt6701_t encoder;
extern AngleSensor sensor1;
extern uart_base_t uart1;
extern volatile uint8_t g_limit_right_flag;
extern volatile uint8_t g_limit_left_flag;

/* ========== 内部函数前向声明 ========== */
static void do_calibration(pendulum_ctx_t* ctx);
static void do_move_to_mid(pendulum_ctx_t* ctx);
static void do_disturb(pendulum_ctx_t* ctx);
static void do_swing(pendulum_ctx_t* ctx);
static uint8_t check_limit_hit(pendulum_ctx_t* ctx, float total_angle, motor_direction_t dir);
static void check_limit_switches(pendulum_ctx_t* ctx);
//
/* ============================================================ */
/*                     状态机主循环入口                           */
/* ============================================================ */

void pendulum_loop(pendulum_ctx_t* ctx, float total_angle, float pendulum_angle)
{
	/* 更新传感器数据 */
	ctx->total_angle = total_angle;
	ctx->pendulum_angle = pendulum_angle;

	/* 限位硬件检测（安全兜底，每次都检查）*/
	check_limit_switches(ctx);
	if (ctx->limit_tripped && ctx->state != STATE_IDLE &&
			ctx->state != STATE_CALIB_DONE && ctx->state != STATE_CALIBRATE &&
			ctx->state != STATE_DISTURB){
		step_motor_stop(&motor);
		ctx->state = STATE_IDLE;
		printf("LIMIT: emergency stop\r\n");
		return;
	}

	/* 状态机调度 */
	switch (ctx->state){
	case STATE_IDLE:
	case STATE_CALIB_DONE:
		break;

	case STATE_CALIBRATE:
		do_calibration(ctx);
		break;

	case STATE_MOVE_MID:
		do_move_to_mid(ctx);
		break;

	case STATE_DISTURB:
		do_disturb(ctx);
		break;

	case STATE_SWING:
		do_swing(ctx);
		break;
	}
}

/* ============================================================ */
/*                        限位校准                               */
/* ============================================================ */

static void do_calibration(pendulum_ctx_t* ctx)
{
	float total_angle = ctx->total_angle;

	switch (ctx->calib_phase){
	case 0: /* 正转中 (POSITIVE_DIR，平台右移)，检测右限位 (PA11) */
		if (LIMIT_RIGHT_IS_HIT()){
			step_motor_stop(&motor);
			ctx->calib.limit_right = total_angle;
			ctx->calib_phase = 1;
			/* 立即反转 (NEGATIVE_DIR)，平台左移，寻找左限位 */
			step_motor_set_speed(&motor, CALIB_SPEED_RPM, NEGATIVE_DIR);
			{
				char bf[16];
				printf("CALIB: right = %s, seeking left...\r\n", ftoa_lite(bf, total_angle, 2));
			}
		}
		break;

	case 1: /* 反转中 (NEGATIVE_DIR，平台左移)，检测左限位 (PA12) */
		if (LIMIT_LEFT_IS_HIT()){
			step_motor_stop(&motor);
			ctx->calib.limit_left = total_angle;
			ctx->calib.limit_center = (ctx->calib.limit_left + ctx->calib.limit_right) / 2.0f;
			ctx->calib.calibrated = 1;
			ctx->calib_phase = 2;
			// 打印校准结果
			{
				char b1[16], b2[16], b3[16];
				printf("CALIB: left=%s right=%s center=%s\r\n",
				       ftoa_lite(b1, ctx->calib.limit_left, 2),
				       ftoa_lite(b2, ctx->calib.limit_right, 2),
				       ftoa_lite(b3, ctx->calib.limit_center, 2));
			}
			// 计算比例尺
			extern float position_scale;
			extern float slide_table_safety_stroke;
			position_scale	= get_positional_scale(ctx->calib.limit_left,
								ctx->calib.limit_right,
								slide_table_safety_stroke);
			// 获取中心x_ref
			extern float x_ref;
			x_ref	= get_linear_position(ctx->calib.limit_center,
							ctx->calib.limit_left,
								position_scale);

		}
		break;

	case 2: /* 新增：移动到中点 */
		{
			float error = ctx->calib.limit_center - total_angle;
			if (self_fabs(error) < MOVE_ARRIVE_THRESH){
				step_motor_stop(&motor);
				ctx->state = STATE_CALIB_DONE;
				char bf[16];
				printf("CALIB: centered at %s, ready\r\n", ftoa_lite(bf, total_angle, 2));
				return;
			}
			if (error > 0)
				step_motor_set_speed(&motor, MOVE_SPEED_RPM, POSITIVE_DIR);
			else
				step_motor_set_speed(&motor, MOVE_SPEED_RPM, NEGATIVE_DIR);
		}
		break;

	}
}

/* ============================================================ */
/*                       移动到中点                              */
/* ============================================================ */

static void do_move_to_mid(pendulum_ctx_t* ctx)
{
	float total_angle = ctx->total_angle;
	float error = ctx->calib.limit_center - total_angle;

	/* 到达判定 */
	if (self_fabs(error) < MOVE_ARRIVE_THRESH){
		step_motor_stop(&motor);
		ctx->state = STATE_DISTURB;
		ctx->disturb_phase = 0;
		ctx->disturb_start_ms = HAL_GetTick();
		/* 开始正转扰动（平台右移）*/
		step_motor_set_speed(&motor, DISTURB_SPEED_RPM, POSITIVE_DIR);
		{
			char bf[16];
			printf("MID: arrived at %s, starting disturb...\r\n", ftoa_lite(bf, total_angle, 2));
		}
		return;
	}

	/* 限位保护 */
	if (check_limit_hit(ctx, total_angle, error > 0 ? POSITIVE_DIR : NEGATIVE_DIR)){
		step_motor_stop(&motor);
		ctx->state = STATE_IDLE;
		printf("ERR: limit hit during move to mid\r\n");
		return;
	}

	/* 设置方向并运动 */
	if (error > 0){
		/* 当前位置在中点左侧，需要向右移动 → POSITIVE_DIR */
		step_motor_set_speed(&motor, MOVE_SPEED_RPM, POSITIVE_DIR);
	}
	else{
		/* 当前位置在中点右侧，需要向左移动 → NEGATIVE_DIR */
		step_motor_set_speed(&motor, MOVE_SPEED_RPM, NEGATIVE_DIR);
	}
}

/* ============================================================ */
/*                      施加初始扰动                             */
/* ============================================================ */

static void do_disturb(pendulum_ctx_t* ctx)
{
	uint32_t elapsed = HAL_GetTick() - ctx->disturb_start_ms;

	switch (ctx->disturb_phase){
	case 0: /* 正转 120ms (POSITIVE_DIR，平台右移) */
		if (elapsed >= DISTURB_DURATION_MS){
			step_motor_stop(&motor);
			ctx->disturb_phase = 1;
			ctx->disturb_start_ms = HAL_GetTick();
			/* 立即反转 (NEGATIVE_DIR，平台左移) */
			step_motor_set_speed(&motor, DISTURB_SPEED_RPM, NEGATIVE_DIR);
		}
		break;

	case 1: /* 反转 120ms (NEGATIVE_DIR，平台左移) */
		if (elapsed >= DISTURB_DURATION_MS){
			step_motor_stop(&motor);
			ctx->disturb_phase = 2;
			/* 初始化起摆参数 */
			ctx->angle_idx = 0;
			ctx->angle_ready = 0;
			ctx->swing_count = 0;
			ctx->push_active = 0;
			memset(ctx->angle_buf, 0, sizeof(ctx->angle_buf));
			ctx->state = STATE_SWING;
			printf("DISTURB: done, entering swing\r\n");
		}
		break;
	}
}

/* ============================================================ */
/*                    小力起摆（核心算法）                         */
/* ============================================================ */

static void do_swing(pendulum_ctx_t* ctx)
{
	/*----- 第一步：采样角度 -----*/
	float angle = ctx->pendulum_angle;
	ctx->angle_buf[ctx->angle_idx] = angle;
	ctx->angle_idx = (ctx->angle_idx + 1) % 3;
	if (ctx->angle_idx == 0) ctx->angle_ready = 1;

	/*----- 第二步：检测是否到达顶部（起摆成功） -----*/
	if (ANGLE_IS_UPSIDE(angle)){
		step_motor_stop(&motor);
		ctx->state = STATE_IDLE;
		{
			char bf[16];
			printf("SWING: upright! angle=%s, done\r\n", ftoa_lite(bf, angle, 1));
		}
		return;
	}

	/*----- 第三步：推力管理 -----*/
	if (ctx->push_active){
		if (HAL_GetTick() - ctx->push_start_ms >= SWING_PUSH_DURATION_MS){
			step_motor_stop(&motor);
			ctx->push_active = 0;
		}
		return; /* 推力期间不检测最高点 */
	}

	/*----- 第四步：最高点检测（3点滑动窗口） -----*/
	if (!ctx->angle_ready) return;

	uint8_t idx0 = (ctx->angle_idx + 0) % 3; /* oldest */
	uint8_t idx1 = (ctx->angle_idx + 1) % 3; /* mid     */
	uint8_t idx2 = (ctx->angle_idx + 2) % 3; /* newest  */

	float a0 = ctx->angle_buf[idx0];
	float a1 = ctx->angle_buf[idx1];
	float a2 = ctx->angle_buf[idx2];

	/* a1 是局部最大值，且已过水平面（90°~270°），排除底部绕回区 */
	if (!(a1 > a0 && a1 > a2 && a1 > 90.0f && a1 < 270.0f)) return;

	/*----- 第五步：决定推力方向 -----*/
	ctx->swing_count++;
	{
		char bf[16];
		printf("SWING: peak #%d at %s deg\r\n", ctx->swing_count, ftoa_lite(bf, a1, 1));
	}

	if (ANGLE_IS_LEFT(a1)){
		/* 摆杆偏左（angle>90°），平台需要向左移动 → NEGATIVE_DIR */
		ctx->swing_push_dir = NEGATIVE_DIR;
	}
	else{
		/* 摆杆偏右（angle≤90°），平台需要向右移动 → POSITIVE_DIR */
		ctx->swing_push_dir = POSITIVE_DIR;
	}

	/*----- 第六步：限位保护 -----*/
	if (check_limit_hit(ctx, ctx->total_angle, ctx->swing_push_dir)){
		printf("SWING: limit hit, skip push\r\n");
		return;
	}

	/*----- 第七步：施加推力 -----*/
	step_motor_set_speed(&motor, SWING_PUSH_SPEED_RPM, ctx->swing_push_dir);
	ctx->push_active = 1;
	ctx->push_start_ms = HAL_GetTick();
}

/* ============================================================ */
/*                        限位保护                               */
/* ============================================================ */

/**
 * @brief 逻辑层限位检查：根据 total_angle 边界和运动方向判断
 */
static uint8_t check_limit_hit(pendulum_ctx_t* ctx, float total_angle, motor_direction_t dir)
{
	if (dir == POSITIVE_DIR && total_angle >= ctx->calib.limit_right - CALIB_MARGIN){
		ctx->limit_tripped = 1;
		step_motor_stop(&motor);
		return 1;
	}
	if (dir == NEGATIVE_DIR && total_angle <= ctx->calib.limit_left + CALIB_MARGIN){
		ctx->limit_tripped = 1;
		step_motor_stop(&motor);
		return 1;
	}
	return 0;
}

/**
 * @brief 硬件层限位IO检测（安全兜底，主循环每次调用）
 *        压下时 IO 为 LOW，未压下时 HIGH
 */
static void check_limit_switches(pendulum_ctx_t* ctx)
{
	if (ctx->state == STATE_CALIBRATE) return; // 校准模式由 do_calibration 自己处理

	if (g_limit_right_flag || g_limit_left_flag){
		g_limit_right_flag = 0;
		g_limit_left_flag = 0;
		step_motor_stop(&motor);
		ctx->limit_tripped = 1;
	}
}

// ============================ 倒立摆命令行实现 ==========================
/**
 * @brief  初始化测试环境
 */
device_err_t cmd_pendulum_init(step_motor_t* motor, uart_base_t* uart, pendulum_ctx_t* ctx)
{
	if (motor == NULL || uart == NULL) return DRV_ERR_NULL;

	// 初始化命令解析器
	cmd_parser_init(uart, motor, ctx);

	// 发送欢迎信息和帮助
	uart_send(uart, (uint8_t*)"\r\n=== Motor CMD Test ===\r\n", 26);
	cmd_send_help(uart);

	return DRV_OK;
}

/**
 * @brief  测试主循环
 */
void cmd_pendulum_loop(void)
{
	cmd_parser_process();
}
