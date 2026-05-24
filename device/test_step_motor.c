//
// test_step_motor.c
// 步进电机驱动单元测试实现
//

#include "test_step_motor.h"

// ──────────────────────────────────────────────
// 测试1: 非阻塞匀速运行
// ──────────────────────────────────────────────
void test_step_motor_constant_speed(step_motor_t* motor)
{
	/* 1. 启动电机 */
	if (step_motor_start(motor) != DRV_OK) return;

	/* 2. 加速到 200rpm 正转，匀速运行 2s */
	step_motor_set_speed(motor, 200, POSITIVE_DIR);
	DELAY_MS(2000);

	/* 3. 加速到 400rpm */
	step_motor_set_speed(motor, 400, POSITIVE_DIR);
	DELAY_MS(2000);

	/* 4. 停止 */
	step_motor_stop(motor);
	DELAY_MS(100);
}

// ──────────────────────────────────────────────
// 测试2: 指定角度运行
// ──────────────────────────────────────────────
void test_step_motor_move_angle(step_motor_t* motor)
{
	/* 正转 180° */
	step_motor_move_angle(motor, POSITIVE_DIR, 180.0f);
	DELAY_MS(500);

	/* 正转 90° */
	step_motor_move_angle(motor, POSITIVE_DIR, 90.0f);
	DELAY_MS(500);

	/* 反转 180° */
	step_motor_move_angle(motor, NEGATIVE_DIR, 180.0f);
	DELAY_MS(500);

	/* 反转 360° */
	step_motor_move_angle(motor, NEGATIVE_DIR, 360.0f);
	DELAY_MS(500);
}

// ──────────────────────────────────────────────
// 测试3: 电机方向更改
// ──────────────────────────────────────────────
void test_step_motor_direction(step_motor_t* motor)
{
	/* 正转 2s */
	step_motor_set_speed(motor, 300, POSITIVE_DIR);
	DELAY_MS(2000);

	/* 停止 */
	step_motor_stop(motor);
	DELAY_MS(100);

	/* 反转 2s */
	step_motor_set_speed(motor, 300, NEGATIVE_DIR);
	DELAY_MS(2000);

	/* 停止 */
	step_motor_stop(motor);
	DELAY_MS(100);
}

// ──────────────────────────────────────────────
// 测试4: 持续运行 + 周期性换向
// ──────────────────────────────────────────────
void test_step_motor_periodic_reverse(step_motor_t* motor)
{
	for (int i = 0; i < 4; i++) {
		/* 正转 3s */
		step_motor_set_speed(motor, 400, POSITIVE_DIR);
		DELAY_MS(3000);

		/* 换向: 反转 3s */
		step_motor_set_speed(motor, 400, NEGATIVE_DIR);
		DELAY_MS(3000);
	}

	/* 测试结束停止 */
	step_motor_stop(motor);
	DELAY_MS(100);
}

// ──────────────────────────────────────────────
// 测试5: 设置不同步长模式
// ──────────────────────────────────────────────
void test_step_motor_step_models(step_motor_t* motor)
{
	motor_step_model_t models[] = {
		FULL_STEP,          // 1 细分
		HALF_STEP,          // 2 细分
		ONE_FOURTH_STEP,    // 4 细分
		ONE_EIGHTH_STEP,    // 8 细分
		ONE_SIXTEENTH_STEP  // 16 细分
	};

	for (int i = 0; i < 5; i++) {
		/* 设置细分模式 */
		motor->step_motor_information.step_model = models[i];

		/* 以当前细分模式正转 2s */
		step_motor_set_speed(motor, 200, POSITIVE_DIR);
		DELAY_MS(2000);

		/* 停止后短暂停顿 */
		step_motor_stop(motor);
		DELAY_MS(500);
	}

	/* 恢复默认细分 */
	motor->step_motor_information.step_model = DEFAULT_STEP;
}

// ──────────────────────────────────────────────
// 运行所有测试
// ──────────────────────────────────────────────
void test_step_motor_run_all(step_motor_t* motor)
{
	/* 测试1: 匀速运行 */
	test_step_motor_constant_speed(motor);

	/* 测试2: 指定角度 */
	test_step_motor_move_angle(motor);

	/* 测试3: 方向更改 */
	test_step_motor_direction(motor);

	/* 测试4: 周期性换向 */
	test_step_motor_periodic_reverse(motor);

	/* 测试5: 不同步长 */
	test_step_motor_step_models(motor);
}