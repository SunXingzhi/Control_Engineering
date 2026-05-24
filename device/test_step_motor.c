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
	/*
	 * 注意: motor_speed_to_freq 计算: freq = rpm × multiple × 10/3
	 * MAX_PWM_FREQUENCY_HZ = 3215 Hz, 所以各细分模式的最大允许 RPM:
	 *   FULL_STEP   (×1):  3215×3/(1×10)  ≈ 964 rpm → 使用 200 rpm
	 *   HALF_STEP   (×2):  3215×3/(2×10)  ≈ 482 rpm → 使用 200 rpm
	 *   1/4 STEP    (×4):  3215×3/(4×10)  ≈ 241 rpm → 使用 200 rpm
	 *   1/8 STEP    (×8):  3215×3/(8×10)  ≈ 120 rpm → 使用 100 rpm
	 *   1/16 STEP   (×16): 3215×3/(16×10) ≈  60 rpm → 使用  50 rpm
	 *
	 * 使用相同 RPM 则实际轴转速一致（不同的仅是 PWM 频率）。
	 * 高细分模式需降低 RPM 以不超出硬件 PWM 频率上限。
	 */
	typedef struct {
		motor_step_model_t model;
		uint32_t           speed_rpm;
	} test_case_t;

	test_case_t cases[] = {
		{ FULL_STEP,          200 },
		{ HALF_STEP,          200 },
		{ ONE_FOURTH_STEP,    200 },
		{ ONE_EIGHTH_STEP,    100 },
		{ ONE_SIXTEENTH_STEP,  50 },
	};

	for (int i = 0; i < 5; i++) {
		/* 设置细分模式并配置 MS 引脚 */
		motor->step_motor_information.step_model = cases[i].model;
		step_motor_set_step_model(motor);

		/* 以当前细分模式正转 2s */
		step_motor_set_speed(motor, cases[i].speed_rpm, POSITIVE_DIR);
		DELAY_MS(2000);

		/* 停止后短暂停顿 */
		step_motor_stop(motor);
		DELAY_MS(500);
	}

	/* 恢复默认细分 */
	motor->step_motor_information.step_model = DEFAULT_STEP;
	step_motor_set_step_model(motor);
}

// ──────────────────────────────────────────────
// 运行所有测试
// ──────────────────────────────────────────────
void test_step_motor_run_all(step_motor_t* motor)
{
	// /* 测试1: 匀速运行 */
	// test_step_motor_constant_speed(motor);
	//
	// /* 测试2: 指定角度 */
	// test_step_motor_move_angle(motor);
	//
	// /* 测试3: 方向更改 */
	// test_step_motor_direction(motor);
	//
	// /* 测试4: 周期性换向 */
	// test_step_motor_periodic_reverse(motor);

	/* 测试5: 不同步长 */
	test_step_motor_step_models(motor);
}