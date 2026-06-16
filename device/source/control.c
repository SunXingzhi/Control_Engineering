#include "../include/control.h"
#include "PID.h"
#include "angle_sensor.h"
#include "mt6701.h"
#include "pendulum.h"
#include "../include/driver_step_motor.h"
#include "qmath.h"

PID_t PID_theta; // θ角度环的PID
PID_t PID_theta_dot; // θ.角速度环的PID
PID_t PID_x; // x位置环的PID

// 调试控制
static uint8_t balance_enabled = 0;   // 平衡控制器启停标志
static uint8_t debug_stream = 0;      // 实时数据流开关

// 倒立摆物理参数
static const float	g		= 9.8f;		// m/s^2    重力加速度
static const float 	l		= 0.20f;	// m        摆杆半长
static const float 	rw		= 0.00636f;	// m        同步轮半径
float			x_ref		= 0.0f;		// m        摆位移参考值（直线模组中点, 校准模式时需要确定）
static float		v_ref		= 0.0f;		// m/s      期望线速度（加速度积分累加）
static uint64_t		last_timeus	= 0;		// us       上次控制循环时间戳
const float		max_rpm		= 420.0f;	// rpm      步进电机最大转速
static float		center_angle	= 180.0f;	// deg      摆杆竖直时的传感器角度(中心角度)
const float		slide_table_safety_stroke	= 0.45f;	// 滑台安全行程(M)

// 标定比例尺
float position_scale	= 0.0f;		// 将磁编码器获取的total_angle,转成要的目标位置的比例尺

// 外部变量
extern AngleSensor	sensor1;
extern mt6701_t		encoder;
extern step_motor_t	motor;
extern pendulum_ctx_t pendulum;

/**
 * ============================================================
 * 倒立摆控制器 — 基于参考程序策略，适配步进电机
 * ============================================================
 *
 * 参考程序的核心策略：
 *   1. 角度环：纯P控制，输出直接当电机命令
 *   2. 位置环：输出"角度偏移"，调整角度环的目标值
 *   3. 不需要逆动力学、不需要速度积分
 *
 * 适配改动：
 *   - DC电机PWM → 步进电机RPM
 *   - 电位器角度 → 角度传感器
 *   - 编码器位置 → 磁编码器位置
 * ============================================================
 */

/**
 * @brief 获取平台直线速度 (m/s)，通过编码器角速度(RPM) × 同步轮半径
 */
static float get_linear_speed(void)
{
	return encoder.sensor.speed * (2.0f * 3.1415926f / 60.0f) * rw;
}

/**
 * @brief 电机输出 — 带死区补偿和斜率限制
 */
static void control_set_motor(float rpm, motor_direction_t dir)
{
	uint16_t target_freq = motor_speed_to_freq(self_fabs(rpm),
	                           motor.step_motor_information.step_model);

	if (target_freq > MAX_PWM_FREQUENCY_HZ)
		target_freq = MAX_PWM_FREQUENCY_HZ;

	if (target_freq > 0 && target_freq < MIN_START_FREQ)
		target_freq = MIN_START_FREQ;

	static uint16_t current_freq = 0;
	int16_t diff = (int16_t)target_freq - (int16_t)current_freq;
	int16_t max_step = MOTOR_STEP_LENGTH_FREQUENCY_HZ;
	if (diff > max_step)       diff = max_step;
	else if (diff < -max_step) diff = -max_step;
	current_freq = (uint16_t)((int16_t)current_freq + diff);

	if (current_freq == 0) {
		step_motor_pwm_off(&motor);
	} else {
		step_motor_set_direction(&motor, dir);
		step_motor_set_pulse_freq(&motor, current_freq);
		step_motor_start(&motor);
	}
}

// 控制器初始化
void control_init()
{
	// 角度环：纯P控制（参考程序只用0.8的P增益，这里按单位换算）
	// 参考：0.8 × angle_error → PWM
	// 这里：Kp × angle_error → RPM
	PID_init(&PID_theta, PID_POSITIONAL, NULL, 20.0f, 0.0f, 3.0f, +420.0f, -420.0f);

	// 角速度环（暂不使用，参考程序没有角速度环）
	PID_init(&PID_theta_dot, PID_POSITIONAL, NULL, 0.0f, 0.0f, 0.0f, +50.0f, -50.0f);

	// 位移环：输出角度偏移量（参考程序的关键设计）
	PID_init(&PID_x, PID_POSITIONAL, NULL, 0.5f, 0.0f, 0.0f, +0.15f, -0.15f);
}

// 控制器进程
void CONTROL_proc()
{
	if (!balance_enabled) return;

	/* 限位保护 */
	if (LIMIT_RIGHT_IS_HIT() || LIMIT_LEFT_IS_HIT()) {
		control_set_enabled(0);
		printf("BALANCE: limit hit, disabled\r\n");
		return;
	}

	// 5ms控制周期
	static uint32_t next = 0;
	if (HAL_GetTick() < next) return;
	next += 5;

	// ============================================================
	// Step 1: 读取传感器
	// ============================================================
	float theta = (AngleSensor_GetAngle(&sensor1) - center_angle) * 0.0174533f;
	float theta_dot = AngleSensor_GetAngularVelocity(&sensor1) * 0.0174533f;

	// ============================================================
	// Step 2: 位置环 — 输出角度偏移量（参考程序的核心策略）
	// ============================================================
	// 参考程序：location = PID_pos(encoder, target)
	//          v_out = P_angle(raw_angle - location, target_angle)
	// 位置环的输出"欺骗"角度环，让角度环以为摆杆偏了，从而推动平台回中
	float x = get_linear_position(pendulum.total_angle,
					pendulum.calib.limit_left, position_scale);
	PID_x.Target = x_ref;
	float pos_offset = PID_calc(&PID_x, x);  // 输出角度偏移量 (rad)

	// 限幅
	if (pos_offset > 0.15f) pos_offset = 0.15f;
	if (pos_offset < -0.15f) pos_offset = -0.15f;

	// ============================================================
	// Step 3: 角度环 — 纯P控制（参考程序的策略）
	// ============================================================
	// 参考：v_out = 0.8 × (raw_angle - location - target_angle)
	// 这里：output = Kp × (theta - pos_offset - 0)
	//                = Kp × (theta - pos_offset)
	// pos_offset 从 theta 中减去，相当于调整了角度零点
	float angle_error = theta - pos_offset;

	// 角度环 PID（主要用 P 和 D）
	PID_theta.Target = 0.0f;
	float output = PID_calc(&PID_theta, angle_error);

	// 角速度环（可选，提供额外阻尼）
	PID_theta_dot.Target = output;
	output = PID_calc(&PID_theta_dot, theta_dot);

	// ============================================================
	// Step 4: 死区补偿 + 输出限幅
	// ============================================================
	// 参考程序：if (v_out > 0 && v_out < 40) v_out = 40;
	// 步进电机的死区：MIN_START_FREQ 对应的 RPM
	uint16_t dead_zone_rpm = motor_freq_to_speed(MIN_START_FREQ,  motor.step_motor_information.step_model);
	if (output > 0 && output < dead_zone_rpm) output = dead_zone_rpm;
	if (output < 0 && output > -dead_zone_rpm) output = -dead_zone_rpm;

	// 输出限幅
	if (output > max_rpm) output = max_rpm;
	if (output < -max_rpm) output = -max_rpm;

	// ============================================================
	// Step 5: 直接驱动电机（参考程序直接输出PWM，这里直接输出RPM）
	// ============================================================
	motor_direction_t dir = (output >= 0) ? POSITIVE_DIR : NEGATIVE_DIR;
	control_set_motor(self_fabs(output), dir);

	// 调试数据流
	if (debug_stream) {
		char b1[16], b2[16], b3[16], b4[16];
		printf("%s,%s,%s,%s\r\n",
		       ftoa_lite(b1, theta, 4),
		       ftoa_lite(b2, pos_offset, 4),
		       ftoa_lite(b3, angle_error, 4),
		       ftoa_lite(b4, output, 1));
	}
}

float get_omega_ref()
{
	return v_ref / rw;
}

uint64_t get_last_timeus()
{
	return last_timeus;
}

void control_reset()
{
	last_timeus = 0;
	v_ref = 0.0f;

	PID_reset(&PID_theta);
	PID_reset(&PID_theta_dot);
	PID_reset(&PID_x);
}

void control_set_center_angle(float angle)
{
	// center_angle = angle;
	AngleSensor_SetCurrentAngle(&sensor1, angle);
}

void control_set_pid_x(float kp, float ki, float kd)
{
	PID_x.Kp = kp;
	PID_x.Ki = ki;
	PID_x.Kd = kd;
}

void control_set_pid_theta(float kp, float ki, float kd)
{
	PID_theta.Kp = kp;
	PID_theta.Ki = ki;
	PID_theta.Kd = kd;
}

void control_set_pid_theta_dot(float kp, float ki, float kd)
{
	PID_theta_dot.Kp = kp;
	PID_theta_dot.Ki = ki;
	PID_theta_dot.Kd = kd;
}

void control_query_pid(void)
{
	char b1[16], b2[16], b3[16], b4[16], b5[16];
	printf("PID_X:   Kp=%s Ki=%s Kd=%s [%s,%s]\r\n",
	       ftoa_lite(b1, PID_x.Kp, 4), ftoa_lite(b2, PID_x.Ki, 4),
	       ftoa_lite(b3, PID_x.Kd, 4), ftoa_lite(b4, PID_x.OutputMin, 2),
	       ftoa_lite(b5, PID_x.OutputMax, 2));
	printf("PID_T:   Kp=%s Ki=%s Kd=%s [%s,%s]\r\n",
	       ftoa_lite(b1, PID_theta.Kp, 4), ftoa_lite(b2, PID_theta.Ki, 4),
	       ftoa_lite(b3, PID_theta.Kd, 4), ftoa_lite(b4, PID_theta.OutputMin, 2),
	       ftoa_lite(b5, PID_theta.OutputMax, 2));
	printf("PID_TD:  Kp=%s Ki=%s Kd=%s [%s,%s]\r\n",
	       ftoa_lite(b1, PID_theta_dot.Kp, 4), ftoa_lite(b2, PID_theta_dot.Ki, 4),
	       ftoa_lite(b3, PID_theta_dot.Kd, 4), ftoa_lite(b4, PID_theta_dot.OutputMin, 2),
	       ftoa_lite(b5, PID_theta_dot.OutputMax, 2));
	printf("CENTER:  %s deg\r\n", ftoa_lite(b1, center_angle, 2));
}

void control_set_enabled(uint8_t enable)
{
	balance_enabled = enable;
	if (!enable) {
		control_reset();
		step_motor_pwm_off(&motor);
	}
}

uint8_t control_is_enabled(void)
{
	return balance_enabled;
}

void control_set_debug_stream(uint8_t enable)
{
	debug_stream = enable;
}

void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetTick_us(void)
{
	return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}
