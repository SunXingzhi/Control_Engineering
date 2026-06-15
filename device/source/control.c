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

// 倒立摆参数
static const float	g		= 9.8f;		// m/s^2    重力加速度
static const float 	l		= 0.20f;	// m        摆杆半长
static const float 	rw		= 0.00636f;	// m        同步轮半径
float			x_ref		= 0.0f;		// m TODO   摆位移参考值（直线模组中点, 校准模式时需要确定）
static float		v_ref		= 0.0f;		// m/s      期望线速度（加速度积分累加）
static uint64_t		last_timeus	= 0;		// us       上次控制循环时间戳
const float		max_rpm		= 420.0f;	// rpm      步进电机最大转速
static float		center_angle	= 180.0f;	// °        摆杆竖直时的传感器角度(中心角度)
const float		slide_table_safety_stroke	= 1.0f;	// TODO 滑台安全行程

// 标定比例尺
float position_scale	= 0.0f;		// 将磁编码器获取的total_angle,转成要的目标位置的比例尺

// 外部变量
extern AngleSensor	sensor1;
extern mt6701_t		encoder;
extern step_motor_t	motor;
extern pendulum_ctx_t pendulum;



/**
 * @brief 获取平台直线速度 (m/s)，通过编码器角速度(RPM) × 同步轮半径
 */
static float get_linear_speed(void)
{
	// encorder.sensor.speed 单位是 RPM，转换为 rad/s 再乘以半径
	return encoder.sensor.speed * (2.0f * 3.1415926f / 60.0f) * rw;
}

/**
 * @brief PID 专用电机输出 — 直接设频率，带启动频率保护和斜率限制
 * @param rpm  目标转速（绝对值）
 * @param dir  方向
 * @note  不经过 ramp 状态机，避免每次重置导致 PID 跟踪失效
 */
static void control_set_motor(float rpm, motor_direction_t dir)
{
	/* 1. RPM → 频率 (Hz) */
	uint16_t target_freq = motor_speed_to_freq(self_fabs(rpm),
	                           motor.step_motor_information.step_model);

	/* 2. 频率钳位 */
	if (target_freq > MAX_PWM_FREQUENCY_HZ)
		target_freq = MAX_PWM_FREQUENCY_HZ;

	/* 3. 启动频率保护：非零时不低于 MIN_START_FREQ */
	if (target_freq > 0 && target_freq < MIN_START_FREQ)
		target_freq = MIN_START_FREQ;

	/* 4. 斜率限制：每 5ms 周期最多变化 MOTOR_STEP_LENGTH_FREQUENCY_HZ (210Hz) */
	static uint16_t current_freq = 0;
	int16_t diff = (int16_t)target_freq - (int16_t)current_freq;
	int16_t max_step = MOTOR_STEP_LENGTH_FREQUENCY_HZ;  // 210
	if (diff > max_step)       diff = max_step;
	else if (diff < -max_step) diff = -max_step;
	current_freq = (uint16_t)((int16_t)current_freq + diff);

	/* 5. 直接控制硬件，绕过 ramp 状态机 */
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
	// 初始化角度环的PID (输入角度, 输出角速度, 限幅 ±4π rad/s ≈ ±12.57)
	PID_init(&PID_theta, PID_POSITIONAL, NULL, 3.1f, 3.1f, 0.3f, +12.57f, -12.57f);

	// 初始化角速度环的PID (输入角速度, 输出角加速度, 限幅 ±125.7 rad/s²)
	PID_init(&PID_theta_dot, PID_POSITIONAL, NULL, 4.1f, 2.1f, 0.7f, +125.7f, -125.7f);

	// 初始化位移环的PID (输入位移, 输出期望倾角, 限幅 ±0.15 rad ≈ ±8.5°)
	PID_init(&PID_x, PID_POSITIONAL, NULL, 2.0f, 0.1f, 1.0f, +0.15f, -0.15f);
}


void control_changelp(float new_lp)
{
	// lp = new_lp;
}

// 控制器进程
void CONTROL_proc()
{
	if (!balance_enabled) return;

	// 下次程序运行的时间
	static uint32_t next = 0;

	// 判断是否到了下次执行时间
	if (HAL_GetTick() < next)
		return;

	// 5ms后再执行一次，
	next += 5;

	// 倒立摆step7_计算轮胎转速.获取时间记录间隔
	// 到时候需要换成hal库的获取时间函数，同时将除数从1000000.0f改为?
	uint64_t nowus = DWT_GetTick_us();
	float deltaT = (nowus - last_timeus) / 1000000.0f;

	// 倒立摆step2_读取角位移传感器的数据，角度转弧度rad/s
	// θ = (传感器角度 - 竖直角度) 转弧度，竖直时 θ=0，右倾为负，左倾为正
	float theta = (AngleSensor_GetAngle(&sensor1) - center_angle) * 0.0174533f;
	float theta_dot = AngleSensor_GetAngularVelocity(&sensor1) * 0.0174533f;

	// 位移环step1_获取当前位移(左侧为相对坐标零点)
	float x = get_linear_position(pendulum.total_angle,
					pendulum.calib.limit_left,
							position_scale);

	// 位移环step2_控制倒立摆的位移
	PID_x.Target = x_ref; // 设置期望位置（x_ref 是全局变量，通常是直线模组的中点位置）
	float theta_ref = PID_calc(&PID_x, x); // 这里 FB 是实际位置 x，计算后得到期望倾角

	// 位移环step3_对theta_ref做额外限幅
	if (theta_ref > 0.15f)
		theta_ref = 0.15f; // 限制在+-8°以内
	if (theta_ref < -0.15f)
		theta_ref = -0.15f;

	// 倒立摆step1_改变角度环的设定值
	PID_theta.Target = theta_ref; // 后面要改，这里为0就是不限制偏移的倒立摆，为theta_ref就是控制在中心的

	// 倒立摆step3_计算外环(角度环)PID的输出角速度
	float theta_dot_ref = PID_calc(&PID_theta, theta);

	// 倒立摆step4_改变内环(角速度环)的设定(参考)值SP
	PID_theta_dot.Target = theta_dot_ref;

	// 倒立摆step5_计算内环(角速度环)PID的输出角加速度
	float theta_dot_dot_ref = PID_calc(&PID_theta_dot, theta_dot);

	// 倒立摆step6_倒立摆的逆解算
	// ẍ = (g·sin(θ) - l·θ̈) / cos(θ)，用l，θ=±90° 时 cos≈0 需保护
	float cos_theta = qcos_rad(theta);
	if (self_fabs(cos_theta) < 0.01f) cos_theta = 0.01f;
	float x_dot_dot_ref = (g * qsin_rad(theta) - theta_dot_dot_ref * l) / cos_theta;

	// 倒立摆step7_加速度积分得到线速度
	// 纯积分，PID 积分项会自动补偿漂移
	if (last_timeus != 0){
		v_ref += x_dot_dot_ref * deltaT;
	}

	// 线速度 → 同步轮角速度 → 电机 RPM
	float omega_ref_local = v_ref / rw;
	float motor_speed_rpm = omega_ref_local * 60.0f / (2.0f * 3.1415926f);
	// 限幅：防止电机转速超过实际可承受范围，避免丢步
	if (motor_speed_rpm > max_rpm)
		motor_speed_rpm = max_rpm;
	if (motor_speed_rpm < -max_rpm)
		motor_speed_rpm = -max_rpm;

	// 倒立摆step8_设置同步轮(电机)的转速（直接设频率，不经过 ramp）
	motor_direction_t dir = (motor_speed_rpm >= 0) ? POSITIVE_DIR : NEGATIVE_DIR;
	control_set_motor(motor_speed_rpm, dir);

	// 调试数据流：θ, θ̇, x, motor_rpm（适合串口绘图器）
	if (debug_stream) {
		char b1[16], b2[16], b3[16], b4[16];
		printf("%s,%s,%s,%s\r\n",
		       ftoa_lite(b1, theta, 4),
		       ftoa_lite(b2, theta_dot, 4),
		       ftoa_lite(b3, x, 4),
		       ftoa_lite(b4, motor_speed_rpm, 1));
	}

	// 倒立摆step9_更新时间值
	last_timeus = nowus;
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
	// 复位暂存值
	last_timeus = 0;
	v_ref = 0.0f;

	// 复位PID控制器
	PID_reset(&PID_theta);
	PID_reset(&PID_theta_dot);
	PID_reset(&PID_x);
}

void control_set_center_angle(float angle)
{
	center_angle = angle;
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

/*
 * @note   建议放在 HAL_Init() 之后、外设初始化之前调用
 */
void DWT_Init(void)
{
	// 使能内核调试跟踪总开关（DWT 属于内核调试组件，必须先开启）
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// 清零周期计数器
	DWT->CYCCNT = 0;
	// 使能周期计数功能，计数器开始随 CPU 时钟自动累加
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  获取微秒级时间戳（对应 us 版本的 HAL_GetTick()）
 * @retval 32位微秒计数值
 */
uint32_t DWT_GetTick_us(void)
{
	// SystemCoreClock 是 HAL 库全局变量，F103 默认 72000000
	return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}
