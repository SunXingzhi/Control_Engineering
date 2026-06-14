#include "../include/control.h"
#include "PID.h"
#include "angle_sensor.h"
#include <math.h>
#include "qmath.h"

static PID_t PID_theta;	     // θ角度环的PID
static PID_t PID_theta_dot;     // θ.角速度环的PID
static PID_t PID_x;	         // x位置环的PID

static const float g = 9.8;	 // m/s^2    重力加速度
static const float lp = 0.20;	 // m        高度const(一阶摆的摆长)
static const float rw = 0.006;	 // m        同步轮半径
static float omega_ref = 0.0f;	 // rad      角度参考值reference
static float x_ref = 0.0f;	 // m        摆位移参考值 按理说是直线模组的中点
static float v_ref = 0.0f;	 // m/s      期望线速度
static uint64_t last_timeus = 0; // us   记录时间
const float max_rpm = 600.0f;	 // rpm      步进电机的最大转速 测量得到的

// 这里的两个函数是电机提供的测量直线速度和位移的函数
// 这里只做声明 后面替换
float get_linear_position(void); // 单位：m
float get_linear_speed(void);	 // 单位：m/s

extern AngleSensor sensor1;

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

void CONTROL_proc()
{

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
	// 这个地方的角度读取可能需要改，由于规定摆竖直向上时为0度？，下面两条代码需要选择一条进行运行
	// float theta = get_angle() * 0.0174533f;  // / 180 * pi = 0.0174533  这个的意思也就是说函数直接返回的数值是角度
	float theta = (AngleSensor_GetAngle(&sensor1) - 180.0f) * 0.0174533f; // 倒立点 = 0 rad 而我们的角度传感器垂直向下是是0度，竖直向上是应该是180度，所以这里需要-180
	float theta_dot = AngleSensor_GetAngularVelocity(&sensor1) * 0.0174533f;

	// 位移环step1_获取当前位移和线速度 可以通过电机的实际转速 + 同步轮 得到
	float x = get_linear_position();  // 新增：摆的实际位置
	float x_dot = get_linear_speed(); // 新增：摆的实际速度

	// 位移环step2_控制倒立摆的位移
	PID_x.Target = x_ref;                  // 设置期望位置（x_ref 是全局变量，通常是直线模组的中点位置）
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
	float x_dot_dot_ref = (g * qsin_rad(theta) - theta_dot_dot_ref * lp) / qcos_rad(theta);

	// 倒立摆step7_计算轮胎转速.积分累加 单位转换
	// if (last_timeus != 0){
	//    omega_ref += 1.0f / rw * x_dot_dot_ref * deltaT;
	//}

	// 倒立摆step7_计算同步轮转速.积分累加 单位转换
	if (last_timeus != 0)
	{
		v_ref += x_dot_dot_ref * deltaT; // 加速度积分得到线速度
	}

	float omega_ref = v_ref / rw; // 同步轮角速度 (rad/s)
	float motor_speed_rpm = omega_ref * 60.0f / (2.0f * 3.1415926f);
	// 限幅：防止电机转速超过实际可承受范围，避免丢步
	if (motor_speed_rpm > max_rpm)
		motor_speed_rpm = max_rpm;
	if (motor_speed_rpm < -max_rpm)
		motor_speed_rpm = -max_rpm;

	// 倒立摆step8_设置同步轮(电机)的转速
	// 这里先留着，等待步进电机算完了再加上
	// 设置方向（具体哪个方向为正，可能需根据安装方向试验）
	// motor_direction_t dir = (motor_speed_rpm >= 0) ? POSITIVE_DIR : NEGATIVE_DIR;
	// 调用步进电机驱动，传入转速绝对值 (rpm) 和方向
	// step_motor_set_speed(&my_step_motor, (float)fabs(motor_speed_rpm), dir);
	//

	// 倒立摆step9_更新时间值
	last_timeus = nowus;
}

float get_omega_ref()
{

	return omega_ref;
}

uint64_t get_last_timeus()
{

	return last_timeus;
}

void control_reset()
{

	// 复位暂存值
	last_timeus = 0;
	omega_ref = 0;
	v_ref = 0.0f;

	// 复位PID控制器
	PID_reset(&PID_theta);
	PID_reset(&PID_theta_dot);
	PID_reset(&PID_x);
}

/**
 * @brief  初始化 DWT 周期计数器，在 main 函数中调用 1 次即可
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
