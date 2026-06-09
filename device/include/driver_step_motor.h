//
// Created by q1325 on 2026/5/16.
//

#ifndef TWO_LINK_DRIVER_STEP_MOTOR_H
#define TWO_LINK_DRIVER_STEP_MOTOR_H
#include "device.h"
#include "PID.h"

// RTOS 条件编译头文件
#ifdef USE_FREERTOS
	#if defined(USE_CMSIS_V2_OS)
		#include "cmsis_os2.h"
	#else
		#include "FreeRTOS.h"
		#include "task.h"
	#endif
#endif


// 根据是否加入了RTOS决定系统级delay的方式
// 如果没有使用FREERTOS, 则DELAY_MS是阻塞的
#ifdef USE_FREERTOS
	#if defined(USE_CMSIS_V2_OS)
		#include "cmsis_os2.h"
		#define DELAY_MS(time_ms)		\
		do{					\
		osDelay(pdMS_TO_TICKS(time_ms));	\
		} while(0)
	#else
		#define DELAY_MS(time_ms)		\
		do{					\
		vTaskDelay(pdMS_TO_TICKS(time_ms));	\
		} while (0)
	#endif
#else
	#if defined(USE_HAL_DRIVER)
		#define DELAY_MS(time_ms)	\
		do{				\
		HAL_Delay(time_ms);		\
		} while(0)
	#endif
#endif


#if !defined(USE_FREERTOS)
	// 非阻塞加速斜坡状态机
	typedef enum ramp_state {
		RAMP_IDLE = 0,
		RAMP_ACCEL,
		RAMP_HOLD,
		RAMP_DECEL,
		RAMP_STEP,
		RAMP_DONE
	} ramp_state_t;

	typedef struct motor_ramp {
		ramp_state_t state;
		uint32_t freq_current;
		uint32_t freq_target;
		uint32_t freq_step;
		uint32_t hold_ticks;
		uint32_t hold_target;   // 保持的总 tick 数
		// uint16_t step_number;	// 指定的步数前进
	} motor_ramp_t;
#else
	// FREERTOS直接使用软件定时器
	typedef struct motor_ramp {
		ramp_state_t state;
		uint16_t freq_current;
		uint16_t freq_target;
		uint16_t freq_step;
		uint32_t hold_ticks;
		uint32_t hold_target;   // 保持的总 tick 数
	} motor_ramp_t;
#endif

// 支持的电机驱动枚举
typedef enum motor_driver_model{
	A4988,
	HR4988
} motor_driver_model_t;

typedef struct motor_base{
	GPIO_TypeDef*	dir_gpio_port;	// 方向引脚端口
	uint16_t	dir_gpio_pin;	// 方向引脚编号
	GPIO_TypeDef*	step_gpio_port;	// 脉冲控制引脚
	uint16_t	step_gpio_pin;	// 脉冲引脚编号
	GPIO_TypeDef*	ms1_gpio_port;	// ms1控制GPIO
	uint16_t	ms1_gpio_pin;
	GPIO_TypeDef*	ms2_gpio_port;
	uint16_t	ms2_gpio_pin;
	GPIO_TypeDef*	ms3_gpio_port;
	uint16_t	ms3_gpio_pin;
	TIM_HandleTypeDef* tim_handle; // 脉冲输出的定时器句柄
	uint32_t tim_channel; // TIM_CHANNEL_1~4
} motor_base_t;

// 步进电机方向
typedef enum motor_direction{
	POSITIVE_DIR = 1, // 正向
	NEGATIVE_DIR = 2,
	STOP_DIR	// 电机停止
} motor_direction_t;

// 步进电机转向状态
typedef enum motor_direction_state{
	DIR_NORMAL = 0,     // 正常运行
	DIR_STOPPING,       // 正在减速到 0，准备换向
	DIR_WAITING,        // 已停，等待 2ms 死区
	DIR_RESTARTING      // 开始反向加速
} motor_direction_state_t;

// 步进电机运动指令 (用于任务间队列通信)
typedef struct motor_command{
	uint32_t target_steps; // 目标步数 (绝对/相对取决于实现)
	motor_direction_t direction;
	uint32_t pulse_freq_hz; // 脉冲频率 (Hz), 控制速度
} motor_command_t;

// 步进电机参数信息(用户可以自定义)
typedef struct step_motor_information{
	uint16_t current_frequency;	// 通过磁编码器获取
	// uint16_t target_frequency;
	motor_step_model_t step_model;
	motor_direction_t dir;
	motor_direction_state_t dir_state;

	// ---- 步数限位（角度运动用，与模式无关） ----
	volatile uint32_t step_remaining;	// 剩余步数（TIM回调中递减）
} step_motor_information_t;

// 步进电机实例
typedef struct step_motor{
	motor_driver_model_t	driver_model;
	motor_base_t		motor_base_info;
	step_motor_information_t step_motor_information;
#if !defined(USE_FREERTOS)
	#if (USE_MOTOR_PID_CONTROL==0)
		motor_ramp_t	ramp;		// 斜坡状态机（仅开环模式）
	#endif
#endif
#if (USE_MOTOR_PID_CONTROL==1)
	PID_t			motor_pid;
#endif
} step_motor_t;

// 相关参数定义
#define PWM_DEFAULT_DUTY_CYCLE			50u	// 默认50, 若要更改需要确定实际脉冲时间是否大于1µs
#define DEFAULT_STEP_MOTOR_DRIVER_MODEL		HR4988

// 电机参数配置
#define START_UP_PWM_FREQUENCY_HZ		1062	// 测量环境24V, 带同步轮
#define DEFAULT_MOTOR_FREQUENCY_HZ		500	// 默认电机频率
#define MAX_PWM_FREQUENCY_HZ			3215	// 电机测量的最大PWM频率
#define MOTOR_STEP_LENGTH_FREQUENCY_HZ		210
#define	FULL_UP_STEP_LENGTH_ANGLE		1.8f	// 单位:度

// 电机速度控制算法参数配置
#define CONTROL_CYCLE_MS			2u	// 倒立摆推荐控制周期)
// 电机 PID 的可靠工作频率上限（实测数据：带载时 1448Hz 以上失步）
// 设置为 1400Hz 留一定安全余量，对应 ~420rpm@FULL_STEP
// 如果更换电机/负载/电压，需要重新标定这个值
#define PID_SAFE_MAX_FREQUENCY_HZ	1400

// 电机PID输出限幅 单位: rpm（对称，支持正反两个方向）
// 使用 PID_SAFE_MAX_FREQUENCY_HZ 而非 MAX_PWM_FREQUENCY_HZ
// PID 输出超过这个值→频率过高→电机失步→encoder=0→误差巨大→输出飙升的恶性循环
#define MOTOR_PID_OUTPUT_MAX(motor_step_model)	motor_freq_to_speed(PID_SAFE_MAX_FREQUENCY_HZ, motor_step_model)
#define MOTOR_PID_OUTPUT_MIN(motor_step_model)	(-(float)motor_freq_to_speed(PID_SAFE_MAX_FREQUENCY_HZ, motor_step_model))

#define MOTOR_ACCEL_LIMIT   20.0f   // rpm/周期，防止启动失步
#define MOTOR_MAX_RPM       960.0f  // 最大输出转速
#define PID_DIVIDER         2       // PID 周期 = 采样周期 × 2

static const float PWM_PULSE_TIME_MIN = 0.00001f; // A4988驱动要求STEP脉冲最小要大于1e-6s


// 工具函数
uint16_t motor_speed_to_freq(float motor_speed_rpm, motor_step_model_t step_model);
uint16_t motor_freq_to_arr(uint16_t pulse_freq_hz);

/* ------------------------------------------*/

// 应用层调用函数
device_err_t step_motor_init(step_motor_t* motor);
device_err_t step_motor_deinit(step_motor_t* motor);
device_err_t step_motor_set_step_model(step_motor_t* motor);
device_err_t step_motor_start(step_motor_t* motor);
device_err_t step_motor_stop(step_motor_t* motor);
device_err_t step_motor_set_speed(step_motor_t* motor,
				float speed,
				motor_direction_t dir);
device_err_t step_motor_move_angle(step_motor_t* motor, motor_direction_t dir, float angle);
device_err_t step_motor_set_pulse_freq(step_motor_t* motor, uint16_t pulse_freq_hz);
device_err_t step_motor_set_direction(step_motor_t* motor, motor_direction_t dir);
void         step_motor_pwm_off(step_motor_t* motor);

#if (USE_MOTOR_PID_CONTROL==0)
	void ramp_step_motor_set(motor_ramp_t* ramp,
			uint32_t current_freq,
			uint32_t target_freq,
			uint16_t step_number,
			uint32_t hold_ms, ramp_state_t state);
	device_err_t ramp_step_motor_tick(motor_ramp_t* ramp, step_motor_t* motor);
#elif (USE_MOTOR_PID_CONTROL==1)
float pid_output_clamp(float raw);
void pid_apply_output(step_motor_t* motor, float output);
void pid_control_tick(step_motor_t* motor);
#endif

#endif //TWO_LINK_DRIVER_STEP_MOTOR_H
