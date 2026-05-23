//
// Created by q1325 on 2026/5/16.
//

#ifndef TWO_LINK_DRIVER_STEP_MOTOR_H
#define TWO_LINK_DRIVER_STEP_MOTOR_H
#include "public_rule.h"

/* RTOS 条件编译头文件 */
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
		#define DELAY_MS(time_ms)			\
		do{						\
		osDelay(pdMS_TO_TICKS(time_ms));	\
		} while(0)
	#else
		#define DELAY_MS(time_ms)			\
		do{						\
		vTaskDelay(pdMS_TO_TICKS(time_ms));	\
		} while (0)
	#endif
#else
	#if defined(USE_HAL_DRIVER)
		#define DELAY_MS(time_ms)		\
		do{				\
		HAL_Delay(time_ms);		\
		} while(0)
	#endif
#endif

typedef struct step_motor step_motor_t;
#if !defined(USE_FREERTOS)
	/* 非阻塞加速斜坡状态机 */

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
		step_motor_t* motor;
		uint32_t freq_current;
		uint32_t freq_target;
		uint32_t freq_step;
		uint32_t hold_ticks;
		uint32_t hold_target;   // 保持的总 tick 数
		uint16_t step_number;	// 指定的步数前进
	} motor_ramp_t;
#else
	// FREERTOS直接使用软件定时器
	typedef struct motor_ramp {
		ramp_state_t state;
		step_motor_t* motor;
		uint16_t freq_current;
		uint16_t freq_target;
		uint16_t freq_step;
		uint32_t hold_ticks;
		uint32_t hold_target;   // 保持的总 tick 数
	} motor_ramp_t;
#endif

#if defined(USE_FREERTOS)
	#define MOTOR_RAMP_TICK_HOOK()  /* 由 FreeRTOS 软件定时器回调调用 */
#else
	#define MOTOR_RAMP_TICK_HOOK()			\
		do{					\
			motor_ramp_tick(&g_ramp);	\
		} while (0)
#endif

#define PWM_DEFAULT_DUTY_CYCLE			50u	// 默认50, 若要更改需要确定实际脉冲时间是否大于1µs
#define DEFAULT_STEP_MOTOR_DRIVER_MODEL		HR4988

// 电机参数配置
#define START_UP_PWM_FREQUENCY_HZ		1062	// 测量环境24V, 带同步轮
#define DEFAULT_MOTOR_FREQUENCY_HZ		500	// 默认电机频率
#define MAX_PWM_FREQUENCY_HZ			3215	// 电机测量的最大PWM频率
#define MOTOR_STEP_LENGH_FREQUENCY_HZ		210
#define	FULL_UP_STEP_LENTH_ANGLE		1.8f	// 单位:度

// 电机速度控制算法参数配置
#define CONTROL_CYCLE_MS			2u	// 倒立摆推荐控制周期)

static const float PWM_PULSE_TIME_MIN = 0.00001f; // A4988驱动要求STEP脉冲最小要大于1e-6s


// 支持的电机驱动枚举
typedef enum motor_driver_model{
	A4988,
	HR4988
} motor_driver_model_t;

// 电机步进方式枚举
typedef enum motor_step_model{
	DEFAULT_STEP		= 1,
	FULL_STEP		= 1,
	HALF_STEP		= 2,
	ONE_FOURTH_STEP		= 4,
	ONE_EIGHTH_STEP		= 8,
	ONE_SIXTEENTH_STEP	= 16
} motor_step_model_t;

typedef struct motor_base{
	GPIO_TypeDef*	dir_gpio_port;	// 方向引脚端口
	uint16_t	dir_gpio_pin;	// 方向引脚编号
	GPIO_TypeDef*	step_gpio_port;	// 脉冲控制引脚
	uint16_t	step_gpio_pin;	// 脉冲引脚编号
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
} step_motor_information_t;

// 步进电机实例
typedef struct step_motor{
	motor_driver_model_t	driver_model;
	motor_base_t		motor_base_info;
	step_motor_information_t step_motor_information;
} step_motor_t;



device_err_t step_motor_init(step_motor_t* motor);
device_err_t step_motor_deinit(step_motor_t* motor);
device_err_t step_motor_set_step_model(step_motor_t* motor);
device_err_t step_motor_start(step_motor_t* motor);
device_err_t step_motor_stop(step_motor_t* motor);
// device_err_t step_motor_set_speed_size(step_motor_t* motor, uint32_t speed);
device_err_t step_motor_set_speed(step_motor_t* motor, uint32_t speed, motor_direction_t dir);
device_err_t step_motor_move_angle(step_motor_t* motor, motor_direction_t dir, float angle);
device_err_t step_motor_update_angle(const step_motor_t* motor, motor_direction_t dir,
                                     const float angle);
device_err_t step_motor_set_pulse_freq(step_motor_t* motor, uint16_t pulse_freq_hz);
device_err_t step_motor_set_direction(step_motor_t* motor, motor_direction_t dir);

device_err_t step_motor_set_absolute_position(step_motor_t* motor, const uint32_t absolute_position);

void ramp_step_motor_set(motor_ramp_t* ramp, step_motor_t* motor,
		uint32_t current_freq,
		uint32_t target_freq,
		uint16_t step_number,
		uint32_t hold_ms, ramp_state_t state);

#endif //TWO_LINK_DRIVER_STEP_MOTOR_H
