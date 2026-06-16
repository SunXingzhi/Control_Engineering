/**
 * @file   pendulum.h
 * @brief  倒立摆起摆逻辑 — 状态机、参数宏、限位读取宏
 *
 * 硬件假设：
 *   - 电机 POSITIVE_DIR = 逆时针 = total_angle 增加 = 平台右移
 *   - 电机 NEGATIVE_DIR = 顺时针 = total_angle 减少 = 平台左移
 *   - 限位开关压下时 IO 为 LOW（未压下时 HIGH，外部上拉到 3.3V）
 *   - 角度传感器 0°=正下方，180°=正上方，0→180 逆时针（从右侧上去）
 */

#ifndef __PENDULUM_H
#define __PENDULUM_H

#include "stm32f1xx_hal.h"
#include "driver_step_motor.h"
#include "uart.h"

/*========== 状态枚举 ==========*/
typedef enum {
    STATE_IDLE = 0,       // 空闲等待指令
    STATE_CALIBRATE,      // 限位校准中
    STATE_CALIB_DONE,     // 校准完成
    STATE_MOVE_MID,       // 移动到中点
    STATE_DISTURB,        // 施加初始扰动
    STATE_SWING,          // 小力起摆
} system_state_t;

/*========== 限位数据 ==========*/
typedef struct {
    float limit_left;     // 左限位 total_angle（平台在最左时记录，较小值）
    float limit_right;    // 右限位 total_angle（平台在最右时记录，较大值）
    float limit_center;   // 中点 = (left + right) / 2
    float margin;         // 限位保护余量（total_angle 单位）
    uint8_t calibrated;   // 校准完成标志
} calibration_t;

/*========== 起摆上下文 ==========*/
typedef struct {
    system_state_t state;
    calibration_t  calib;

    /* 校准阶段 */
    uint8_t  calib_phase;       // 0=正转寻右限位, 1=反转寻左限位

    /* 扰动阶段 */
    uint32_t disturb_start_ms;
    uint8_t  disturb_phase;     // 0=正转, 1=反转, 2=完成

    /* 起摆阶段 - 角度缓存 */
    float    angle_buf[3];      // 连续3次角度采样
    uint8_t  angle_idx;         // 写入索引
    uint8_t  angle_ready;       // 缓冲区满标志

    /* 起摆阶段 - 方向控制 */
    motor_direction_t swing_push_dir;
    uint32_t push_start_ms;
    uint8_t  push_active;
    uint8_t  swing_count;

    /* 限位保护 */
    uint8_t  limit_tripped;

    /* 传感器数据 */
    float    total_angle;
    float    pendulum_angle;
} pendulum_ctx_t;

/*========== 可调参数宏 ==========*/

/* 校准 */
#define CALIB_SPEED_RPM         100.0f   // 校准移动速度 (RPM)
#define CALIB_MARGIN            5.0f    // 限位保护余量（total_angle 单位）

/* 移动到中点 */
#define MOVE_SPEED_RPM          100.0f   // 中速移动
#define MOVE_ARRIVE_THRESH      2.0f    // 到达中点判定阈值

/* 扰动 */
#define DISTURB_SPEED_RPM       400.0f  // 扰动速度（RPM）
#define DISTURB_DURATION_MS     120     // 单方向扰动持续时间(ms)

/* 起摆推力 */
#define SWING_PUSH_SPEED_RPM    400.0f   // 起摆推力速度（RPM）
#define SWING_PUSH_DURATION_MS  60      // 单次推力时长(ms)

/* 角度阈值 */
#define ANGLE_UPRIGHT           180.0f
#define ANGLE_UPRIGHT_BAND      15.0f   // 起摆成功容差(±)
#define ANGLE_LEFT_THRESH       90.0f   // ≤90° 偏右, >90° 偏左

/*========== 传感器方向宏 ==========*/
// 0°→180° 逆时针，0°在右，90°水平，180°在左
#define ANGLE_IS_UPSIDE(angle)  ((angle) > (180.0f - ANGLE_UPRIGHT_BAND) && \
                                 (angle) < (180.0f + ANGLE_UPRIGHT_BAND))
#define ANGLE_IS_LEFT(angle)    ((angle) > ANGLE_LEFT_THRESH)
#define ANGLE_IS_RIGHT(angle)   ((angle) <= ANGLE_LEFT_THRESH)

/*========== 限位消抖 ==========*/
#define LIMIT_DEBOUNCE_MS       20      // 消抖时间 (ms)，机械开关典型抖动 5~15ms

/*========== 限位 IO 读取宏 ==========*/
// 压下时 IO 为 LOW (GPIO_PIN_RESET)，未压下时 HIGH
#define LIMIT_RIGHT_IS_HIT()  (HAL_GPIO_ReadPin(MOTOR_LIMIT_CLOSE_GPIO_Port, \
                                                 MOTOR_LIMIT_CLOSE_Pin) == GPIO_PIN_RESET)
#define LIMIT_LEFT_IS_HIT()   (HAL_GPIO_ReadPin(MOTOR_LIMIT_REMOTE_GPIO_Port, \
                                                 MOTOR_LIMIT_REMOTE_Pin) == GPIO_PIN_RESET)

/*========== 函数声明 ==========*/

/**
 * @brief 解析串口命令（"001"=校准, "002"=起摆）
 */
void pendulum_parse_command(pendulum_ctx_t *ctx, uint8_t *buf, uint16_t len);

/**
 * @brief 起摆状态机主循环（在 while(1) 中每周期调用一次）
 * @param ctx       起摆上下文
 * @param total_angle 磁编码器累积角度
 * @param pendulum_angle 角度传感器值（滤波后）
 */
void pendulum_loop(pendulum_ctx_t *ctx, float total_angle, float pendulum_angle);


/**
 * @brief  初始化测试环境（串口 + 电机 + 命令解析器）
 * @param  motor: 电机实例指针
 * @param  uart:  串口实例指针
 * @retval device_err_t
 */
device_err_t cmd_pendulum_init(step_motor_t* motor, uart_base_t* uart, pendulum_ctx_t* ctx);

/**
 * @brief  测试主循环（在 main while(1) 中调用）
 * @note   非阻塞，检查串口命令并执行
 */
void cmd_pendulum_loop(void);
#endif /* __PENDULUM_H */
