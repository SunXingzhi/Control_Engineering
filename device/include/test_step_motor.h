//
// test_step_motor.h
// 步进电机驱动单元测试
//

#ifndef TEST_STEP_MOTOR_H
#define TEST_STEP_MOTOR_H

#include "driver_step_motor.h"

/**
 * @brief 测试1: 非阻塞匀速运行 (step_motor_set_speed)
 *        启动电机 → 加速到指定转速 → 运行 2s → 停止
 */
void test_step_motor_constant_speed(step_motor_t* motor);

/**
 * @brief 测试2: 指定角度运行 (step_motor_move_angle)
 *        停止后 → 走指定角度 → 等待完成 → 重复验证
 */
void test_step_motor_move_angle(step_motor_t* motor);

/**
 * @brief 测试3: 电机方向更改
 *        正转 2s → 停止 → 反转 2s → 停止
 */
void test_step_motor_direction(step_motor_t* motor);

/**
 * @brief 测试4: 持续运行 + 周期性换向
 *        循环：正转 3s → 换向 → 反转 3s
 */
void test_step_motor_periodic_reverse(step_motor_t* motor);

/**
 * @brief 测试5: 设置不同步长模式 (full / half / 1/4 / 1/8 / 1/16)
 *        遍历各细分模式，每种运行 2s
 */
void test_step_motor_step_models(step_motor_t* motor);

/**
 * @brief 运行所有测试（按顺序依次执行）
 * @param motor: 步进电机实例指针
 */
void test_step_motor_run_all(step_motor_t* motor);

#endif // TEST_STEP_MOTOR_H