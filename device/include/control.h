#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/**
 * @file   control.h
 * @brief  倒立摆平衡控制器 — 串级PID (位移环→角度环→角速度环)
 */

/* 初始化平衡控制系统 */
void control_init(void);

/* 修改摆长参数 */
void control_changelp(float new_lp);

/* 平衡控制主循环 (在 while(1) 中每 5ms 调用一次) */
void CONTROL_proc(void);

/* 复位控制器 (PID状态 + 内部变量) */
void control_reset(void);

/* 获取同步轮角速度参考值 (rad/s) */
float get_omega_ref(void);

/* 获取上次控制循环的时间戳 (us) */
uint64_t get_last_timeus(void);

/**
 * @brief  初始化 DWT 周期计数器，在 main 函数中调用 1 次即可
 * @note   建议放在 HAL_Init() 之后、外设初始化之前调用
 */
void DWT_Init(void);

/**
 * @brief  获取微秒级时间戳 (基于 DWT->CYCCNT，精度 1/72MHz ≈ 14ns)
 * @retval 32位微秒计数值
 */
uint32_t DWT_GetTick_us(void);

#endif /* CONTROL_H */
