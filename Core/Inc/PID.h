#ifndef PID_H
#define PID_H

#include <stdint.h>

// 我们设定一个结构体，来保存一些长期会用到的数据
// upperlimit、upperlimit用于限幅输出co的上下限，实际意义是（绝对值）不超过电压最大值
typedef struct {
    
    float kp; // 比例系数
    float ki; // 积分项系数
    float kd; // 微分项系数
    float SP; // 用户设定值

    uint64_t t_k_1;     //   t[k-1]上次运行PID的时间
    float err_k_1;      // err[k-1]上次运行PID的误差
    float err_int_k_1;  // err[k-1]上次运行PID的积分值
    
    float upperlimit; // 输出上限 
    float lowerlimit; // 输出下限
    
}PID_struct;


// PID初始化
void PID_init(PID_struct* PID, float kp, float ki, float kd);

// 改变SP值
void PID_changeSP(PID_struct* PID, float sp);

// 进行一次运算
float PID_compute(PID_struct* PID, float FB);

// 改变上下限
void PID_changelimit(PID_struct* PID, float upper, float lower);

// 复位PID
void PID_reset(PID_struct* PID);

// PID测试
void pid_test();
#endif