/**
 * 速度PID
 *
 */
#ifndef DEVICE_PID_H
#define DEVICE_PID_H


// 前向声明
typedef struct PID PID_t;

// 算法类型
typedef enum PID_ALGO {
	PID_POSITIONAL,     // 位置式
	PID_INCREMENTAL,    // 增量式
} PID_algo_t;

// 算法接口（每个算法实现这套函数）
typedef struct PID_AlgoInterface {
	void* (*init)(void);                              // 分配算法私有状态
	float (*calc)(PID_t* pid, float actual);          // 计算输出
	void  (*reset)(PID_t* pid);                       // 重置状态
	void  (*destroy)(void* state);                    // 释放私有状态
} PID_AlgoInterface_t;

// PID 主结构
typedef struct PID {
	// ---- 公共 ----
	PID_algo_t     pid_algo;      // 使用哪种算法
	void*          algo_state;    // 算法私有状态（不关心具体内容）
	void*          args;          // 领域参数（电机细分模式、传感器量程等）

	float Target;
	float Output;
	float Error;
	float LastError;
	float Kp, Ki, Kd;
	float OutputMax, OutputMin;

	// ---- 函数接口 ----
	const PID_AlgoInterface_t* interface;  // 函数表
} PID_t;



// 统一 API
PID_t* PID_init(PID_t* pid, PID_algo_t algo, void* args,
		float kp, float ki, float kd,
		float output_max, float output_min);
float  PID_calc(PID_t* pid, float actual);
void   PID_reset(PID_t* pid);


#endif //DEVICE_PID_H