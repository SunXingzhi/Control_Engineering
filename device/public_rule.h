//
// Created by q1325 on 2026/5/16.
//

#ifndef TWO_LINK_PUBLIC_RULE_H
#define TWO_LINK_PUBLIC_RULE_H
#include "stm32f1xx.h"


#define FLOAT_COMPARATIVE_ACCURACY 4

// 设备处理类
typedef enum device_err{
	DRV_OK = 0,
	DRV_ERR_NULL,
	DRV_ERR_INIT,
	DRV_ERR_TIMEOUT,
	DRV_ERR_PARAM,
	DRV_ERR_BUSY,
	DRV_ERR_IO,
} device_err_t;

// 常见数值运算方法
float self_fabs(float x);

//
#endif //TWO_LINK_PUBLIC_RULE_H
