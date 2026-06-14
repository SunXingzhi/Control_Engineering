#ifndef LQR_CONTROLLER_CONFIG_H
#define LQR_CONTROLLER_CONFIG_H

/* 参数未完成实机验证前，两个开关都必须保持为 0。 */
#define LQR_MOTOR_OUTPUT_ENABLED        0
#define LQR_PARAMETERS_VERIFIED         0

#define LQR_CONTROL_DT_S                0.002f
#define LQR_SENSOR_TIMEOUT_MS           6u

/* 占位机械参数，完成测量后统一替换。 */
#define LQR_EFFECTIVE_STAGE_MASS_KG     1.0f
#define LQR_PULLEY_TRAVEL_PER_REV_M     0.040f
#define LQR_STAGE_CENTER_ENCODER_RAD    0.0f

/* 当前默认角度原始值单位为度，零点和方向需要实机确认。 */
#define LQR_THETA1_ZERO_RAW             0.0f
#define LQR_THETA2_ZERO_RAW             0.0f
#define LQR_THETA1_SIGN                 1.0f
#define LQR_THETA2_SIGN                 1.0f
#define LQR_ANGLE_RAW_TO_RAD            0.017453292519943295f

/* 安全限制和执行器限制，低输出实机测试后再调整。 */
#define LQR_MAX_STAGE_POSITION_M        0.250f
#define LQR_MAX_ABS_ANGLE_RAD           0.2617993878f
#define LQR_MAX_ABS_FORCE_N             30.0f
#define LQR_MAX_STAGE_SPEED_MPS         0.300f
#define LQR_MAX_MOTOR_RPM               450.0f
#define LQR_MAX_MOTOR_RPM_STEP          20.0f

/* 占位增益，状态顺序为 x_dot、theta1_dot、theta2_dot、x、theta1、theta2。 */
static const float LQR_K[6] = {
	10.983915461f,
	0.703945735f,
	12.106533086f,
	10.000000000f,
	-125.426013281f,
	172.536771922f
};

#endif
