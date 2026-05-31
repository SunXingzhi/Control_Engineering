#include "angle_sensor.h"

/**
 * @brief 初始化角度传感器对象
 */
void AngleSensor_Init(AngleSensor* sensor,
                      ADC_HandleTypeDef* hadc,
                      uint32_t Channel,
                      uint32_t SampleTime,
                      float k, float b,
                      float filter_coeff)
{
	sensor->hadc = hadc;
	sensor->Channel = Channel;
	sensor->SampleTime = SampleTime;
	sensor->k = k;
	sensor->b = b;

	/* 滤波初始化 */
	sensor->filter_coeff = (filter_coeff < 0.0f) ? 0.0f : (filter_coeff > 1.0f) ? 1.0f : filter_coeff;
	sensor->filtered_value = 0.0f;
	sensor->filter_initialized = 0;
}

/**
 * @brief 读取ADC原始值
 */
uint32_t AngleSensor_ReadRaw(AngleSensor* sensor)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = sensor->Channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = sensor->SampleTime;

	if (HAL_ADC_ConfigChannel(sensor->hadc, &sConfig) != HAL_OK){
		return 0;
	}

	HAL_ADC_Start(sensor->hadc);
	if (HAL_ADC_PollForConversion(sensor->hadc, 10) != HAL_OK){
		HAL_ADC_Stop(sensor->hadc);
		return 0;
	}

	uint32_t raw = HAL_ADC_GetValue(sensor->hadc);
	HAL_ADC_Stop(sensor->hadc);
	return raw;
}

/**
 * @brief 获取瞬时角度值
 */
float AngleSensor_GetAngle(AngleSensor* sensor)
{
	uint32_t raw = AngleSensor_ReadRaw(sensor);
	return (float)raw * sensor->k + sensor->b;
}

/**
 * @brief 获取经过一阶低通滤波后的角度值
 * @note  公式：Y(n) = α * X(n) + (1-α) * Y(n-1)
 *        其中：α = filter_coeff，X(n) 是本次瞬时值，Y(n-1) 是上次滤波值
 */
float AngleSensor_GetFilteredAngle(AngleSensor* sensor)
{
	float raw_angle = AngleSensor_GetAngle(sensor);

	if (sensor->filter_initialized == 0){
		/* 第一次直接用瞬时值初始化，避免从0缓慢上升 */
		sensor->filtered_value = raw_angle;
		sensor->filter_initialized = 1;
	}
	else{
		/* 指数移动平均滤波 */
		sensor->filtered_value = sensor->filter_coeff * raw_angle +
			(1.0f - sensor->filter_coeff) * sensor->filtered_value;
	}
	return sensor->filtered_value;
}

/**
 * @brief 设置校准参数
 */
void AngleSensor_SetCalibration(AngleSensor* sensor, float k, float b)
{
	sensor->k = k;
	sensor->b = b;
}

/**
 * @brief 设置滤波系数
 */
void AngleSensor_SetFilterCoeff(AngleSensor* sensor, float coeff)
{
	if (coeff < 0.0f) coeff = 0.0f;
	if (coeff > 1.0f) coeff = 1.0f;
	sensor->filter_coeff = coeff;
}
