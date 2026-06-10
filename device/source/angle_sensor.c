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
    sensor->hadc       = hadc;
    sensor->Channel    = Channel;
    sensor->SampleTime = SampleTime;
    sensor->k          = k;
    sensor->b          = b;
    sensor->offset     = 0.0f;   /* 初始偏移量为0 */

    /* 滤波初始化 */
    sensor->filter_coeff       = (filter_coeff < 0.0f) ? 0.0f :
                                 (filter_coeff > 1.0f) ? 1.0f : filter_coeff;
    sensor->filtered_value     = 0.0f;
    sensor->filter_initialized = 0;
}

/**
 * @brief 读取ADC原始值
 */
uint32_t AngleSensor_ReadRaw(AngleSensor* sensor)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = sensor->Channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sensor->SampleTime;

    if (HAL_ADC_ConfigChannel(sensor->hadc, &sConfig) != HAL_OK) {
        return 0;
    }

    HAL_ADC_Start(sensor->hadc);
    if (HAL_ADC_PollForConversion(sensor->hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(sensor->hadc);
        return 0;
    }

    uint32_t raw = HAL_ADC_GetValue(sensor->hadc);
    HAL_ADC_Stop(sensor->hadc);
    return raw;
}

/**
 * @brief 获取瞬时角度值（已扣除偏移量）
 */
float AngleSensor_GetAngle(AngleSensor* sensor)
{
    uint32_t raw = AngleSensor_ReadRaw(sensor);
    float raw_angle = (float)raw * sensor->k + sensor->b;
    return raw_angle - sensor->offset;
}

/**
 * @brief 获取经过一阶低通滤波后的角度值（已扣除偏移量）
 * @note  公式：Y(n) = α * X(n) + (1-α) * Y(n-1)
 */
float AngleSensor_GetFilteredAngle(AngleSensor* sensor)
{
    float raw_angle = AngleSensor_GetAngle(sensor);  /* 已经减去了offset */

    if (sensor->filter_initialized == 0) {
        /* 第一次直接用瞬时值初始化 */
        sensor->filtered_value = raw_angle;
        sensor->filter_initialized = 1;
    } else {
        /* 指数移动平均滤波 */
        sensor->filtered_value = sensor->filter_coeff * raw_angle +
                                (1.0f - sensor->filter_coeff) * sensor->filtered_value;
    }
    return sensor->filtered_value;
}

/**
 * @brief 设置线性校准系数（k、b）
 */
void AngleSensor_SetCalibration(AngleSensor* sensor, float k, float b)
{
    sensor->k = k;
    sensor->b = b;
    /* 注意：改变k/b会使当前offset对应的实际偏移角度发生变化，
       通常需要重新进行偏移校准，此处不做自动处理 */
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

/**
 * @brief 将当前物理角度设置为指定的目标角度值（自动计算offset）
 * @note  通过读取当前ADC原始值，计算出使输出变为target_angle所需的偏移量。
 *        同时重置滤波状态，使滤波值立即等于target_angle。
 */
void AngleSensor_SetCurrentAngle(AngleSensor* sensor, float target_angle)
{
    /* 读取当前原始角度（不受offset影响） */
    uint32_t raw = AngleSensor_ReadRaw(sensor);
    float current_raw_angle = (float)raw * sensor->k + sensor->b;

    /* 计算偏移量：offset = 当前原始角度 - 目标角度 */
    sensor->offset = current_raw_angle - target_angle;

    /* 重置滤波器，使滤波输出直接变为目标角度，避免缓慢过渡 */
    sensor->filtered_value = target_angle;
    sensor->filter_initialized = 1;   /* 标记已初始化，后续正常滤波 */
}

/**
 * @brief 直接设置偏移量
 * @note  提供直接修改offset的接口，并自动更新滤波状态，避免过渡延迟。
 */
void AngleSensor_SetOffset(AngleSensor* sensor, float offset)
{
    sensor->offset = offset;

    /* 重新计算当前已偏移后的角度，并更新滤波值 */
    float corrected_angle = AngleSensor_GetAngle(sensor); /* 读取一次瞬时值（已减新offset） */
    sensor->filtered_value = corrected_angle;
    sensor->filter_initialized = 1;
}