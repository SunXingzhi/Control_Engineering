#include "angle_sensor.h"

/* ---------- 内部辅助函数 ---------- */

/**
 * @brief 计算当前未规范化的原始角度（已减 offset）
 */
float angle_sensor_get_unwrapped(AngleSensor* sensor)
{
    uint32_t raw = AngleSensor_ReadRaw(sensor);
    return (float)raw * sensor->k + sensor->b - sensor->offset;
}

/**
 * @brief 更新低通滤波器，返回未规范化的滤波角度
 */
static float angle_sensor_update_filter(AngleSensor* sensor)
{
    float raw_angle = angle_sensor_get_unwrapped(sensor);

    if (sensor->filter_initialized == 0) {
        sensor->filtered_value = raw_angle;
        sensor->filter_initialized = 1;
    } else {
        sensor->filtered_value = sensor->filter_coeff * raw_angle +
                                (1.0f - sensor->filter_coeff) * sensor->filtered_value;
    }
    return sensor->filtered_value;
}

/**
 * @brief 将任意角度规范化到 [0, range)
 * @note  用 while 循环替代 fmodf，避免链接软浮点数学库
 *        倒立摆角度变化范围有限（通常 < 3 圈），循环次数极少
 */
static float normalize_angle(float angle, float range)
{
    while (angle >= range) angle -= range;
    while (angle < 0.0f)  angle += range;
    return angle;
}

/* ---------- 公有函数实现 ---------- */

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
    sensor->offset     = 0.0f;

    sensor->filter_coeff   = (filter_coeff < 0.0f) ? 0.0f :
                             (filter_coeff > 1.0f) ? 1.0f : filter_coeff;
    sensor->filtered_value = 0.0f;
    sensor->filter_initialized = 0;

    sensor->prev_angle    = 0.0f;
    sensor->prev_time_ms  = 0;
    sensor->vel_initialized = 0;

    sensor->vel_filter_coeff       = 0.15f;
    sensor->filtered_velocity      = 0.0f;
    sensor->vel_filter_initialized = 0;

    sensor->range = 360.0f;
}

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
 * @brief 获取瞬时角度（已扣除偏移，且规范化到 0~360°）
 */
float AngleSensor_GetAngle(AngleSensor* sensor)
{
    float unwrapped = angle_sensor_get_unwrapped(sensor);
    return normalize_angle(unwrapped, sensor->range);
}

/**
 * @brief 获取滤波后的角度（已扣除偏移，且规范化到 0~360°）
 * @note  内部滤波值保持未规范化，保证微分连续性
 */
float AngleSensor_GetFilteredAngle(AngleSensor* sensor)
{
    float unwrapped = angle_sensor_update_filter(sensor);
    return normalize_angle(unwrapped, sensor->range);
}

float AngleSensor_GetFilteredAngleUnwrapped(AngleSensor* sensor)
{
    return angle_sensor_update_filter(sensor);
}

/**
 * @brief 获取平滑后的角速度（内部使用未规范化角度，避免环绕跳变）
 */
float AngleSensor_GetAngularVelocity(AngleSensor* sensor)
{
    /* 获取未规范化的最新滤波角度用于微分 */
    float cur_angle = angle_sensor_update_filter(sensor);
    uint32_t cur_time = HAL_GetTick();

    float instant_velocity = 0.0f;

    if (sensor->vel_initialized) {
        uint32_t dt_ms = cur_time - sensor->prev_time_ms;
        if (dt_ms > 0) {
            float d_angle = cur_angle - sensor->prev_angle;

            /* 环绕修正：自动处理跨越量程边界的角度跳变 */
            float half_range = sensor->range * 0.5f;
            if (d_angle > half_range) {
                d_angle -= sensor->range;
            } else if (d_angle < -half_range) {
                d_angle += sensor->range;
            }

            instant_velocity = d_angle * 1000.0f / (float)dt_ms;
        }
    } else {
        sensor->vel_initialized = 1;
    }

    /* 保存本次状态供下次微分使用 */
    sensor->prev_angle   = cur_angle;
    sensor->prev_time_ms = cur_time;

    /* 瞬时速度一阶低通滤波 */
    if (sensor->vel_filter_initialized == 0) {
        sensor->filtered_velocity = instant_velocity;
        sensor->vel_filter_initialized = 1;
    } else {
        sensor->filtered_velocity = sensor->vel_filter_coeff * instant_velocity +
                                   (1.0f - sensor->vel_filter_coeff) * sensor->filtered_velocity;
    }

    return sensor->filtered_velocity;
}

void AngleSensor_SetCalibration(AngleSensor* sensor, float k, float b)
{
    sensor->k = k;
    sensor->b = b;
}

void AngleSensor_SetFilterCoeff(AngleSensor* sensor, float coeff)
{
    if (coeff < 0.0f) coeff = 0.0f;
    if (coeff > 1.0f) coeff = 1.0f;
    sensor->filter_coeff = coeff;
}

void AngleSensor_SetVelocityFilterCoeff(AngleSensor* sensor, float coeff)
{
    if (coeff < 0.0f) coeff = 0.0f;
    if (coeff > 1.0f) coeff = 1.0f;
    sensor->vel_filter_coeff = coeff;
    sensor->vel_filter_initialized = 0;  // 重置以立即应用新系数
}

void AngleSensor_SetCurrentAngle(AngleSensor* sensor, float target_angle)
{
    uint32_t raw = AngleSensor_ReadRaw(sensor);
    float current_unwrapped = (float)raw * sensor->k + sensor->b - sensor->offset;
    float offset = current_unwrapped - target_angle;
    sensor->offset += offset;  // 累加偏移，使 current_unwrapped' = target_angle

    /* 重置滤波器到新的未规范化值（即 target_angle） */
    sensor->filtered_value = target_angle;
    sensor->filter_initialized = 1;

    /* 重置速度状态 */
    sensor->vel_initialized        = 0;
    sensor->vel_filter_initialized = 0;
    sensor->prev_angle             = 0.0f;
    sensor->prev_time_ms           = 0;
    sensor->filtered_velocity      = 0.0f;
}

void AngleSensor_SetOffset(AngleSensor* sensor, float offset)
{
    sensor->offset = offset;

    /* 同步滤波值到新的未规范化角度 */
    float corrected_unwrapped = angle_sensor_get_unwrapped(sensor);
    sensor->filtered_value = corrected_unwrapped;
    sensor->filter_initialized = 1;

    /* 重置速度状态 */
    sensor->vel_initialized        = 0;
    sensor->vel_filter_initialized = 0;
    sensor->prev_angle             = 0.0f;
    sensor->prev_time_ms           = 0;
    sensor->filtered_velocity      = 0.0f;
}

void AngleSensor_SetRange(AngleSensor* sensor, float range)
{
    if (range > 0.0f) {
        sensor->range = range;
    }
}
