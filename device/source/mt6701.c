/**
 * @file    mt6701.c
 * @brief   MT6701 磁编码器驱动实现（SPI + DMA）
 */

#include "../include/mt6701.h"

/* ---- 全局设备指针（HAL 回调签名固定，无法传参）---- */
mt6701_t* g_dev = NULL;

/* ---- 内部辅助函数 ---- */

float cycle_diff(float diff, float cycle)
{
	if (diff > (cycle / 2.0f))
		diff -= cycle;
	else if (diff < -(cycle / 2.0f))
		diff += cycle;
	return diff;
}

uint8_t calculate_crc(uint32_t data)
{
	uint8_t crc = 0;
	const uint32_t poly = 0x43; /* X^6 + X + 1 */

	for (int i = 17; i >= 0; i--){
		uint8_t bit = (data >> i) & 1;
		crc <<= 1;
		if (((crc >> 6) ^ bit) & 1)
			crc ^= poly;
		crc &= 0x3F;
	}
	return crc;
}

/* ---- API 实现 ---- */

device_err_t angle_sensor_init(mt6701_t* dev)
{
	if (dev == NULL) return DRV_ERR_NULL;
	if (dev->sensor.hspi == NULL) return DRV_ERR_NULL;
	if (dev->sensor.cs_gpiox == NULL) return DRV_ERR_NULL;
	if (dev->sensor.htim == NULL) return DRV_ERR_NULL;

	g_dev = dev;

	dev->sensor.angle_raw = 0;
	dev->sensor.speed_raw = 0;
	dev->sensor.angle = 0.0f;
	dev->sensor.speed = 0.0f;
	dev->sensor.angle_last = 0.0f;
	dev->sensor.angle_prev = 0.0f;
	dev->sensor.angle_total = 0.0f;
	dev->sensor.first_sample = 1;

	/* TX 填充 dummy 数据 */
	dev->tx_buf[0] = 0xFF;
	dev->tx_buf[1] = 0xFF;
	dev->tx_buf[2] = 0xFF;

	/* 拉低 CS，发起第一次 SPI DMA 读取 */
	HAL_GPIO_WritePin(dev->sensor.cs_gpiox, dev->sensor.cs_gpio_pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(dev->sensor.hspi, dev->tx_buf, dev->rx_buf, 3) != HAL_OK)
		return DRV_ERR_INIT;

	HAL_Delay(10); /* 等待第一次角度数据就绪 */

	/* 启动速度计算定时器 */
	if (HAL_TIM_Base_Start_IT(dev->sensor.htim) != HAL_OK)
		return DRV_ERR_INIT;

	HAL_Delay(10);

	return DRV_OK;
}

device_err_t angle_sensor_read_angle(mt6701_t* dev, float* angle)
{
	if (dev == NULL || angle == NULL) return DRV_ERR_NULL;
	*angle = dev->sensor.angle;
	return DRV_OK;
}

device_err_t angle_sensor_read_speed(mt6701_t* dev, float* speed)
{
	if (dev == NULL || speed == NULL) return DRV_ERR_NULL;
	*speed = dev->sensor.speed;
	return DRV_OK;
}

device_err_t angle_sensor_read_total_angle(mt6701_t* dev, float* angle)
{
	if (dev == NULL || angle == NULL) return DRV_ERR_NULL;
	*angle = dev->sensor.angle_total;
	return DRV_OK;
}

/* ---- HAL 回调（从中断上下文调用）---- */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
	if (g_dev == NULL) return;
	if (hspi->Instance != g_dev->sensor.hspi->Instance) return;

	mt6701_t* dev = g_dev;
	angle_sensor_t* s = &dev->sensor;

	HAL_GPIO_WritePin(s->cs_gpiox, s->cs_gpio_pin, GPIO_PIN_SET);

	/* 提取 14 位角度原始值 */
	int angle_raw = (dev->rx_buf[1] >> 2) | (dev->rx_buf[0] << 6);

	/* CRC 校验 */
	uint8_t crc_raw = dev->rx_buf[2] & 0x3F;
	uint32_t crc_data = ((uint32_t)dev->rx_buf[0] << 16 |
		(uint32_t)dev->rx_buf[1] << 8 |
		(uint32_t)dev->rx_buf[2]) >> 6;
	if (calculate_crc(crc_data) != crc_raw){
		/* CRC 失败：丢弃本次数据，重新发起读取 */
		HAL_GPIO_WritePin(s->cs_gpiox, s->cs_gpio_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_DMA(s->hspi, dev->tx_buf, dev->rx_buf, 3);
		return;
	}

	s->angle_raw = (uint32_t)angle_raw;
	s->angle = MT6701_ANGLE_MAX * angle_raw / MT6701_RAW_MAX;

	/* 累计角度：检测过零点并累加/递减 */
	float diff = s->angle - s->angle_prev;
	if (diff > 3.1415926f) /* 从 ~2π 跳到 ~0，说明反转过零 */
		diff -= MT6701_ANGLE_MAX;
	else if (diff < -3.1415926f) /* 从 ~0 跳到 ~2π，说明正转过零 */
		diff += MT6701_ANGLE_MAX;
	s->angle_total += diff;
	s->angle_prev = s->angle; /* 移到这里，供下次过零检测用 */

	/* 再次发起读取 */
	HAL_GPIO_WritePin(s->cs_gpiox, s->cs_gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(s->hspi, dev->tx_buf, dev->rx_buf, 3);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
	if (g_dev == NULL) return;
	if (hspi->Instance != g_dev->sensor.hspi->Instance) return;

	mt6701_t* dev = g_dev;
	angle_sensor_t* s = &dev->sensor;

	/* CS 拉高再拉低，重新发起读取 */
	HAL_GPIO_WritePin(s->cs_gpiox, s->cs_gpio_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(s->cs_gpiox, s->cs_gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(s->hspi, dev->tx_buf, dev->rx_buf, 3);
}


/* ======================== TIM3: 编码器采样 ======================== */
/**
 * @brief  编码器速度更新（TIM3 中断，1ms 周期）
 * @note   一阶低通滤波消除编码器噪声
 *         alpha=0.15 → 截止频率约 24Hz
 */
void encoder_update_speed(void)
{
	if (g_dev == NULL) return;

	angle_sensor_t* s = &g_dev->sensor;

	if (s->first_sample){
		s->first_sample = 0;
		s->angle_last = s->angle;
		s->speed = 0.0f;
	}

	// 每隔 SPEED_CALC_DIV 次 (5ms) 计算一次速度，增大角度差以减少量化噪声

	static uint8_t calc_div = 0;
	if (++calc_div < SPEED_CALC_DIV_TICKS) return;
	calc_div = 0;

	float diff = cycle_diff(s->angle - s->angle_last, MT6701_ANGLE_MAX);
	s->angle_last = s->angle;

	// 5ms 累积的角速度 → 单个采样噪声降低为 1/5
	float omega_raw = diff * speed_calc_freq / SPEED_CALC_DIV_TICKS;  // rad/s
	float speed_raw = omega_to_rpm(omega_raw);                  // rpm

	// 一阶低通: y = alpha*x + (1-alpha)*y_prev
	// alpha=0.3, 5ms 周期 → 截止频率约 11Hz
	#define SPEED_FILTER_ALPHA  0.3f
	s->speed = SPEED_FILTER_ALPHA * speed_raw
	         + (1.0f - SPEED_FILTER_ALPHA) * s->speed;
	s->speed_raw = (int32_t)(omega_raw * 100.0f);
}