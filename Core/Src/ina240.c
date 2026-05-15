/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ina240.c
  * @brief   INA240A2 电流采样芯片驱动
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ina240.h"
#include "adc.h"

/* USER CODE BEGIN 0 */

extern DMA_HandleTypeDef hdma_adc2;

// ADC2 DMA 环形缓冲区（DMA 自动更新）
volatile uint16_t adc_buffer[INA240_NUM_CHANNELS];

// 零偏校准值（无电流时各通道的 ADC 原始读数）
static uint16_t zero_offset[INA240_NUM_CHANNELS];

// EMA 滤波后输出（在 HAL_ADC_ConvCpltCallback 中更新）
// filtered += (new - filtered) >> EMA_SHIFT，等效 N≈(2^(shift+1)-1) 过采样
#define INA240_EMA_SHIFT  3   // k=8, 时间常数 ≈0.8ms @10kHz
static uint16_t filtered_buffer[INA240_NUM_CHANNELS];

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  将 ADC 原始值转换为电流（安培）
  * @param  adc_value: ADC 原始采样值（12-bit, 0-4095）
  * @retval 电流值（A），正值为正方向，反之为负
  * @note   Vout = I * Rshunt * Gain + Vref
  *         I = (Vout - Vref) / (Rshunt * Gain)
  *         Vout = ADC_value / 4095 * ADC_REF
  */
static float adc_to_current(uint16_t adc_value, uint16_t offset)
{
    int16_t diff = (int16_t)(adc_value - offset);
    float voltage = (float)diff * INA240_ADC_REF / 4095.0f;
    return voltage / (INA240_SHUNT_RES * INA240_GAIN);
}

/**
  * @brief  快速读取电流（ISR 安全，读 EMA 滤波后的值）
  * @param  channel: 电流通道
  * @retval 电流值（安培），无效通道返回 0
  */
float INA240_GetCurrentFast(INA240_Channel_t channel)
{
    if (channel >= INA240_NUM_CHANNELS)
        return 0.0f;

    return adc_to_current(filtered_buffer[channel], zero_offset[channel]);
}

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/**
  * @brief  初始化电流采样
  * @note   启动 ADC2 DMA 连续转换，4 个通道循环采样
  * @retval None
  */
void INA240_Init(void)
{
    // 初始化为理论中点值 2048（12-bit ADC，1.65V 偏置对应）
    for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
    {
        zero_offset[i] = 2048;
        filtered_buffer[i] = 2048;
    }

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, INA240_NUM_CHANNELS);
    __HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT);  // HAL_Start_DMA 会重启用 HT, 关掉
}

/**
  * @brief  校准零偏：取 256 次 ADC 扫描平均值作为零点
  * @note   必须在电机无电流时调用（系统启动初期）
  * @retval None
  */
void INA240_Calibrate(void)
{
    uint32_t sum[INA240_NUM_CHANNELS] = {0};
    const uint32_t samples = 256;

    for (uint32_t n = 0; n < samples; n++)
    {
        for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
        {
            sum[i] += adc_buffer[i];
        }
        // ADC 由 TIM3 TRGO 以 10kHz 触发，1ms 等待确保多次更新
        HAL_Delay(1);
    }

    for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
    {
        zero_offset[i] = (uint16_t)(sum[i] / samples);
        // 同步 EMA 滤波器初始值到校准结果，消除启动瞬态
        filtered_buffer[i] = zero_offset[i];
    }
}

/**
  * @brief  获取指定通道的电流值（非 ISR 路径，同样读滤波值）
  * @param  channel: 电流通道
  * @retval 电流值（安培），无效通道返回 0
  */
float INA240_GetCurrent(INA240_Channel_t channel)
{
    if (channel >= INA240_NUM_CHANNELS)
        return 0.0f;

    return adc_to_current(filtered_buffer[channel], zero_offset[channel]);
}

/**
  * @brief  获取所有通道的电流值
  * @param  currents: 输出数组，长度至少为 INA240_NUM_CHANNELS
  * @retval None
  */
void INA240_GetAllCurrents(float *currents)
{
    for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
    {
        currents[i] = adc_to_current(filtered_buffer[i], zero_offset[i]);
    }
}

void INA240_SetZeroOffset(INA240_Channel_t channel, uint16_t offset)
{
    if (channel < INA240_NUM_CHANNELS) {
        zero_offset[channel] = offset;
        filtered_buffer[channel] = offset;
    }
}

uint16_t INA240_GetZeroOffset(INA240_Channel_t channel)
{
    return (channel < INA240_NUM_CHANNELS) ? zero_offset[channel] : 0;
}

void INA240_SetAllZeroOffsets(const uint16_t offsets[INA240_NUM_CHANNELS])
{
    for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++) {
        zero_offset[i] = offsets[i];
        filtered_buffer[i] = offsets[i];
    }
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/**
  * @brief  ADC 转换完成回调（DMA 传输完成触发，10kHz）
  * @param  hadc: ADC 句柄
  * @retval None
  * @note   对每通道执行 EMA 低通滤波，等效 ~31 倍过采样
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc2)
    {
        // EMA 滤波：filtered += (adc - filtered) >> INA240_EMA_SHIFT
        for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
        {
            int32_t diff = (int32_t)adc_buffer[i] - (int32_t)filtered_buffer[i];
            filtered_buffer[i] = (uint16_t)((int32_t)filtered_buffer[i] + (diff >> INA240_EMA_SHIFT));
        }
    }
}

/* USER CODE END 3 */
