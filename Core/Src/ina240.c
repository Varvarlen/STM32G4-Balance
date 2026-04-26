/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ina240.c
  * @brief   INA240A1 电流采样芯片驱动
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ina240.h"
#include "adc.h"

/* USER CODE BEGIN 0 */

// ADC2 DMA 环形缓冲区（半字 16-bit 匹配 DMA 传输宽度）
static uint16_t adc_buffer[INA240_NUM_CHANNELS];

// 零偏校准值（无电流时各通道的 ADC 原始读数）
static uint16_t zero_offset[INA240_NUM_CHANNELS];

// 累积滤波参数
#define INA240_ACCUM_TARGET    64    // 每通道累积采样数
#define INA240_ACCUM_TARGET_DIV  64U // 除法用（无浮点）

static uint32_t accum[INA240_NUM_CHANNELS];     // 累加器
static volatile uint32_t accum_count;            // 当前累积计数
static uint16_t filtered_buffer[INA240_NUM_CHANNELS]; // 滤波后输出

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  将 ADC 原始值转换为电流（安培）
  * @param  adc_value: ADC 原始采样值（12-bit, 0-4095）
  * @retval 电流值（A），正值为正方向，负值为反方向
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

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/**
  * @brief  初始化电流采样
  * @note   启动 ADC2 DMA 连续转换，4 个通道循环采样：
  *         IN13(PA5) → IN3(PA6) → IN5(PC4) → IN12(PB2)
  *         对应 Motor1_U → Motor1_W → Motor2_U → Motor2_W
  * @retval None
  */
void INA240_Init(void)
{
    // 初始化零偏为理论中点值 2048（12-bit ADC，1.65V 对应）
    for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
    {
        zero_offset[i] = 2048;
        filtered_buffer[i] = 2048;
        accum[i] = 0;
    }
    accum_count = 0;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, INA240_NUM_CHANNELS);
}

void INA240_Calibrate(void)
{
    // 取 16 次采样平均作为零偏值
    uint32_t sum[INA240_NUM_CHANNELS] = {0};
    const uint32_t samples = 16;

    for (uint32_t n = 0; n < samples; n++)
    {
        for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
        {
            sum[i] += adc_buffer[i];
        }
        HAL_Delay(1);
    }

    for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
    {
        zero_offset[i] = (uint16_t)(sum[i] / samples);
    }
}

/**
  * @brief  获取指定通道的电流值
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

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/**
  * @brief  ADC 转换完成回调（DMA 模式）
  * @param  hadc: ADC 句柄
  * @retval None
  * @note   DMA Circular 模式下，adc_buffer 持续更新最新值
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc2)
    {
        for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
        {
            accum[i] += adc_buffer[i];
        }

        if (++accum_count >= INA240_ACCUM_TARGET)
        {
            for (uint32_t i = 0; i < INA240_NUM_CHANNELS; i++)
            {
                filtered_buffer[i] = (uint16_t)(accum[i] / INA240_ACCUM_TARGET_DIV);
                accum[i] = 0;
            }
            accum_count = 0;
        }
    }
}

/* USER CODE END 3 */
