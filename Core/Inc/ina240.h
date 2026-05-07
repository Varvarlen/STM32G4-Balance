/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ina240.h
  * @brief   INA240A2 电流采样芯片驱动
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __INA240_H__
#define __INA240_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/** @brief 电流通道枚举 (顺序必须匹配 ADC 扫描: rank1=IN13, rank2=IN3, rank3=IN5, rank4=IN12)
 *  @note  枚举名使用 INA240 芯片引脚标签 (U/W)，实际连接的电机相约见各枚举值的注释
 *         INA240_MOTOR1_U 实际采样 M1 V/B 相，INA240_MOTOR1_W 实际采样 M1 U/A 相 */
typedef enum {
    INA240_MOTOR1_U = 0,    /**< M1 V/B相 — PA5 — ADC2_IN13 — rank1 */
    INA240_MOTOR1_W = 1,    /**< M1 U/A相 — PA6 — ADC2_IN3  — rank2 */
    INA240_MOTOR2_U = 2,    /**< M2 V/B相 — PC4 — ADC2_IN5  — rank3 */
    INA240_MOTOR2_W = 3,    /**< M2 W/C相 — PB2 — ADC2_IN12 — rank4 */
    INA240_NUM_CHANNELS
} INA240_Channel_t;

/** @brief INA240A2 硬件参数 */
#define INA240_GAIN         50.0f   /**< 放大倍率 50V/V */
#define INA240_SHUNT_RES    0.020f  /**< 采样电阻 20mΩ */
#define INA240_VREF         1.65f   /**< 偏置电压 Vs/2 = 1.65V */
#define INA240_ADC_REF      3.3f    /**< ADC 参考电压 */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/** @brief 初始化电流采样（启动 ADC DMA 连续转换） */
void INA240_Init(void);

/** @brief 校准零偏：无电流时调用，记录当前读数为零点 */
void INA240_Calibrate(void);

/** @brief 获取指定通道电流值（安培） */
float INA240_GetCurrent(INA240_Channel_t channel);

/** @brief 获取所有通道电流值 */
void INA240_GetAllCurrents(float *currents);

// ISR 安全：直接从 ADC DMA 缓冲区读取原始值并转换为电流
float INA240_GetCurrentFast(INA240_Channel_t channel);

// ADC 原始缓冲区（DMA 自动更新，ISR 可直接索引读取）
extern volatile uint16_t adc_buffer[INA240_NUM_CHANNELS];

// 零偏设置/读取（校准用）
void     INA240_SetZeroOffset(INA240_Channel_t channel, uint16_t offset);
uint16_t INA240_GetZeroOffset(INA240_Channel_t channel);
void     INA240_SetAllZeroOffsets(const uint16_t offsets[INA240_NUM_CHANNELS]);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __INA240_H__ */
