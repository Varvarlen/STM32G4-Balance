#ifndef VBUS_H
#define VBUS_H

#include <stdint.h>

/**
  * @brief  初始化母线电压测量
  * @note   加载 Flash 中的 VBUS 校准系数 (slope/offset)
  *         若无有效校准数据, 使用分压比理论值作为默认值
  */
void VBUS_Init(void);

/**
  * @brief  读取实时母线电压 (V)
  * @note   软件触发 ADC2 注入组通道, 单次转换, 阻塞等待完成
  * @retval 母线电压 (V), 已应用校准系数
  */
float VBUS_Read(void);

/**
  * @brief  校准采样点录入
  * @param  actual_voltage  外部高精度电源的实际电压值 (V)
  * @note   调用两次 (不同电压点) 后自动计算 slope/offset 并存入 Flash
  *         例: VBUS_CalSample(6.00f);  // 电源设在 6.00V
  *              VBUS_CalSample(7.40f);  // 电源设在 7.40V → 自动计算并保存
  */
void VBUS_CalSample(float actual_voltage);

/**
  * @brief  查询校准状态
  * @retval 1=已校准 (Flash 中有有效 slope/offset), 0=使用理论默认值
  */
uint8_t VBUS_IsCalibrated(void);

/**
  * @brief  获取校准系数 (调试用)
  */
float VBUS_GetSlope(void);
float VBUS_GetOffset(void);

#endif
