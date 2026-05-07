/*
 * mt6701.h
 *
 *  MT6701 14-bit 磁编码器 SPI (SSI) 驱动
 *
 *  硬件连接：
 *    SPI3_SCK  - PC10
 *    SPI3_MISO - PC11
 *    SPI3_MOSI - PB5
 *    编码器0 CS - PB4
 *    编码器1 CS - PA4
 *
 *  MT6701 SSI 协议（等效 SPI Mode 1, CPOL=0, CPHA=1）
 *  24-bit 帧: Byte0[7:0]=Angle[13:6]  Byte1[7:2]=Angle[5:0]  Byte2[7:4]=Status
 *  角度分辨率: 14-bit (0 ~ 16383)
 */

#ifndef MT6701_H
#define MT6701_H

#include "stm32g4xx_hal.h"

#define MT6701_RESOLUTION   16384U  // 14-bit
#define MT6701_ANGLE_MAX    16383U
#define MT6701_NUM_ENCODERS 2U

// 编码器状态 (SF[3:0])
#define MT6701_STATUS_NORMAL            0x00U  // 正常工作
#define MT6701_STATUS_MAG_WEAK          0x01U  // 磁场过弱
#define MT6701_STATUS_MAG_STRONG        0x02U  // 磁场过强

typedef struct {
    uint16_t angle;     // 14-bit 角度值 (0-16383)
    uint8_t status;     // 状态 SF[3:0]
    uint8_t error;      // 非0表示读取失败
} MT6701_Data_t;

// 初始化编码器（配置 CS 引脚初始状态）
void MT6701_Init(void);

// 读取指定编码器的原始 24-bit SSI 帧数据
//   index: 0=PB4, 1=PA4
//   raw:   输出原始 24-bit SSI 帧 (低24位有效)
//   返回:  0=成功, 非0=失败
uint8_t MT6701_ReadRaw(uint8_t index, uint32_t *raw);

// 读取指定编码器的 14-bit 角度
//   index: 0=PB4, 1=PA4
//   返回:  角度值 (0-16383)，出错返回 0xFFFF
uint16_t MT6701_ReadAngle(uint8_t index);

// 读取指定编码器的状态 SF[3:0]
//   index: 0=PB4, 1=PA4
//   返回:  MT6701_STATUS_NORMAL / _MAG_WEAK / _MAG_STRONG，出错返回 0xFF
uint8_t MT6701_ReadStatus(uint8_t index);

// 一次性读取角度 + 状态
//   index: 0=PB4, 1=PA4
//   data:  输出结构体
//   返回:  0=成功, 非0=失败
uint8_t MT6701_GetData(uint8_t index, MT6701_Data_t *data);

// ===== DMA 乒乓读取接口（ISR 中调用） =====

// 启动指定编码器的 DMA 读取（CS 拉低 → SPI DMA → 回调解析 → 启动另一个）
void MT6701_StartDMA(uint8_t index);
// DMA 完成后由 HAL_SPI_TxRxCpltCallback 调用（解析帧 + 更新缓存 + 乒乓切换）
void MT6701_OnDMAComplete(uint8_t index);
// CS 延时定时器钩子（TIM6 由 CubeMX 初始化，此函数预留，当前为空）
void MT6701_CSDelay_Init(void);
// 延时到期后由 HAL_TIM_PeriodElapsedCallback 调用
void MT6701_OnCSDelayComplete(void);

#endif /* MT6701_H */
