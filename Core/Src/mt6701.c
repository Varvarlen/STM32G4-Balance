/*
 * mt6701.c
 *
 *  MT6701 14-bit 磁编码器 SPI (SSI) 驱动
 *
 *  SSI 通信时序：
 *    1. CS 拉低
 *    2. 发送 3 字节 0x00（同时接收 3 字节响应，24 个时钟周期）
 *    3. CS 拉高
 *
 *  24-bit SSI 帧格式：
 *    Byte0[7:0] = Angle[13:6]  (高位)
 *    Byte1[7:2] = Angle[5:0]   (低位)
 *    Byte1[1:0] = 保留
 *    Byte2[7:4] = Status SF[3:0]
 *      0x0 = 正常
 *      0x1 = 磁场过弱
 *      0x2 = 磁场过强
 *    Byte2[3:0] = 保留 / 按键状态
 *
 *  角度计算公式: angle = ((uint16_t)data[0] << 6) | (data[1] >> 2)
 */

#include "mt6701.h"
#include "main.h"   // 使用 CubeMX 生成的 CS 引脚定义
#include "spi.h"    // 使用 hspi3

// CS 引脚映射表 (index -> {GPIO_Port, GPIO_Pin})
static const GPIO_TypeDef *cs_port[MT6701_NUM_ENCODERS] = {
    mt6701_0_GPIO_Port,    // index 0: PB4
    mt6701_1_GPIO_Port     // index 1: PA4
};

static const uint16_t cs_pin[MT6701_NUM_ENCODERS] = {
    mt6701_0_Pin,   // index 0: PB4
    mt6701_1_Pin    // index 1: PA4
};

// ------------------------------------------------------------------
//  初始化：确保两个 CS 引脚初始为高电平（片选无效状态）
// ------------------------------------------------------------------
void MT6701_Init(void)
{
    for (uint8_t i = 0; i < MT6701_NUM_ENCODERS; i++)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[i], cs_pin[i], GPIO_PIN_SET);
    }
}

// ------------------------------------------------------------------
//  微秒级延时（不依赖定时器，用于 CS 时序控制）
// ------------------------------------------------------------------
static void MT6701_DelayUs(volatile uint32_t us)
{
    // 170MHz 主频下约 170 循环 ≈ 1μs
    for (; us > 0; us--)
        for (volatile uint32_t i = 0; i < 170; i++);
}

// ------------------------------------------------------------------
//  读取指定编码器的原始 24-bit SSI 帧数据
// ------------------------------------------------------------------
uint8_t MT6701_ReadRaw(uint8_t index, uint32_t *raw)
{
    uint8_t tx_data[3] = {0x00, 0x00, 0x00};
    uint8_t rx_data[3] = {0x00, 0x00, 0x00};
    HAL_StatusTypeDef ret;

    if (index >= MT6701_NUM_ENCODERS || raw == NULL)
        return 1;

    // CS 拉低，等待编码器就绪
    HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[index], cs_pin[index], GPIO_PIN_RESET);
    MT6701_DelayUs(5);

    // SPI 收发 3 字节（24 个时钟周期，读取完整 SSI 帧）
    ret = HAL_SPI_TransmitReceive(&hspi3, tx_data, rx_data, 3, HAL_MAX_DELAY);

    // 等待 SPI 完全停止后再拉高 CS
    MT6701_DelayUs(5);
    HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[index], cs_pin[index], GPIO_PIN_SET);

    // CS 高电平保持时间，确保编码器复位内部状态机
    MT6701_DelayUs(10);

    if (ret != HAL_OK)
        return ret;

    *raw = ((uint32_t)rx_data[0] << 16) | ((uint32_t)rx_data[1] << 8) | rx_data[2];
    return 0;
}

// ------------------------------------------------------------------
//  读取 14-bit 角度值 (0 ~ 16383)
// ------------------------------------------------------------------
uint16_t MT6701_ReadAngle(uint8_t index)
{
    uint32_t raw;

    if (MT6701_ReadRaw(index, &raw) != 0)
        return 0xFFFF;

    // Byte0[7:0] = Angle[13:6], Byte1[7:2] = Angle[5:0]
    return ((uint16_t)(raw >> 10)) & 0x3FFF;
}

// ------------------------------------------------------------------
//  读取编码器状态 SF[3:0]
//    0x0 = 正常
//    0x1 = 磁场过弱
//    0x2 = 磁场过强
//    其它 = 保留
// ------------------------------------------------------------------
uint8_t MT6701_ReadStatus(uint8_t index)
{
    uint32_t raw;

    if (MT6701_ReadRaw(index, &raw) != 0)
        return 0xFF;

    // Byte2[7:4] = Status SF[3:0]
    return (uint8_t)((raw >> 4) & 0x0F);
}

// ------------------------------------------------------------------
//  一次性读取角度 + 状态
// ------------------------------------------------------------------
uint8_t MT6701_GetData(uint8_t index, MT6701_Data_t *data)
{
    uint32_t raw;
    uint8_t ret;

    if (data == NULL)
        return 1;

    ret = MT6701_ReadRaw(index, &raw);
    if (ret != 0)
    {
        data->error = 1;
        return ret;
    }

    data->angle  = ((uint16_t)(raw >> 10)) & 0x3FFF;
    data->status = (uint8_t)((raw >> 4) & 0x0F);
    data->error  = 0;

    return 0;
}

// ===== DMA 乒乓读取 =====
#include "foc.h"
#include "encoder_cache.h"

EncoderCache_t g_enc[MT6701_NUM_ENCODERS] = {0};

// 3 字节 DMA 传输缓冲（TX 始终为 0，RX 由 DMA 填充）
static uint8_t dma_tx[MT6701_NUM_ENCODERS][3];
static uint8_t dma_rx[MT6701_NUM_ENCODERS][3];
// 当前正在 DMA 的编码器索引
static volatile uint8_t dma_current_index = 0;

// 启动指定编码器的 SPI DMA 读取
void MT6701_StartDMA(uint8_t index)
{
    dma_current_index = index;
    HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[index], cs_pin[index], GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi3, dma_tx[index], dma_rx[index], 3);
}

// DMA 完成后由 HAL_SPI_TxRxCpltCallback 调用
void MT6701_OnDMAComplete(uint8_t index)
{
    // CS 拉高，结束本次 SSI 帧读取
    HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[index], cs_pin[index], GPIO_PIN_SET);

    // 解析 24-bit SSI 帧
    uint32_t raw = ((uint32_t)dma_rx[index][0] << 16)
                 | ((uint32_t)dma_rx[index][1] << 8)
                 |  dma_rx[index][2];

    // Byte0[7:0] = Angle[13:6], Byte1[7:2] = Angle[5:0] → 右移 10 位 = 低 14 位
    uint16_t raw_angle = ((uint16_t)(raw >> 10)) & 0x3FFF;
    uint8_t  status    = (uint8_t)((raw >> 4) & 0x0F);

    // 更新缓存（32-bit float 原子写入，ISR 安全）
    g_enc[index].raw_angle  = raw_angle;
    g_enc[index].mech_angle = (float)raw_angle * 6.283185307f / 16384.0f;
    g_enc[index].elec_angle = g_enc[index].mech_angle * (float)MOTOR_POLE_PAIRS;
    g_enc[index].status     = status;
    g_enc[index].fresh      = 1;

    // 启动另一个编码器（乒乓）
    MT6701_StartDMA(1 - index);
}

// SPI TX/RX 完成回调（由 HAL_SPI_IRQHandler 触发）
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        MT6701_OnDMAComplete(dma_current_index);
    }
}

// SPI 错误回调（由 HAL_SPI_IRQHandler 触发）
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        // 重启乒乓（从当前编码器重新读取）
        MT6701_StartDMA(dma_current_index);
    }
}
