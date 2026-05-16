#ifndef ENCODER_CACHE_H
#define ENCODER_CACHE_H

#include <stdint.h>
#include "mt6701.h"

// 编码器角度缓存（SPI DMA ISR 写入，FOC ISR / FreeRTOS 任务读取）
typedef struct {
    volatile uint16_t raw_angle;       // 14-bit 原始角度（ISR 写入，需 volatile）
    volatile float    mech_angle;      // 机械角度 (rad)
    volatile float    elec_angle;      // 电角度 (rad) = mech * 7
    uint8_t  status;                  // MT6701 状态字
    volatile uint8_t fresh;           // 新数据标志
} EncoderCache_t;

extern EncoderCache_t g_enc[MT6701_NUM_ENCODERS];

// FOC 同步快照 (ADC ISR 写入, SpeedLoop Task 读取)
// 同一 20kHz 周期内捕获 mech_angle 和 iq, 消除不同 ISR 源的时间偏差
typedef struct {
    volatile float mech_angle;  // 编码器机械角度 (rad)
    volatile float iq;          // q 轴电流反馈 (A)
} FOC_Snapshot_t;

extern volatile FOC_Snapshot_t g_foc_snap[MT6701_NUM_ENCODERS];

#endif
