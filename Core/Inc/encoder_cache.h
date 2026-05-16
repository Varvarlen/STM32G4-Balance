#ifndef ENCODER_CACHE_H
#define ENCODER_CACHE_H

#include <stdint.h>
#include <stdbool.h>
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

// 中值滤波环形缓冲 — ISR 推入, 任务侧读取
#define ENC_MEDIAN_WINDOW  16  // 窗口大小 (2^n, 快速取模)

typedef struct {
    float   buf[ENC_MEDIAN_WINDOW];
    uint8_t head;    // ISR 写入位置
    uint8_t count;   // 当前缓冲采样数 (<= WINDOW)
} EncMedian_t;

extern EncMedian_t g_enc_median[MT6701_NUM_ENCODERS];

// ISR 安全推入 (非阻塞, 满时覆盖最旧)
void EncMedian_Push(EncMedian_t *m, float val);
// 任务侧读取 — 取中值后清空缓冲; 不足 4 个采样返回 false
bool EncMedian_Read(EncMedian_t *m, float *out);

#endif
