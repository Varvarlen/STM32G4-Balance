#ifndef SPEED_CAPTURE_H
#define SPEED_CAPTURE_H

#include <stdint.h>

// 1kHz 阶跃响应采集: 256 帧 × 4 字段 = 4KB 静态 BSS
#define SC_BURST_SAMPLES  256
#define SC_BURST_FIELDS   4     // speed_fb, iq, speed_ref, T_load_est

// 二进制 dump: MAGIC(4B) + COUNT(2B) + FIELDS(1B) + RESV(1B) + DATA
#define SC_MAGIC  0x53554252u   // "RBUS" in byte stream (LE: R=52 B=42 U=55 S=53)

typedef struct {
    float *buf;                 // 指向静态 BSS 缓冲区
    volatile uint16_t wr;       // 写入计数 (ISR/任务写入)
    uint16_t total;             // 总采样数
    volatile uint8_t active;    // 采集中
    volatile uint8_t ready;     // 数据就绪, 待 dump
} SpeedCapture_t;

void SpeedCapture_Init(void);
void SpeedCapture_Start(uint16_t samples);
void SpeedCapture_Write(float fb, float iq, float ref, float tload);
void SpeedCapture_Dump(void);       // 二进制 dump 到串口
uint8_t SpeedCapture_IsBusy(void);  // 采集或 dump 中
uint8_t SpeedCapture_IsReady(void); // 采集完成, 数据就绪

#endif
