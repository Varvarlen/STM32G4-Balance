/*
 * ws2812b.h
 *
 *  Created on: 2026-04-01
 *      Author: User
 */

#ifndef WS2812B_H
#define WS2812B_H

#include "stm32g4xx_hal.h"

// 灯珠数量
#define WS2812B_LED_COUNT 4

// RGB颜色结构体
typedef struct {
    uint8_t r;  // 红色分量
    uint8_t g;  // 绿色分量
    uint8_t b;  // 蓝色分量
} WS2812B_Color_t;

// 初始化WS2812B驱动
void WS2812B_Init(void);

// 设置单个灯珠颜色
void WS2812B_SetLED(uint8_t index, WS2812B_Color_t color);

// 设置所有灯珠颜色
void WS2812B_SetAll(WS2812B_Color_t color);

// 更新灯珠显示（非阻塞）
void WS2812B_Update(void);

// 基本灯效接口

// 呼吸效果
void WS2812B_Breathe(WS2812B_Color_t color, uint16_t period);

// 流水灯效果
void WS2812B_RunningLight(WS2812B_Color_t color, uint16_t speed);

// 闪烁效果
void WS2812B_Blink(WS2812B_Color_t color, uint16_t period);

// 彩虹效果
void WS2812B_Rainbow(uint16_t speed);

#endif /* WS2812B_H */