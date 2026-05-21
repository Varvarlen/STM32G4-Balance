/*
 * buzzer.h
 *
 *  Created on: 2026-04-01
 *      Author: User
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "stm32g4xx_hal.h"

// 蜂鸣器初始化
void Buzzer_Init(void);

// 蜂鸣器发声
//  RTOS 启动前: 阻塞 (HAL_Delay), 调用后立即返回
//  RTOS 启动后: 非阻塞, 由 Buzzer_Update() 在到期时调用 Buzzer_Stop()
//  freq: 频率(Hz)
//  duration: 持续时间(ms)
void Buzzer_Beep(uint16_t freq, uint16_t duration);

// 蜂鸣器停止
void Buzzer_Stop(void);

// 变频滑音: duration_ms 内频率从 freq_start 连续滑到 freq_end (Hz)
void Buzzer_Sweep(uint16_t freq_start, uint16_t freq_end, uint16_t duration_ms);

// 每 1ms 调用一次 (平衡任务中), 检查蜂鸣是否到期
void Buzzer_Update(void);

// 播放音调
// note: 音符(0-8对应C4-B4等)
// duration: 持续时间(ms)
void Buzzer_PlayNote(uint8_t note, uint16_t duration);

#endif /* BUZZER_H */
