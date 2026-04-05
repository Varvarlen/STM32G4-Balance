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
// freq: 频率(Hz)
// duration: 持续时间(ms)
void Buzzer_Beep(uint16_t freq, uint16_t duration);

// 蜂鸣器停止
void Buzzer_Stop(void);

// 播放音调
// note: 音符(0-8对应C4-B4等)
// duration: 持续时间(ms)
void Buzzer_PlayNote(uint8_t note, uint16_t duration);

#endif /* BUZZER_H */
