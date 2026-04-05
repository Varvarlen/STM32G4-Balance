/*
 * buzzer.c
 *
 *  Created on: 2026-04-01
 *      Author: User
 */

#include "../Inc/buzzer.h"
#include "tim.h"

// 音符频率表 (C4-B4)
static const uint16_t note_freq[] = {
    262,  // C4
    294,  // D4
    330,  // E4
    349,  // F4
    392,  // G4
    440,  // A4
    494,  // B4
    523,  // C5
    587   // D5
};

// 蜂鸣器初始化
void Buzzer_Init(void)
{
    // TIM1已经在CubeMX中初始化
    // 这里只需要确保TIM1_CH1 (PA8) 配置正确
    // 初始状态下关闭蜂鸣器
    Buzzer_Stop();
}

// 蜂鸣器发声
void Buzzer_Beep(uint16_t freq, uint16_t duration)
{
    if (freq == 0) {
        Buzzer_Stop();
        return;
    }
    
    // 计算TIM1的周期和占空比
    // 假设TIM1时钟频率为170MHz
    uint32_t tim_clock = 170000000;
    uint16_t prescaler = 170-1;
    uint16_t period = 0;
    
    // 计算合适的预分频器和周期
    // 目标是使PWM频率接近指定频率
    period = (tim_clock / (freq * (prescaler + 1))) - 1;
    
    // 调整预分频器以确保周期在有效范围内
    while (period > 65535) {
        prescaler++;
        period = (tim_clock / (freq * (prescaler + 1))) - 1;
    }
    
    // 更新TIM1配置
    __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim1, period);
    
    // 设置占空比为50%
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
    
    // 启动PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    // 延时指定时间
    if (duration > 0) {
        HAL_Delay(duration);
        Buzzer_Stop();
    }
}

// 蜂鸣器停止
void Buzzer_Stop(void)
{
    // 停止PWM输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

// 播放音调
void Buzzer_PlayNote(uint8_t note, uint16_t duration)
{
    if (note < sizeof(note_freq) / sizeof(note_freq[0])) {
        Buzzer_Beep(note_freq[note], duration);
    }
}