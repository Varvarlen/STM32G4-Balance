/*
 * ws2812b.c
 *
 *  Created on: 2026-04-01
 *      Author: User
 */

#include "ws2812b.h"
#include "main.h"
#include "tim.h"

// WS2812B时序参数（单位：ns）
#define WS2812B_T0H 400   // 0码高电平时间
#define WS2812B_T0L 850   // 0码低电平时间
#define WS2812B_T1H 800   // 1码高电平时间
#define WS2812B_T1L 450   // 1码低电平时间
#define WS2812B_TRS 50000 // 复位时间

// PWM配置参数
#define WS2812B_PWM_FREQ 800000  // 800kHz PWM频率
#define WS2812B_PWM_PERIOD 212   // PWM周期（213个计数）

// 计算PWM占空比
#define WS2812B_DUTY_0 ((WS2812B_T0H * (WS2812B_PWM_PERIOD + 1)) / 1250)
#define WS2812B_DUTY_1 ((WS2812B_T1H * (WS2812B_PWM_PERIOD + 1)) / 1250)

// DMA缓冲区大小（每个灯珠24位，每位需要1个PWM周期）
#define WS2812B_BUFFER_SIZE (WS2812B_LED_COUNT * 24+40)

// 全局变量
static uint32_t ws2812b_buffer[WS2812B_BUFFER_SIZE];  // DMA缓冲区
static WS2812B_Color_t ws2812b_leds[WS2812B_LED_COUNT]; // 灯珠颜色缓存
static TIM_HandleTypeDef *htim;  // 定时器句柄

/**
 * @brief 初始化WS2812B驱动
 * @param None
 * @retval None
 */
void WS2812B_Init(void)
{
    // 这里需要根据实际硬件配置定时器和DMA
    // 假设使用TIM2_CH3 (PA2)
    // 定时器配置代码会在CubeMX生成的tim.c中
    htim = &htim2;
    
    // 初始化灯珠颜色缓存
    for (uint8_t i = 0; i < WS2812B_LED_COUNT; i++) {
        ws2812b_leds[i].r = 0;
        ws2812b_leds[i].g = 0;
        ws2812b_leds[i].b = 0;
    }
    
    // 初始化DMA缓冲区
    for (uint16_t i = 0; i < WS2812B_BUFFER_SIZE; i++) {
        ws2812b_buffer[i] = 0;
    }
}

/**
 * @brief 将颜色数据转换为PWM占空比数据
 * @param None
 * @retval None
 */
static void WS2812B_UpdateBuffer(void)
{
    uint16_t index = 0;
    
    for (uint8_t led = 0; led < WS2812B_LED_COUNT; led++) {
        // WS2812B的数据格式是GRB
        uint32_t color = ((uint32_t)ws2812b_leds[led].g << 16) |
                         ((uint32_t)ws2812b_leds[led].r << 8) |
                         ((uint32_t)ws2812b_leds[led].b);
        
        // 从最高位开始处理
        for (int8_t bit = 23; bit >= 0; bit--) {
            if (color & (1 << bit)) {
                ws2812b_buffer[index++] = WS2812B_DUTY_1;
            } else {
                ws2812b_buffer[index++] = WS2812B_DUTY_0;
            }
        }
    }
}

/**
 * @brief 设置单个灯珠颜色
 * @param index: 灯珠索引（0-3）
 * @param color: 颜色值
 * @retval None
 */
void WS2812B_SetLED(uint8_t index, WS2812B_Color_t color)
{
    if (index < WS2812B_LED_COUNT) {
        ws2812b_leds[index] = color;
    }
}

/**
 * @brief 设置所有灯珠颜色
 * @param color: 颜色值
 * @retval None
 */
void WS2812B_SetAll(WS2812B_Color_t color)
{
    for (uint8_t i = 0; i < WS2812B_LED_COUNT; i++) {
        ws2812b_leds[i] = color;
    }
}

/**
 * @brief 更新灯珠显示（非阻塞）
 * @param None
 * @retval None
 */
void WS2812B_Update(void)
{
    // 更新DMA缓冲区
     WS2812B_UpdateBuffer();
    
    // 确保定时器和DMA已经初始化
    if (htim->State == HAL_TIM_STATE_READY) {
        // 启动DMA传输
        if (HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t *)ws2812b_buffer, WS2812B_BUFFER_SIZE) != HAL_OK) {
            HAL_Delay(500);// 错误处理
        }
    }
}

/**
 * @brief 呼吸效果
 * @param color: 颜色值
 * @param period: 呼吸周期（ms）
 * @retval None
 */
void WS2812B_Breathe(WS2812B_Color_t color, uint16_t period)
{
    static uint32_t last_time = 0;
    static uint8_t brightness = 0;
    static int8_t direction = 1;
    
    if (HAL_GetTick() - last_time > period / 256) {
        last_time = HAL_GetTick();
        
        // 计算当前亮度
        WS2812B_Color_t temp_color;
        temp_color.r = (color.r * brightness) / 255;
        temp_color.g = (color.g * brightness) / 255;
        temp_color.b = (color.b * brightness) / 255;
        
        // 设置所有灯珠
        WS2812B_SetAll(temp_color);
        WS2812B_Update();
        
        // 更新亮度
        brightness += direction;
        if (brightness == 255 || brightness == 0) {
            direction = -direction;
        }
    }
}

/**
 * @brief 流水灯效果
 * @param color: 颜色值
 * @param speed: 速度（ms）
 * @retval None
 */
void WS2812B_RunningLight(WS2812B_Color_t color, uint16_t speed)
{
    static uint32_t last_time = 0;
    static uint8_t current_led = 0;
    
    if (HAL_GetTick() - last_time > speed) {
        last_time = HAL_GetTick();
        
        // 关闭所有灯
        WS2812B_SetAll((WS2812B_Color_t){0, 0, 0});
        
        // 点亮当前灯
        WS2812B_SetLED(current_led, color);
        WS2812B_Update();
        
        // 更新当前灯索引
        current_led = (current_led + 1) % WS2812B_LED_COUNT;
    }
}

/**
 * @brief 闪烁效果
 * @param color: 颜色值
 * @param period: 闪烁周期（ms）
 * @retval None
 */
void WS2812B_Blink(WS2812B_Color_t color, uint16_t period)
{
    static uint32_t last_time = 0;
    static uint8_t state = 0;
    
    if (HAL_GetTick() - last_time > period / 2) {
        last_time = HAL_GetTick();
        
        if (state) {
            // 关闭所有灯
            WS2812B_SetAll((WS2812B_Color_t){0, 0, 0});
        } else {
            // 点亮所有灯
            WS2812B_SetAll(color);
        }
        
        WS2812B_Update();
        state = !state;
    }
}

/**
 * @brief 彩虹效果
 * @param speed: 速度（ms）
 * @retval None
 */
void WS2812B_Rainbow(uint16_t speed)
{
    static uint32_t last_time = 0;
    static uint8_t hue = 0;
    
    if (HAL_GetTick() - last_time > speed) {
        last_time = HAL_GetTick();
        
        for (uint8_t i = 0; i < WS2812B_LED_COUNT; i++) {
            // 计算每个灯珠的颜色
            uint8_t current_hue = (hue + i * 64) % 256;
            WS2812B_Color_t color;
            
            // HSL转RGB
            if (current_hue < 85) {
                color.r = 255 - current_hue * 3;
                color.g = current_hue * 3;
                color.b = 0;
            } else if (current_hue < 170) {
                current_hue -= 85;
                color.r = 0;
                color.g = 255 - current_hue * 3;
                color.b = current_hue * 3;
            } else {
                current_hue -= 170;
                color.r = current_hue * 3;
                color.g = 0;
                color.b = 255 - current_hue * 3;
            }
            
            WS2812B_SetLED(i, color);
        }
        
        WS2812B_Update();
        hue = (hue + 1) % 256;
    }
}

/**
 * @brief TIM PWM DMA传输完成回调
 * @param htim: 定时器句柄
 * @retval None
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2) {
        // 停止PWM输出
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
    }
}