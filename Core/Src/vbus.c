#include "vbus.h"
#include "adc.h"
#include "calibration.h"
#include <stdio.h>

// 分压比理论值 (R1=100k / R2=10k → Vbus*10/(100+10) = Vbus/11)
// 未校准时使用此理论值: Vbus = adc_val * (3.3/4096) * 11.0
#define VBUS_DIVIDER_RATIO      11.0f       // 分压比: (R1+R2)/R2 = 110/10
#define VBUS_ADC_REF            3.3f        // ADC 参考电压
#define VBUS_ADC_RESOLUTION     4096.0f     // 12-bit
#define VBUS_DEFAULT_SLOPE      (VBUS_DIVIDER_RATIO * VBUS_ADC_REF / VBUS_ADC_RESOLUTION)  // ≈0.00885 V/LSB
#define VBUS_DEFAULT_OFFSET     0.0f

#define VBUS_CAL_SAMPLES        200         // 每个校准点采样次数
#define VBUS_CAL_DELAY_MS       1           // 采样间隔

static float g_vbus_slope  = VBUS_DEFAULT_SLOPE;
static float g_vbus_offset = VBUS_DEFAULT_OFFSET;
static uint8_t g_vbus_calibrated = 0;

// 校准中间状态
static float vbus_cal_v[2]   = {0};    // 用户提供的实际电压值
static float vbus_cal_adc[2] = {0};    // ADC 均值
static int   vbus_cal_count  = 0;

/**
  * @brief 软件触发一次注入转换, 返回 JDR1 原始值
  * @retval 12-bit ADC 原始值 (0-4095)
  */
static uint16_t vbus_read_raw(void)
{
    // 直接操作寄存器: 软件触发注入转换, 避免 HAL 状态机在时刻
    // 在 regular DMA 连续转换下可能出现的竞态问题
    ADC2->CR |= ADC_CR_JADSTART;            // 软件触发注入组
    while (!(ADC2->ISR & ADC_ISR_JEOC));    // 等待注入转换完成
    ADC2->ISR |= ADC_ISR_JEOC;              // 清除 JEOC 标志 (w1c)
    return (uint16_t)ADC2->JDR1;
}

void VBUS_Init(void)
{
    // 从 Flash 加载校准系数
    if (CALIB_FlashIsValid(&g_calib) && g_calib.vbus_slope > 0.0001f) {
        g_vbus_slope  = g_calib.vbus_slope;
        g_vbus_offset = g_calib.vbus_offset;
        g_vbus_calibrated = 1;
    } else {
        g_vbus_slope  = VBUS_DEFAULT_SLOPE;
        g_vbus_offset = VBUS_DEFAULT_OFFSET;
        g_vbus_calibrated = 0;
    }
}

float VBUS_Read(void)
{
    uint16_t raw = vbus_read_raw();
    return g_vbus_slope * (float)raw + g_vbus_offset;
}

void VBUS_CalSample(float actual_voltage)
{
    if (vbus_cal_count >= 2) {
        vbus_cal_count = 0;  // 重置, 开始新一轮校准
        printf("[VBUS CAL] Reset, starting new calibration\r\n");
    }

    // 多次采样取均值
    float adc_sum = 0.0f;
    for (int i = 0; i < VBUS_CAL_SAMPLES; i++) {
        adc_sum += (float)vbus_read_raw();
        HAL_Delay(VBUS_CAL_DELAY_MS);
    }
    float adc_mean = adc_sum / (float)VBUS_CAL_SAMPLES;

    vbus_cal_v[vbus_cal_count]   = actual_voltage;
    vbus_cal_adc[vbus_cal_count] = adc_mean;
    vbus_cal_count++;

    printf("[VBUS CAL] Point %d: V=%.3fV ADC=%.1f\r\n",
           vbus_cal_count, actual_voltage, adc_mean);

    if (vbus_cal_count == 2) {
        float dv = vbus_cal_v[1] - vbus_cal_v[0];
        float da = vbus_cal_adc[1] - vbus_cal_adc[0];
        if (da < 0.001f) {
            printf("[VBUS CAL] FAILED: ADC values too close (%.1f vs %.1f)\r\n",
                   vbus_cal_adc[0], vbus_cal_adc[1]);
            vbus_cal_count = 0;
            return;
        }
        g_vbus_slope  = dv / da;
        g_vbus_offset = vbus_cal_v[0] - g_vbus_slope * vbus_cal_adc[0];
        g_vbus_calibrated = 1;

        // 保存到 Flash
        g_calib.vbus_slope  = g_vbus_slope;
        g_calib.vbus_offset = g_vbus_offset;
        if (CALIB_FlashSave(&g_calib) == HAL_OK) {
            printf("[VBUS CAL] OK: slope=%.6f V/LSB offset=%.3fV (saved)\r\n",
                   g_vbus_slope, g_vbus_offset);
        } else {
            printf("[VBUS CAL] OK: slope=%.6f V/LSB offset=%.3fV (flash FAILED)\r\n",
                   g_vbus_slope, g_vbus_offset);
        }
        vbus_cal_count = 0;
    }
}

uint8_t VBUS_IsCalibrated(void)
{
    return g_vbus_calibrated;
}

float VBUS_GetSlope(void)  { return g_vbus_slope; }
float VBUS_GetOffset(void) { return g_vbus_offset; }
