/*
 * buzzer.c
 *
 *  Created on: 2026-04-01
 *      Author: User
 */

#include "../Inc/buzzer.h"
#include "tim.h"
#include "cmsis_os.h"
#include "task.h"

static uint8_t  b_active = 0;
static TickType_t b_stop_tick = 0;

// 为频率范围计算合适的 PSC 和 ARR
static void buzzer_calc_range(uint16_t f_start, uint16_t f_end,
                              uint32_t *psc, uint32_t *arr_start, uint32_t *arr_end)
{
    uint32_t tim_clock = 170000000;
    *psc = 170 - 1;
    *arr_start = tim_clock / (f_start * (*psc + 1)) - 1;
    *arr_end   = tim_clock / (f_end   * (*psc + 1)) - 1;

    while (*arr_start > 65535 || *arr_end > 65535) {
        (*psc)++;
        *arr_start = tim_clock / (f_start * (*psc + 1)) - 1;
        *arr_end   = tim_clock / (f_end   * (*psc + 1)) - 1;
    }
}

// 配置 PA8→TIM1_CH1 PWM 并启动
static void buzzer_hw_start(uint32_t psc, uint32_t arr)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr / 2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Buzzer_Init(void)
{
    Buzzer_Stop();
}

void Buzzer_Beep(uint16_t freq, uint16_t duration)
{
    if (freq == 0) {
        Buzzer_Stop();
        return;
    }

    uint32_t psc, arr, arr_end;
    buzzer_calc_range(freq, freq, &psc, &arr, &arr_end);
    buzzer_hw_start(psc, arr);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        b_stop_tick = xTaskGetTickCount() + pdMS_TO_TICKS(duration);
        b_active = 1;
    } else {
        HAL_Delay(duration);
        Buzzer_Stop();
    }
}

void Buzzer_Sweep(uint16_t freq_start, uint16_t freq_end, uint16_t duration_ms)
{
    if (freq_start == 0 || freq_end == 0 || duration_ms == 0) return;
    if (freq_start == freq_end) {
        Buzzer_Beep(freq_start, duration_ms);
        return;
    }

    uint32_t psc, arr_start, arr_end;
    buzzer_calc_range(freq_start, freq_end, &psc, &arr_start, &arr_end);
    buzzer_hw_start(psc, arr_start);

    int steps = 30;
    int step_ms = duration_ms / steps;
    if (step_ms < 1) step_ms = 1;

    int32_t diff = (int32_t)arr_end - (int32_t)arr_start;
    uint8_t rtos = (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);

    for (int i = 1; i <= steps; i++) {
        uint32_t arr = (uint32_t)((int32_t)arr_start + (diff * i) / steps);
        __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr / 2);

        if (rtos) osDelay(step_ms);
        else      HAL_Delay(step_ms);
    }

    Buzzer_Stop();
}

void Buzzer_Stop(void)
{
    b_active = 0;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void Buzzer_Update(void)
{
    if (b_active && xTaskGetTickCount() >= b_stop_tick) {
        Buzzer_Stop();
    }
}
