#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "tim.h"
#include "ina240.h"
#include "pi.h"

#define MOTOR_COUNT     2U
#define FOC_PWM_FREQ    10000U    // 10kHz
#define FOC_DT          (1.0f / FOC_PWM_FREQ)  // 100us
#define MOTOR_POLE_PAIRS 7U

// 电机工作模式
typedef enum {
    MOTOR_MODE_OFF = 0,
    MOTOR_MODE_SIX_STEP,
    MOTOR_MODE_VOLTAGE_SINE,
    MOTOR_MODE_CURRENT_LOOP
} MotorMode_t;

// PWM 三相通道映射（一个定时器上的三路 PWM）
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_a;   // TIM_CHANNEL_x — A 相
    uint32_t ch_b;   // TIM_CHANNEL_x — B 相
    uint32_t ch_c;   // TIM_CHANNEL_x — C 相
    uint32_t arr;    // AutoReload 值（用于占空比计算）
} PWM_Channels_t;

typedef struct {
    uint8_t motor_id;              // 0 = M1, 1 = M2
    PWM_Channels_t pwm;            // PWM 输出通道
    INA240_Channel_t ch_u;         // 电流采样第一相
    INA240_Channel_t ch_v;         // 电流采样第二相
    MotorMode_t mode;              // 当前工作模式
    float elec_angle;              // 电角度 (rad)，由编码器 DMA ISR 更新
    float dt;                      // 控制周期 (s)
    float voltage_mag;             // 开环电压幅值 (V)
    float iq_ref;                  // q 轴电流给定 (A)，上层任务写入
    float id_ref;                  // d 轴电流给定 (A)，通常为 0
    PI_t id_pi;                    // d 轴 PI 控制器
    PI_t iq_pi;                    // q 轴 PI 控制器
    float ia, ib, ic;              // 三相电流（调试用）
    float id, iq;                  // dq 轴电流（调试用）
    float vd, vq;                  // dq 轴电压输出（调试用）
    float duty_a, duty_b, duty_c;  // 三相占空比 [0, 1]（调试用）
} Motor_t;

extern Motor_t g_motor[2];

void FOC_Init(void);

#endif
