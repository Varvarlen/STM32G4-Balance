#include "svpwm.h"
#include "motor_hal.h"
#include <math.h>

void SVPWM_SetVab(float v_alpha, float v_beta, Motor_t *motor)
{
    // 归一化到 Vbus 标幺（用实测母线电压, BalanceLoop 1ms 更新缓存）
    float vbus = g_foc_vbus;
    if (vbus < 3.0f) vbus = 3.0f;  // 防止除零/低压异常
    float inv_vbus = 1.0f / vbus;
    float va = v_alpha * inv_vbus;
    float vb = v_beta * inv_vbus;

    // 逆 Clarke（Vα,Vβ → 三相电压，标幺值）
    float v_a = va;
    float v_b = -0.5f * va + 0.8660254038f * vb;
    float v_c = -0.5f * va - 0.8660254038f * vb;

    // 注入三次谐波（中线钳位 = SVPWM 等效）
    float v_max = v_a;
    if (v_b > v_max) v_max = v_b;
    if (v_c > v_max) v_max = v_c;

    float v_min = v_a;
    if (v_b < v_min) v_min = v_b;
    if (v_c < v_min) v_min = v_c;

    float v_offset = (v_max + v_min) * 0.5f;
    v_a -= v_offset;
    v_b -= v_offset;
    v_c -= v_offset;

    // 标幺值 + 0.5 = 占空比 [0, 1]
    float duty_a = v_a + 0.5f;
    float duty_b = v_b + 0.5f;
    float duty_c = v_c + 0.5f;

    Motor_SetDuty(motor, duty_a, duty_b, duty_c);
    motor->duty_a = duty_a;
    motor->duty_b = duty_b;
    motor->duty_c = duty_c;
}
