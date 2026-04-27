#include "svpwm.h"
#include "motor_hal.h"
#include <math.h>

void SVPWM_SetVab(float v_alpha, float v_beta, Motor_t *motor)
{
    // 逆 Clarke（Vα,Vβ → 三相电压）
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + 0.8660254038f * v_beta;
    float v_c = -0.5f * v_alpha - 0.8660254038f * v_beta;

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

    // 归一化到 [0, 1]，中心为 0.5
    float duty_a = v_a + 0.5f;
    float duty_b = v_b + 0.5f;
    float duty_c = v_c + 0.5f;

    Motor_SetDuty(motor, duty_a, duty_b, duty_c);

    // 保存调试变量
    motor->duty_a = duty_a;
    motor->duty_b = duty_b;
    motor->duty_c = duty_c;
}
