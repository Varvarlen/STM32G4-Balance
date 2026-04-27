#ifndef SVPWM_H
#define SVPWM_H

#include "foc.h"

// SVPWM 计算：Vα, Vβ → 三相占空比 + 写 PWM 寄存器
void SVPWM_SetVab(float v_alpha, float v_beta, Motor_t *motor);

#endif
