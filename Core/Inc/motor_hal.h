#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include "foc.h"

// 使能所有电机（拉高 PC14 → MP6536 SHDNB）
void Motor_Enable(void);
// 禁能所有电机（拉低 PC14）
void Motor_Disable(void);
// 启动指定电机的 PWM 输出
void Motor_StartPWM(Motor_t *motor);
// 停止指定电机的 PWM 输出
void Motor_StopPWM(Motor_t *motor);
// 设置三相占空比 [0, 1]
void Motor_SetDuty(Motor_t *motor, float duty_a, float duty_b, float duty_c);
// 设置 q 轴电流给定（带限幅 ±2A）
void Motor_SetIqRef(Motor_t *motor, float iq_ref);
// 单电机失能：停止 FOC 闭环 + 50% 零电压 PWM（电机不转但不关总使能）
void Motor_Neutralize(Motor_t *motor);

#endif
