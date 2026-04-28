#include "six_step.h"
#include "motor_hal.h"
#include "mt6701.h"

// 6 扇区对应三相开关表 (1=上桥通, 0=下桥通)
typedef struct {
    float a, b, c;
} StepTable_t;

static const StepTable_t step_table[6] = {
    { 1, 0, 0 },   // 扇区 0: A+ B-  (C 关断)
    { 1, 1, 0 },   // 扇区 1: A+ C-  (B 关断)
    { 0, 1, 0 },   // 扇区 2: B+ C-  (A 关断)
    { 0, 1, 1 },   // 扇区 3: B+ A-  (C 关断)
    { 0, 0, 1 },   // 扇区 4: C+ A-  (B 关断)
    { 1, 0, 1 },   // 扇区 5: C+ B-  (A 关断)
};

void SixStep_Init(Motor_t *motor, float voltage_mag)
{
    motor->mode = MOTOR_MODE_SIX_STEP;
    motor->voltage_mag = voltage_mag;
}

void SixStep_Run(Motor_t *motor)
{
    // 读机械角度 (0-16383) → 电气角度 → 扇区 (0-5)
    uint16_t raw_angle = MT6701_ReadAngle(motor->motor_id);
    uint16_t elec_raw = (raw_angle * MOTOR_POLE_PAIRS) % MT6701_RESOLUTION;
    uint8_t sector = (uint8_t)((uint32_t)elec_raw * 6 / MT6701_RESOLUTION);

    // 反转方向：扇区逆序
    if (motor->direction < 0)
        sector = (6 - sector) % 6;

    const StepTable_t *step = &step_table[sector];
    float mag = motor->voltage_mag;

    Motor_SetDuty(motor, step->a * mag, step->b * mag, step->c * mag);
}
