#include "foc.h"
#include "motor_hal.h"
#include "main.h"
#include "tim.h"

// 全局电机对象
Motor_t g_motor[2];

void FOC_Init(void)
{
    // --- M1 (TIM4, 编码器 PB4=index 0) ---
    // PCB丝印UVW ≡ 软件ABC: U=A, V=B, W=C
    g_motor[0].motor_id = 0;
    g_motor[0].pwm.htim = &htim4;
    g_motor[0].pwm.ch_a = TIM_CHANNEL_1;  // PB6 → M1-U(A)
    g_motor[0].pwm.ch_b = TIM_CHANNEL_2;  // PB7 → M1-V(B)
    g_motor[0].pwm.ch_c = TIM_CHANNEL_4;  // PB9 → M1-W(C)
    g_motor[0].pwm.arr = 8499;
    g_motor[0].ch_u = INA240_MOTOR1_W;    // PA6 → M1-U(A)相 (实验C确认)
    g_motor[0].ch_v = INA240_MOTOR1_U;    // PA5 → M1-V(B)相 (实验C确认)
    g_motor[0].phase_comp = 0.0f;         // A/B传感器=标准Clarke, 无需旋转
    g_motor[0].direction = 1;
    g_motor[0].mode = MOTOR_MODE_OFF;
    g_motor[0].dt = FOC_DT;
    g_motor[0].voltage_mag = 0.0f;
    g_motor[0].iq_ref = 0.0f;
    g_motor[0].id_ref = 0.0f;

    // --- M2 (TIM3, 编码器 PA4=index 1) ---
    // PWM A/C交换 + 电流传感器交换 → 等效标准A/B相序, phase_comp=0
    // 详见 docs/FOC_DEBUG_EXPERIENCE.md
    g_motor[1].motor_id = 1;
    g_motor[1].pwm.htim = &htim3;
    g_motor[1].pwm.ch_a = TIM_CHANNEL_4;  // PB1 → M2-W(C)
    g_motor[1].pwm.ch_b = TIM_CHANNEL_3;  // PB0 → M2-V(B)
    g_motor[1].pwm.ch_c = TIM_CHANNEL_2;  // PA7 → M2-U(A)
    g_motor[1].pwm.arr = 8499;
    g_motor[1].ch_u = INA240_MOTOR2_W;    // PB2 → M2-W(C)相
    g_motor[1].ch_v = INA240_MOTOR2_U;    // PC4 → M2-V(B)相
    g_motor[1].phase_comp = 0.0f;
    g_motor[1].direction = 1;
    g_motor[1].mode = MOTOR_MODE_OFF;
    g_motor[1].dt = FOC_DT;
    g_motor[1].voltage_mag = 0.0f;
    g_motor[1].iq_ref = 0.0f;
    g_motor[1].id_ref = 0.0f;

    // PI 参数 (两电机相同)
    PI_Init(&g_motor[0].id_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
    PI_Init(&g_motor[0].iq_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
    PI_Init(&g_motor[1].id_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
    PI_Init(&g_motor[1].iq_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
}
