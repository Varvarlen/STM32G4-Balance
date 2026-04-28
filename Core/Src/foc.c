#include "foc.h"
#include "motor_hal.h"
#include "main.h"
#include "tim.h"

// 全局电机对象
Motor_t g_motor[2];

void FOC_Init(void)
{
    // --- M1 (TIM4, 编码器 PB4=index 0) ---
    g_motor[0].motor_id = 0;
    g_motor[0].pwm.htim = &htim4;
    g_motor[0].pwm.ch_a = TIM_CHANNEL_1;  // PB6
    g_motor[0].pwm.ch_b = TIM_CHANNEL_2;  // PB7
    g_motor[0].pwm.ch_c = TIM_CHANNEL_4;  // PB9
    g_motor[0].pwm.arr = 8499;
    g_motor[0].ch_u = INA240_MOTOR1_U;    // PA5, ADC2_IN13
    g_motor[0].ch_v = INA240_MOTOR1_W;    // PA6, ADC2_IN3
    g_motor[0].direction = 1;
    g_motor[0].mode = MOTOR_MODE_OFF;
    g_motor[0].dt = FOC_DT;
    g_motor[0].voltage_mag = 0.0f;
    g_motor[0].iq_ref = 0.0f;
    g_motor[0].id_ref = 0.0f;

    // --- M2 (TIM3, 编码器 PA4=index 1) ---
    g_motor[1].motor_id = 1;
    g_motor[1].pwm.htim = &htim3;
    g_motor[1].pwm.ch_a = TIM_CHANNEL_2;  // PA7
    g_motor[1].pwm.ch_b = TIM_CHANNEL_3;  // PB0
    g_motor[1].pwm.ch_c = TIM_CHANNEL_4;  // PB1
    g_motor[1].pwm.arr = 8499;
    g_motor[1].ch_u = INA240_MOTOR2_U;    // PC4, ADC2_IN5
    g_motor[1].ch_v = INA240_MOTOR2_W;    // PB2, ADC2_IN12
    g_motor[1].direction = 1;
    g_motor[1].mode = MOTOR_MODE_OFF;
    g_motor[1].dt = FOC_DT;
    g_motor[1].voltage_mag = 0.0f;
    g_motor[1].iq_ref = 0.0f;
    g_motor[1].id_ref = 0.0f;

    // 初始化 PI 控制器（电流环参数：Kp=1.0, Ki=50, 限幅 ±7.4V）
    for (int i = 0; i < 2; i++)
    {
        PI_Init(&g_motor[i].id_pi, 1.0f, 50.0f, 7.4f, -7.4f);
        PI_Init(&g_motor[i].iq_pi, 1.0f, 50.0f, 7.4f, -7.4f);
    }
}
