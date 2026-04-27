#include "current_ctrl.h"
#include "transform.h"
#include "svpwm.h"
#include "ina240.h"

/**
  * @brief  一次电流环控制：读电流 → Clarke → Park → PI → 逆Park → SVPWM
  * @param  motor: 电机对象指针
  * @retval 无
  * @note   此函数在 ADC ISR 中调用（10kHz），不调用任何 FreeRTOS API
  */
void CurrentCtrl_Run(Motor_t *motor)
{
    // 1. 读电流（INA240 快速接口，直接从 ADC DMA 缓冲区读取）
    float Ia = INA240_GetCurrentFast(motor->ch_u);
    float Ib = INA240_GetCurrentFast(motor->ch_v);
    float Ic = -(Ia + Ib);

    motor->ia = Ia;
    motor->ib = Ib;
    motor->ic = Ic;

    // 2. Clarke 变换 + Park 变换（三相静止 → 两相旋转）
    float I_alpha, I_beta;
    Clarke(Ia, Ib, Ic, &I_alpha, &I_beta);

    float I_d, I_q;
    Park(I_alpha, I_beta, motor->elec_angle, &I_d, &I_q);

    motor->id = I_d;
    motor->iq = I_q;

    // 3. PI 控制（d/q 轴电流闭环）
    float V_d = PI_Step(&motor->id_pi, motor->id_ref - I_d, motor->dt);
    float V_q = PI_Step(&motor->iq_pi, motor->iq_ref - I_q, motor->dt);

    motor->vd = V_d;
    motor->vq = V_q;

    // 4. 逆 Park 变换 + SVPWM（两相旋转 → 三相静止 → PWM 占空比）
    float V_alpha, V_beta;
    InvPark(V_d, V_q, motor->elec_angle, &V_alpha, &V_beta);
    SVPWM_SetVab(V_alpha, V_beta, motor);
}
