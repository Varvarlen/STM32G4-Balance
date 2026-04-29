#include "current_ctrl.h"
#include "transform.h"
#include "svpwm.h"
#include "ina240.h"
#include "fast_math.h"

void CurrentCtrl_Run(Motor_t *motor)
{
    // 1. 读电流
    float Ia = INA240_GetCurrentFast(motor->ch_u);
    float Ib = INA240_GetCurrentFast(motor->ch_v);
    float Ic = -(Ia + Ib);

    motor->ia = Ia;
    motor->ib = Ib;
    motor->ic = Ic;

    // 2. Clarke
    float I_alpha, I_beta;
    Clarke(Ia, Ib, Ic, &I_alpha, &I_beta);

    // 3. Park/InvPark 共享 sincos（含相位补偿）
    float s, c;
    fast_sincos(motor->elec_angle + motor->phase_comp, &s, &c);

    float I_d =  I_alpha * c + I_beta * s;
    float I_q = -I_alpha * s + I_beta * c;
    motor->id = I_d;
    motor->iq = I_q;

    // 4. PI
    float V_d = PI_Step(&motor->id_pi, motor->id_ref - I_d, motor->dt);
    float V_q = PI_Step(&motor->iq_pi, motor->iq_ref - I_q, motor->dt);
    motor->vd = V_d;
    motor->vq = V_q;

    // 5. 逆 Park
    float V_alpha = V_d * c - V_q * s;
    float V_beta  = V_d * s + V_q * c;

    // 6. SVPWM
    SVPWM_SetVab(V_alpha, V_beta, motor);
}
