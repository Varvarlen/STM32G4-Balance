#include "pos_ctrl.h"
#include <math.h>

void PosCtrl_Init(PosCtrl_t *pc, float kp, float speed_max)
{
    pc->kp = kp;
    pc->pos_ref = 0.0f;
    pc->speed_max = speed_max;
    pc->pos_err_filt = 0.0f;
    pc->active = 0;
}

float PosCtrl_Run(PosCtrl_t *pc, float pos_fb)
{
    float error = pc->pos_ref - pos_fb;
    // LPF (τ≈50ms) 抑制 EKF 噪声, 25Hz 处衰减 ~18dB
    pc->pos_err_filt += (error - pc->pos_err_filt) * 0.02f;
    float speed_ref = pc->pos_err_filt * pc->kp;
    if (speed_ref > pc->speed_max) speed_ref = pc->speed_max;
    else if (speed_ref < -pc->speed_max) speed_ref = -pc->speed_max;
    return speed_ref;
}

void PosCtrl_EnterMode(PosCtrl_t *pc, float pos_ref)
{
    pc->pos_ref = pos_ref;
    pc->pos_err_filt = 0.0f;  // 进入模式时复位, 避免旧值冲击
    pc->active = 1;
}

void PosCtrl_ExitMode(PosCtrl_t *pc)
{
    pc->active = 0;
    pc->pos_ref = 0.0f;
}
