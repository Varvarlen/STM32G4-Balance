#include "pos_ctrl.h"
#include "speed_ctrl.h"
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
    // EMA τ≈5ms: 抑制量化噪声 (14-bit LSB=0.00038rad)
    pc->pos_err_filt += (error - pc->pos_err_filt) * POS_ERR_EMA_ALPHA;
    float speed_raw = pc->pos_err_filt * pc->kp;
    if (speed_raw > pc->speed_max) speed_raw = pc->speed_max;
    else if (speed_raw < -pc->speed_max) speed_raw = -pc->speed_max;
    // 速度环 SPEED_RAMP_MAX (20000 RPM/s) 接管斜坡平滑
    return speed_raw;
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
