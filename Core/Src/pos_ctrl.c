#include "pos_ctrl.h"
#include "speed_ctrl.h"
#include <math.h>

void PosCtrl_Init(PosCtrl_t *pc, float kp, float speed_max)
{
    pc->kp = kp;
    pc->pos_ref = 0.0f;
    pc->speed_max = speed_max;
    pc->pos_err_filt = 0.0f;
    pc->speed_ref_ramp = 0.0f;
    pc->active = 0;
}

float PosCtrl_Run(PosCtrl_t *pc, float pos_fb)
{
    float error = pc->pos_ref - pos_fb;
    // EMA τ≈5ms: 抑制量化噪声 (14-bit LSB=0.00038rad), 不再兼任斜坡
    pc->pos_err_filt += (error - pc->pos_err_filt) * 0.181f;
    float speed_raw = pc->pos_err_filt * pc->kp;
    if (speed_raw > pc->speed_max) speed_raw = pc->speed_max;
    else if (speed_raw < -pc->speed_max) speed_raw = -pc->speed_max;
    // 显式斜坡 (2000 RPM/s), 阶跃时软化速度参考
    float step = POS_RAMP_MAX * SPEED_LOOP_DT;
    float err = speed_raw - pc->speed_ref_ramp;
    if (err > step) pc->speed_ref_ramp += step;
    else if (err < -step) pc->speed_ref_ramp -= step;
    else pc->speed_ref_ramp = speed_raw;
    return pc->speed_ref_ramp;
}

void PosCtrl_EnterMode(PosCtrl_t *pc, float pos_ref, float cur_speed)
{
    pc->pos_ref = pos_ref;
    pc->pos_err_filt = 0.0f;  // 进入模式时复位, 避免旧值冲击
    pc->speed_ref_ramp = cur_speed;  // 从当前速度开始斜坡, 防止跃变
    pc->active = 1;
}

void PosCtrl_ExitMode(PosCtrl_t *pc)
{
    pc->active = 0;
    pc->pos_ref = 0.0f;
    pc->speed_ref_ramp = 0.0f;
}
