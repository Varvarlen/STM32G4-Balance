#include "pos_ctrl.h"
#include <math.h>

void PosCtrl_Init(PosCtrl_t *pc, float kp, float speed_max)
{
    pc->kp = kp;
    pc->pos_ref = 0.0f;
    pc->speed_max = speed_max;
    pc->active = 0;
}

float PosCtrl_Run(PosCtrl_t *pc, float pos_fb)
{
    float error = pc->pos_ref - pos_fb;
    // 到位死区: 防止编码器量化噪声 (0.022°/LSB) 经 Kp=210 放大引发啸叫
    if (fabsf(error) < POS_DEADBAND_RAD) {
        return 0.0f;
    }
    float speed_ref = error * pc->kp;
    if (speed_ref > pc->speed_max) speed_ref = pc->speed_max;
    else if (speed_ref < -pc->speed_max) speed_ref = -pc->speed_max;
    return speed_ref;
}

void PosCtrl_EnterMode(PosCtrl_t *pc, float pos_ref)
{
    pc->pos_ref = pos_ref;
    pc->active = 1;
}

void PosCtrl_ExitMode(PosCtrl_t *pc)
{
    pc->active = 0;
    pc->pos_ref = 0.0f;
}
