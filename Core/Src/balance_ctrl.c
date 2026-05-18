#include "balance_ctrl.h"
#include <math.h>

void BalanceCtrl_Init(BalanceCtrl_t *bc)
{
    bc->kp_angle     = BALANCE_KP_DEFAULT;
    bc->kd_gyro      = BALANCE_KD_DEFAULT;
    bc->kff_speed    = 0.0f;
    bc->target_angle = 0.0f;
    bc->target_speed = 0.0f;
    bc->steer        = 0.0f;
    bc->output_max   = BALANCE_OUTPUT_MAX;

    bc->tilt_angle   = 0.0f;
    bc->gyro_rate    = 0.0f;
    bc->balance_out  = 0.0f;
    bc->speed_ref_l  = 0.0f;
    bc->speed_ref_r  = 0.0f;
    bc->active       = 0;
}

void BalanceCtrl_Run(BalanceCtrl_t *bc)
{
    if (!bc->active) {
        bc->balance_out = 0.0f;
        bc->speed_ref_l = 0.0f;
        bc->speed_ref_r = 0.0f;
        return;
    }

    // 平衡 PID: 角度比例 + 角速度阻尼 + 遥控速度偏置
    float tilt_err = bc->target_angle - bc->tilt_angle;
    float output = bc->kp_angle * tilt_err
                 + bc->kd_gyro  * bc->gyro_rate
                 + bc->target_speed;

    // 限幅
    if (output >  bc->output_max) output =  bc->output_max;
    if (output < -bc->output_max) output = -bc->output_max;

    bc->balance_out = output;

    // 差速混合: 左右轮 = 平衡输出 ± 转向偏置
    float steer = bc->steer;
    if (steer >  BALANCE_STEER_MAX) steer =  BALANCE_STEER_MAX;
    if (steer < -BALANCE_STEER_MAX) steer = -BALANCE_STEER_MAX;

    bc->speed_ref_l = output + steer;
    bc->speed_ref_r = output - steer;
}
