#include "balance_ctrl.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

void BalanceCtrl_Init(BalanceCtrl_t *bc)
{
    bc->kp_angle     = BALANCE_KP_DEFAULT;
    bc->kd_gyro      = BALANCE_KD_DEFAULT;

    bc->target_angle   = 0.0f;
    bc->target_speed   = 0.0f;
    bc->target_yaw_rate = 0.0f;
    bc->steer          = 0.0f;
    bc->output_max     = BALANCE_OUTPUT_MAX;

    bc->tilt_angle   = 0.0f;
    bc->gyro_rate    = 0.0f;
    bc->gyro_filt    = 0.0f;
    bc->balance_out  = 0.0f;
    bc->speed_ref_l  = 0.0f;
    bc->speed_ref_r  = 0.0f;
    bc->active       = 0;
}

void BalanceCtrl_Run(BalanceCtrl_t *bc)
{
    COMPILER_BARRIER();  // CLI任务可能已修改 kp_angle/kd_gyro/target_angle/output_max, 强制从内存重载

    if (!bc->active) {
        bc->balance_out = 0.0f;
        bc->speed_ref_l = 0.0f;
        bc->speed_ref_r = 0.0f;
        return;
    }

    // 陀螺仪EMA滤波 (τ≈5ms), 必须在保护判断之前更新
    bc->gyro_filt += (bc->gyro_rate - bc->gyro_filt) * BALANCE_GYRO_EMA_ALPHA;

    // 倾倒保护: 倾角或角速度超限自动急停
    // 前倾(tilt变负) → target - (-30) = +28 → 正RPM向前追 ✓
    float tilt_err = bc->target_angle - bc->tilt_angle;
    if (tilt_err > BALANCE_TILT_MAX || tilt_err < -BALANCE_TILT_MAX ||
        bc->gyro_filt > BALANCE_GYRO_MAX || bc->gyro_filt < -BALANCE_GYRO_MAX) {
        bc->active = 0;
        bc->balance_out = 0.0f;
        bc->speed_ref_l = 0.0f;
        bc->speed_ref_r = 0.0f;
        printf("[PROTECT] tilt=%.1f target=%.1f err=%.1f gyro_filt=%.0f\r\n",
               bc->tilt_angle, bc->target_angle, tilt_err, bc->gyro_filt);
        bc->gyro_filt = 0.0f;  // 重置, 避免下次B被旧值误触
        return;
    }

    // target_speed 由速度外环通过 target_angle 间接控制
    float output = bc->kp_angle * tilt_err
                 - bc->kd_gyro  * bc->gyro_filt;

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
