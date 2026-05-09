#include "pi.h"

void PI_Init(PI_t *pi, float kp, float ki, float out_max, float out_min)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_max = out_max;
    pi->out_min = out_min;
}

float PI_Step(PI_t *pi, float error, float dt)
{
    float p_term = pi->kp * error;
    float output = p_term + pi->integral;

    // 条件积分（clamping anti-windup）：
    // 输出饱和且 error 同向时冻结积分，error 反向时允许退饱和
    if (!((output >= pi->out_max && error > 0.0f) ||
          (output <= pi->out_min && error < 0.0f))) {
        pi->integral += pi->ki * error * dt;
    }

    // 积分限幅
    if (pi->integral > pi->out_max)  pi->integral = pi->out_max;
    if (pi->integral < pi->out_min)  pi->integral = pi->out_min;

    output = p_term + pi->integral;

    // 输出限幅
    if (output > pi->out_max)  output = pi->out_max;
    if (output < pi->out_min)  output = pi->out_min;

    return output;
}

void PI_Reset(PI_t *pi)
{
    pi->integral = 0.0f;
}
