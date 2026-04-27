#include "transform.h"
#include <math.h>

#define ONE_OVER_SQRT3  0.5773502692f  // 1/sqrt(3)
#define SQRT3_OVER_2     0.8660254038f  // sqrt(3)/2

void Clarke(float Ia, float Ib, float Ic, float *I_alpha, float *I_beta)
{
    *I_alpha = Ia;
    *I_beta = (Ia + 2.0f * Ib) * ONE_OVER_SQRT3;
    (void)Ic;
}

void Park(float I_alpha, float I_beta, float theta, float *I_d, float *I_q)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *I_d =  I_alpha * c + I_beta * s;
    *I_q = -I_alpha * s + I_beta * c;
}

void InvPark(float V_d, float V_q, float theta, float *V_alpha, float *V_beta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *V_alpha = V_d * c - V_q * s;
    *V_beta = V_d * s + V_q * c;
}
