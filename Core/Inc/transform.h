#ifndef TRANSFORM_H
#define TRANSFORM_H

// Clarke 变换：Ia, Ib, Ic → Iα, Iβ（幅值不变形式）
void Clarke(float Ia, float Ib, float Ic, float *I_alpha, float *I_beta);

// Park 变换：Iα, Iβ → Id, Iq
void Park(float I_alpha, float I_beta, float theta, float *I_d, float *I_q);

// 逆 Park 变换：Vd, Vq → Vα, Vβ
void InvPark(float V_d, float V_q, float theta, float *V_alpha, float *V_beta);

#endif
