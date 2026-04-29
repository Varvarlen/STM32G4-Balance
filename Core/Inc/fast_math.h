#ifndef FAST_MATH_H
#define FAST_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

// 快速 sincos 查表（512 点线性插值），替代 ISR 中的 sinf/cosf
void fast_sincos(float theta, float *s, float *c);

#ifdef __cplusplus
}
#endif

#endif
