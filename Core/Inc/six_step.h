#ifndef SIX_STEP_H
#define SIX_STEP_H

#include "foc.h"

void SixStep_Init(Motor_t *motor, float voltage_mag);
void SixStep_Run(Motor_t *motor);

#endif
