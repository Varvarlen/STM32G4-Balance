#ifndef CURRENT_CTRL_H
#define CURRENT_CTRL_H

#include "foc.h"

/**
  * @brief  一次电流环控制（在 ADC ISR 中调用）
  * @param  motor: 电机对象指针
  * @retval 无
  */
void CurrentCtrl_Run(Motor_t *motor);

#endif
