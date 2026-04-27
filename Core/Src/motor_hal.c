#include "motor_hal.h"
#include "main.h"
#include "tim.h"

// PC14 在 CubeMX 中未设标签，直接使用 GPIOC + GPIO_PIN_14
#define MOTOR_ENABLE_PORT   GPIOC
#define MOTOR_ENABLE_PIN    GPIO_PIN_14

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, GPIO_PIN_SET);
    // TIM3 是主定时器，TIM4 由其 TRGO 驱动（Trigger Mode）
    // 启动 TIM3 CH2 确保主计数器运行，M1 才能获得 PWM 计数脉冲
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
}

void Motor_StartPWM(Motor_t *motor)
{
    if (motor == NULL) return;
    HAL_TIM_PWM_Start(motor->pwm.htim, motor->pwm.ch_a);
    HAL_TIM_PWM_Start(motor->pwm.htim, motor->pwm.ch_b);
    HAL_TIM_PWM_Start(motor->pwm.htim, motor->pwm.ch_c);
}

void Motor_StopPWM(Motor_t *motor)
{
    if (motor == NULL) return;
    HAL_TIM_PWM_Stop(motor->pwm.htim, motor->pwm.ch_a);
    HAL_TIM_PWM_Stop(motor->pwm.htim, motor->pwm.ch_b);
    HAL_TIM_PWM_Stop(motor->pwm.htim, motor->pwm.ch_c);
}

void Motor_SetDuty(Motor_t *motor, float duty_a, float duty_b, float duty_c)
{
    if (motor == NULL) return;
    // 限幅
    if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_c > 1.0f) duty_c = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f;

    uint32_t arr = motor->pwm.arr;
    __HAL_TIM_SET_COMPARE(motor->pwm.htim, motor->pwm.ch_a, (uint32_t)(duty_a * arr));
    __HAL_TIM_SET_COMPARE(motor->pwm.htim, motor->pwm.ch_b, (uint32_t)(duty_b * arr));
    __HAL_TIM_SET_COMPARE(motor->pwm.htim, motor->pwm.ch_c, (uint32_t)(duty_c * arr));
}
