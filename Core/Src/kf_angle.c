/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kf_angle.c
  * @brief   一维卡尔曼滤波器（含陀螺仪零偏估计）
  *
  *  状态方程:
  *    angle_k = angle_{k-1} + (gyro - bias) * dt
  *    bias_k  = bias_{k-1}
  *
  *  观测方程:
  *    accel_angle = angle + v
  ******************************************************************************
  */
/* USER CODE END Header */

#include "kf_angle.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  初始化卡尔曼滤波器
  * @param  kf     滤波器实例
  * @param  init_angle  初始倾角（°）
  * @param  qa     角度过程噪声
  * @param  qb     零偏过程噪声
  * @param  rm     加速度计观测噪声
  * @note   调参说明：
  *         增大 Q_angle → 更信任陀螺仪（响应更快，但噪声大）
  *         增大 Q_bias  → 零偏跟踪更快（但可能不稳定）
  *         增大 R_measure → 更信任陀螺仪积分（响应平滑，但延迟大）
  */
void KalmanAngle_Init(KalmanAngle_t *kf, float init_angle,
                      float qa, float qb, float rm)
{
    kf->angle = init_angle;
    kf->bias  = 0.0f;

    /* 协方差初始值（越大表示对初始状态越不确定） */
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;

    kf->Q_angle   = qa;
    kf->Q_bias    = qb;
    kf->R_measure = rm;
}

/**
  * @brief  预测步骤
  *         使用陀螺仪角速度积分更新倾角估计
  * @param  kf         滤波器实例
  * @param  gyro_rate  陀螺仪角速度（°/s）
  * @param  dt         距上次预测的时间间隔（s）
  */
void KalmanAngle_Predict(KalmanAngle_t *kf, float gyro_rate, float dt)
{
    /* 先验状态估计 */
    kf->angle += (gyro_rate - kf->bias) * dt;

    /* 先验协方差估计: P = F * P * F^T + Q */
    float dtP11 = dt * kf->P[1][1];

    kf->P[0][0] += dt * (dtP11 - kf->P[0][1] - kf->P[1][0]) + kf->Q_angle;
    kf->P[0][1] -= dtP11;
    kf->P[1][0] -= dtP11;
    kf->P[1][1] += kf->Q_bias * dt;
}

/**
  * @brief  更新步骤
  *         使用加速度计计算的倾角作为观测值进行校正
  * @param  kf           滤波器实例
  * @param  accel_angle  加速度计计算的倾角（°）
  */
void KalmanAngle_Update(KalmanAngle_t *kf, float accel_angle)
{
    /* 创新（观测残差），处理 ±180° 角度环绕 */
    float y = accel_angle - kf->angle;
    if (y > 180.0f) y -= 360.0f;
    if (y < -180.0f) y += 360.0f;

    /* 创新协方差: S = H * P * H^T + R */
    float S = kf->P[0][0] + kf->R_measure;

    /* 卡尔曼增益: K = P * H^T / S */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    /* 后验状态估计 */
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    /* 后验协方差估计: P = (I - K*H) * P */
    float P00_tmp = kf->P[0][0];
    float P01_tmp = kf->P[0][1];
    kf->P[0][0] -= K0 * P00_tmp;
    kf->P[0][1] -= K0 * P01_tmp;
    kf->P[1][0] -= K1 * P00_tmp;
    kf->P[1][1] -= K1 * P01_tmp;
}

/**
  * @brief  获取当前估计倾角（°）
  */
float KalmanAngle_GetAngle(const KalmanAngle_t *kf)
{
    return kf->angle;
}

/**
  * @brief  获取当前估计的陀螺仪零偏（°/s）
  */
float KalmanAngle_GetBias(const KalmanAngle_t *kf)
{
    return kf->bias;
}

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
