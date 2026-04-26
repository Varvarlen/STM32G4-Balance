/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kf_angle.h
  * @brief   一维卡尔曼滤波器（含陀螺仪零偏估计）
  *          State = [倾角, 陀螺仪零偏]
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __KF_ANGLE_H__
#define __KF_ANGLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/** @brief 卡尔曼滤波器实例 */
typedef struct {
    /* 状态 */
    float angle;        /**< 估计倾角（°） */
    float bias;         /**< 陀螺仪零偏（°/s） */

    /* 协方差矩阵 P (2x2) */
    float P[2][2];

    /* 噪声参数（需根据实际传感器调参） */
    float Q_angle;      /**< 角度过程噪声 */
    float Q_bias;       /**< 零偏过程噪声 */
    float R_measure;    /**< 加速度计观测噪声 */
} KalmanAngle_t;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/** @brief 初始化卡尔曼滤波器
  * @param  kf       滤波器实例指针
  * @param  init_angle  初始倾角（°）
  * @param  qa       角度过程噪声（默认 0.001f）
  * @param  qb       零偏过程噪声（默认 0.003f）
  * @param  rm       加速度计观测噪声（默认 0.03f）
  */
void KalmanAngle_Init(KalmanAngle_t *kf, float init_angle,
                      float qa, float qb, float rm);

/** @brief 预测步骤（使用陀螺仪积分）
  * @param  kf          滤波器实例
  * @param  gyro_rate   陀螺仪角速度（°/s）
  * @param  dt          上次预测以来的时间（s）
  */
void KalmanAngle_Predict(KalmanAngle_t *kf, float gyro_rate, float dt);

/** @brief 更新步骤（使用加速度计观测）
  * @param  kf           滤波器实例
  * @param  accel_angle  加速度计计算的倾角（°）
  */
void KalmanAngle_Update(KalmanAngle_t *kf, float accel_angle);

/** @brief 获取当前估计倾角（°） */
float KalmanAngle_GetAngle(const KalmanAngle_t *kf);

/** @brief 获取当前估计陀螺仪零偏（°/s） */
float KalmanAngle_GetBias(const KalmanAngle_t *kf);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __KF_ANGLE_H__ */
