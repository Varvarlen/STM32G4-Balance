/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6500.h
  * @brief   MPU6500 六轴传感器 SPI 驱动
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MPU6500_H__
#define __MPU6500_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/** @brief MPU6500 WHO_AM_I 返回值为 0x70 */
#define MPU6500_WHO_AM_I_VAL         0x70U

/** @brief 寄存器地址 */
#define MPU6500_REG_SMPLRT_DIV       0x19U
#define MPU6500_REG_CONFIG           0x1AU
#define MPU6500_REG_GYRO_CONFIG      0x1BU
#define MPU6500_REG_ACCEL_CONFIG     0x1CU
#define MPU6500_REG_ACCEL_CONFIG2    0x1DU
#define MPU6500_REG_INT_PIN_CFG      0x37U
#define MPU6500_REG_INT_ENABLE       0x38U
#define MPU6500_REG_ACCEL_XOUT_H     0x3BU
#define MPU6500_REG_ACCEL_XOUT_L     0x3CU
#define MPU6500_REG_ACCEL_YOUT_H     0x3DU
#define MPU6500_REG_ACCEL_YOUT_L     0x3EU
#define MPU6500_REG_ACCEL_ZOUT_H     0x3FU
#define MPU6500_REG_ACCEL_ZOUT_L     0x40U
#define MPU6500_REG_TEMP_OUT_H       0x41U
#define MPU6500_REG_TEMP_OUT_L       0x42U
#define MPU6500_REG_GYRO_XOUT_H      0x43U
#define MPU6500_REG_GYRO_XOUT_L      0x44U
#define MPU6500_REG_GYRO_YOUT_H      0x45U
#define MPU6500_REG_GYRO_YOUT_L      0x46U
#define MPU6500_REG_GYRO_ZOUT_H      0x47U
#define MPU6500_REG_GYRO_ZOUT_L      0x48U
#define MPU6500_REG_SIG_PATH_RESET   0x68U
#define MPU6500_REG_USER_CTRL        0x6AU
#define MPU6500_REG_PWR_MGMT_1       0x6BU
#define MPU6500_REG_PWR_MGMT_2       0x6CU
#define MPU6500_REG_WHO_AM_I         0x75U

/** @brief 加速度计量程 */
typedef enum {
    MPU6500_ACCEL_RANGE_2G  = 0x00U,    /**< ±2g  */
    MPU6500_ACCEL_RANGE_4G  = 0x08U,    /**< ±4g  */
    MPU6500_ACCEL_RANGE_8G  = 0x10U,    /**< ±8g  */
    MPU6500_ACCEL_RANGE_16G = 0x18U,    /**< ±16g */
} MPU6500_AccelRange_t;

/** @brief 陀螺仪量程 */
typedef enum {
    MPU6500_GYRO_RANGE_250DPS  = 0x00U, /**< ±250 °/s  */
    MPU6500_GYRO_RANGE_500DPS  = 0x08U, /**< ±500 °/s  */
    MPU6500_GYRO_RANGE_1000DPS = 0x10U, /**< ±1000 °/s */
    MPU6500_GYRO_RANGE_2000DPS = 0x18U, /**< ±2000 °/s */
} MPU6500_GyroRange_t;

/** @brief 加速度计数据（g） */
typedef struct {
    float x;
    float y;
    float z;
} MPU6500_Accel_t;

/** @brief 陀螺仪数据（°/s） */
typedef struct {
    float x;
    float y;
    float z;
} MPU6500_Gyro_t;

/** @brief CS 引脚定义（PC6） */
#define MPU6500_CS_Pin          GPIO_PIN_6
#define MPU6500_CS_GPIO_Port    GPIOC

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/** @brief 读取 MPU6500 寄存器（调试用） */
int8_t MPU6500_ReadReg(uint8_t reg, uint8_t *data);

/** @brief 初始化 MPU6500 */
int8_t MPU6500_Init(void);

/** @brief 检测芯片是否在线（读 WHO_AM_I） */
int8_t MPU6500_CheckID(void);

/** @brief 设置加速度计量程 */
int8_t MPU6500_SetAccelRange(MPU6500_AccelRange_t range);

/** @brief 设置陀螺仪量程 */
int8_t MPU6500_SetGyroRange(MPU6500_GyroRange_t range);

/** @brief 读取加速度计（g） */
int8_t MPU6500_ReadAccel(MPU6500_Accel_t *accel);

/** @brief 读取陀螺仪（°/s） */
int8_t MPU6500_ReadGyro(MPU6500_Gyro_t *gyro);

/** @brief 读取温度（°C） */
int8_t MPU6500_ReadTemperature(float *temp);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MPU6500_H__ */
