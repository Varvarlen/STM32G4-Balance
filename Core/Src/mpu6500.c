/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6500.c
  * @brief   MPU6500 六轴传感器 SPI 驱动
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mpu6500.h"
#include "spi.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */

// 当前量程配置（用于灵敏度换算）
static MPU6500_AccelRange_t current_accel_range = MPU6500_ACCEL_RANGE_2G;
static MPU6500_GyroRange_t  current_gyro_range  = MPU6500_GYRO_RANGE_250DPS;

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  CS 片选控制
  */
static void cs_select(void)
{
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
}

static void cs_deselect(void)
{
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  读取 MPU6500 寄存器（单字节）
  *         一次 SPI 事务: [addr|0x80] [0xFF] → 收 [garbage] [data]
  */
static int8_t read_reg(uint8_t reg, uint8_t *data)
{
    uint8_t tx[2] = {reg | 0x80, 0xFF};
    uint8_t rx[2];
    cs_select();
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 5) != HAL_OK) {
        cs_deselect();
        return -1;
    }
    cs_deselect();
    *data = rx[1];
    return 0;
}

/**
  * @brief  写入 MPU6500 寄存器
  */
static int8_t write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & 0x7F, data};
    cs_select();
    if (HAL_SPI_Transmit(&hspi2, tx, 2, 5) != HAL_OK) {
        cs_deselect();
        return -1;
    }
    cs_deselect();
    return 0;
}

/**
  * @brief  连续读取多个寄存器（突发模式，地址自增）
  *         一次连续 SPI 事务: [addr|0x80] [0xFF×N] → 收 [garbage] [data×N]
  */
static int8_t read_burst(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[15];  // 1 address + max 14 data bytes (accel+temp+gyro)
    uint8_t rx[15];
    tx[0] = reg | 0x80;
    for (uint16_t i = 1; i <= len; i++) tx[i] = 0xFF;
    cs_select();
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + 1, 5) != HAL_OK) {
        cs_deselect();
        return -1;
    }
    cs_deselect();
    for (uint16_t i = 0; i < len; i++) data[i] = rx[i + 1];  // skip address-phase garbage
    return 0;
}

/**
  * @brief  SPI2 速度切换 — 配置寄存器 ≤1MHz, 传感器寄存器 ≤20MHz
  *         MPU6500 数据手册: 配置寄存器 SPI 时钟上限 1MHz
  *         动态切换避免低速拖慢 1kHz 传感器读取
  */
static void mpu6500_spi_set_prescaler(uint32_t prescaler)
{
    hspi2.Init.BaudRatePrescaler = prescaler;
    HAL_SPI_Init(&hspi2);
}

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/**
  * @brief  读取 MPU6500 寄存器（调试用）
  * @note   保守使用低速，适应任意寄存器
  */
int8_t MPU6500_ReadReg(uint8_t reg, uint8_t *data)
{
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);
    int8_t ret = read_reg(reg, data);
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16);
    return ret;
}

/**
  * @brief  初始化 MPU6500
  * @note   标准流程: 复位→信号路径复位→禁用I2C→配置→唤醒
  *         配置寄存器 SPI ≤1MHz, 传感器寄存器 SPI ≤20MHz
  * @retval 0=成功, -1=检测失败
  */
int8_t MPU6500_Init(void)
{
    // 切换到低速用于配置寄存器访问 (≤1MHz)
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);

    // 步骤1：完整设备复位，等待振荡器稳定
    write_reg(MPU6500_REG_PWR_MGMT_1, 0x80);  // DEVICE_RESET=1
    HAL_Delay(100);

    // 步骤2：重置全部信号路径 (gyro+accel+temp)
    write_reg(MPU6500_REG_SIG_PATH_RESET, 0x07);
    HAL_Delay(100);

    // 步骤3：检查芯片 ID
    uint8_t whoami = 0;
    read_reg(MPU6500_REG_WHO_AM_I, &whoami);
    printf("[MPU6500] WHO_AM_I=0x%02X (expected 0x%02X)\r\n", whoami, MPU6500_WHO_AM_I_VAL);
    if (whoami != MPU6500_WHO_AM_I_VAL) {
        mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16);
        return -1;
    }

    // 步骤4：禁用 I2C 接口，确保纯 SPI 模式
    write_reg(MPU6500_REG_USER_CTRL, 0x10);  // I2C_IF_DIS=1

    // 步骤5：使能所有轴（确保 PWR_MGMT_2 为默认值）
    write_reg(MPU6500_REG_PWR_MGMT_2, 0x00);

    // 步骤6：配置数字低通滤波器 + 量程 + 采样率（全部在低速下）
    // 先做直接 write_reg，再做包裹函数调用（它们内部会切速度，但最终回到高速）
    // 因此在每个包裹函数调用之后重新切回低速
    write_reg(MPU6500_REG_CONFIG, 0x02);         // DLPF_CFG=2, 陀螺仪带宽 92Hz
    write_reg(MPU6500_REG_ACCEL_CONFIG2, 0x02);   // 加速度计 DLPF_CFG=2
    write_reg(MPU6500_REG_SMPLRT_DIV, 0x00);      // 采样率=1kHz/(0+1)=1kHz
    MPU6500_SetAccelRange(MPU6500_ACCEL_RANGE_4G); // 内部切低→写→切高
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);
    MPU6500_SetGyroRange(MPU6500_GYRO_RANGE_250DPS); // 内部切低→写→切高
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);

    // 步骤7：退出休眠，选择陀螺仪 PLL 作为时钟源
    write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);  // SLEEP=0, CLKSEL=PLL gyro X
    HAL_Delay(10);  // PLL 锁定

    // 等待传感器输出稳定
    HAL_Delay(50);

    // 切换到高速用于传感器数据读取 (≤20MHz)
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16);

    return 0;
}

/**
  * @brief  检测芯片是否在线
  * @retval 0=正常, -1=未检测到
  */
int8_t MPU6500_CheckID(void)
{
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);
    uint8_t whoami = 0;
    read_reg(MPU6500_REG_WHO_AM_I, &whoami);
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16);
    return (whoami == MPU6500_WHO_AM_I_VAL) ? 0 : -1;
}

/**
  * @brief  设置加速度计量程
  */
int8_t MPU6500_SetAccelRange(MPU6500_AccelRange_t range)
{
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);
    current_accel_range = range;
    int8_t ret = write_reg(MPU6500_REG_ACCEL_CONFIG, (uint8_t)range);
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16);
    return ret;
}

/**
  * @brief  设置陀螺仪量程
  */
int8_t MPU6500_SetGyroRange(MPU6500_GyroRange_t range)
{
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);
    current_gyro_range = range;
    int8_t ret = write_reg(MPU6500_REG_GYRO_CONFIG, (uint8_t)range);
    mpu6500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16);
    return ret;
}

/**
  * @brief  将加速度计原始值转换为 g
  */
static float accel_raw_to_g(int16_t raw, MPU6500_AccelRange_t range)
{
    static const float lsb_per_g[] = {
        16384.0f,   // ±2g
        8192.0f,    // ±4g
        4096.0f,    // ±8g
        2048.0f     // ±16g
    };
    return (float)raw / lsb_per_g[range >> 3];
}

/**
  * @brief  将陀螺仪原始值转换为 °/s
  */
static float gyro_raw_to_dps(int16_t raw, MPU6500_GyroRange_t range)
{
    static const float lsb_per_dps[] = {
        131.0f,     // ±250°/s
        65.5f,      // ±500°/s
        32.8f,      // ±1000°/s
        16.4f       // ±2000°/s
    };
    return (float)raw / lsb_per_dps[range >> 3];
}

/**
  * @brief  读取加速度计（g）
  */
int8_t MPU6500_ReadAccel(MPU6500_Accel_t *accel)
{
    uint8_t buf[6];
    if (read_burst(MPU6500_REG_ACCEL_XOUT_H, buf, 6) != 0)
        return -1;

    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);

    // 一次性诊断: 打印原始 hex 值, 验证 SPI burst 读取是否正确
    static uint8_t diag_done = 0;
    if (!diag_done) {
        diag_done = 1;
        printf("[MPU6500] ACCEL raw hex: XH=%02X XL=%02X YH=%02X YL=%02X ZH=%02X ZL=%02X\r\n",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        printf("[MPU6500] ACCEL raw int: X=%d Y=%d Z=%d\r\n", rx, ry, rz);
        printf("[MPU6500] ACCEL g: X=%.4f Y=%.4f Z=%.4f (range=%d lsb/g=%.0f)\r\n",
               accel_raw_to_g(rx, current_accel_range),
               accel_raw_to_g(ry, current_accel_range),
               accel_raw_to_g(rz, current_accel_range),
               current_accel_range, 16384.0f / (1 << (current_accel_range >> 3)));
    }

    accel->x = accel_raw_to_g(rx, current_accel_range);
    accel->y = accel_raw_to_g(ry, current_accel_range);
    accel->z = accel_raw_to_g(rz, current_accel_range);

    return 0;
}

/**
  * @brief  读取陀螺仪（°/s）
  */
int8_t MPU6500_ReadGyro(MPU6500_Gyro_t *gyro)
{
    uint8_t buf[6];
    if (read_burst(MPU6500_REG_GYRO_XOUT_H, buf, 6) != 0)
        return -1;

    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);

    // 一次性诊断
    static uint8_t gyro_diag_done = 0;
    if (!gyro_diag_done) {
        gyro_diag_done = 1;
        printf("[MPU6500] GYRO raw hex: XH=%02X XL=%02X YH=%02X YL=%02X ZH=%02X ZL=%02X\r\n",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        printf("[MPU6500] GYRO dps: X=%.2f Y=%.2f Z=%.2f\r\n",
               gyro_raw_to_dps(rx, current_gyro_range),
               gyro_raw_to_dps(ry, current_gyro_range),
               gyro_raw_to_dps(rz, current_gyro_range));
    }

    gyro->x = gyro_raw_to_dps(rx, current_gyro_range);
    gyro->y = gyro_raw_to_dps(ry, current_gyro_range);
    gyro->z = gyro_raw_to_dps(rz, current_gyro_range);

    return 0;
}

/**
  * @brief  一次 CS 事务读取加速度计+陀螺仪（14字节连续burst）
  *         从 ACCEL_XOUT_H(0x3B) 读到 GYRO_ZOUT_L(0x48)
  *         单次 CS 事务保证数据一致性, 避免两次读取间被 PWM 噪声干扰
  */
int8_t MPU6500_ReadAll(MPU6500_Accel_t *accel, MPU6500_Gyro_t *gyro)
{
    uint8_t buf[14];
    if (read_burst(MPU6500_REG_ACCEL_XOUT_H, buf, 14) != 0)
        return -1;

    // 字节布局: [0:5]=accel [6:7]=temp [8:13]=gyro
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    accel->x = accel_raw_to_g(ax, current_accel_range);
    accel->y = accel_raw_to_g(ay, current_accel_range);
    accel->z = accel_raw_to_g(az, current_accel_range);

    gyro->x  = gyro_raw_to_dps(gx, current_gyro_range);
    gyro->y  = gyro_raw_to_dps(gy, current_gyro_range);
    gyro->z  = gyro_raw_to_dps(gz, current_gyro_range);

    return 0;
}

/**
  * @brief  读取温度（°C）
  * @note   Temp = (raw / 333.87) + 21.0
  *         灵敏度 333.87 LSB/°C, 21°C 时输出 0 LSB
  */
int8_t MPU6500_ReadTemperature(float *temp)
{
    uint8_t buf[2];
    if (read_burst(MPU6500_REG_TEMP_OUT_H, buf, 2) != 0)
        return -1;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    *temp = (float)raw / 333.87f + 21.0f;
    return 0;
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
