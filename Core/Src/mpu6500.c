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
  * @brief  SPI 发送一字节（半双工，用于写寄存器）
  *         修复 MPU6500 SPI 缺陷: 全双工 TransmitReceive 会导致从机
  *         把 MOSI 上的 0x00 当成新命令, 破坏读取数据。
  * @param  tx: 发送字节
  */
static void spi_send(uint8_t tx)
{
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY);
}

/**
  * @brief  读取 MPU6500 寄存器（单字节）
  *         半双工: 先发地址 → 再只收数据, MOSI 不发 0x00
  */
static int8_t read_reg(uint8_t reg, uint8_t *data)
{
    uint8_t addr = reg | 0x80;
    cs_select();
    HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, 1, HAL_MAX_DELAY);
    cs_deselect();
    return 0;
}

/**
  * @brief  写入 MPU6500 寄存器
  */
static int8_t write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg & 0x7F;
    buf[1] = data;
    cs_select();
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
    cs_deselect();
    return 0;
}

/**
  * @brief  连续读取多个寄存器（突发模式，地址自增）
  *         半双工修复: 只发地址, 然后只收数据, 避免 MOSI 干扰
  */
static int8_t read_burst(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = reg | 0x80;
    cs_select();
    HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, len, HAL_MAX_DELAY);
    cs_deselect();
    return 0;
}

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/**
  * @brief  读取 MPU6500 寄存器（调试用）
  */
int8_t MPU6500_ReadReg(uint8_t reg, uint8_t *data)
{
    return read_reg(reg, data);
}

/**
  * @brief  初始化 MPU6500
  * @note   完整复位 + 唤醒 + 默认配置
  * @retval 0=成功, -1=检测失败
  */
int8_t MPU6500_Init(void)
{
    // 检查芯片 ID
    if (MPU6500_CheckID() != 0)
        return -1;

    // 步骤1：完整设备复位
    write_reg(MPU6500_REG_PWR_MGMT_1, 0x80);  // DEVICE_RESET=1
    HAL_Delay(50);

    // 步骤2：退出休眠，选择陀螺仪 PLL 作为时钟源（比内部振荡器更稳定）
    //   bit6=0 (SLEEP=0)
    //   bit2:0=001 (CLKSEL=PLL with gyro X reference)
    write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    // 步骤3：配置数字低通滤波器
    //   CONFIG (0x1A): DLPF_CFG=2 → 陀螺仪带宽 92Hz, 延迟 3.9ms
    write_reg(MPU6500_REG_CONFIG, 0x02);
    //   ACCEL_CONFIG2 (0x1D): DLPF_CFG=2 → 加速度计带宽 92Hz
    write_reg(0x1D, 0x02);

    // 步骤4：设置量程
    //   加速度 ±4g（平衡车工作时会有一定倾斜，±2g 可能饱和）
    //   陀螺仪 ±250°/s（平衡车角速度通常不会超过 250°/s）
    MPU6500_SetAccelRange(MPU6500_ACCEL_RANGE_4G);
    MPU6500_SetGyroRange(MPU6500_GYRO_RANGE_250DPS);

    // 步骤5：采样率配置
    //   SMPLRT_DIV=0 → 采样率 = 1kHz / (0+1) = 1kHz
    write_reg(MPU6500_REG_SMPLRT_DIV, 0x00);

    // 等待传感器输出稳定
    HAL_Delay(50);

    return 0;
}

/**
  * @brief  检测芯片是否在线
  * @retval 0=正常, -1=未检测到
  */
int8_t MPU6500_CheckID(void)
{
    uint8_t whoami = 0;
    read_reg(MPU6500_REG_WHO_AM_I, &whoami);
    return (whoami == MPU6500_WHO_AM_I_VAL) ? 0 : -1;
}

/**
  * @brief  设置加速度计量程
  */
int8_t MPU6500_SetAccelRange(MPU6500_AccelRange_t range)
{
    current_accel_range = range;
    return write_reg(MPU6500_REG_ACCEL_CONFIG, (uint8_t)range);
}

/**
  * @brief  设置陀螺仪量程
  */
int8_t MPU6500_SetGyroRange(MPU6500_GyroRange_t range)
{
    current_gyro_range = range;
    return write_reg(MPU6500_REG_GYRO_CONFIG, (uint8_t)range);
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

    gyro->x = gyro_raw_to_dps(rx, current_gyro_range);
    gyro->y = gyro_raw_to_dps(ry, current_gyro_range);
    gyro->z = gyro_raw_to_dps(rz, current_gyro_range);

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
