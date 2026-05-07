/*
 * calibration.h
 *
 *  FOC 校准模块 — 5 项独立校准实验，结果持久化到 MCU 内部 Flash
 *
 *  并发安全: 同一时刻只能有一个校准实验运行 (calib_busy 锁)
 *
 *  使用方式：在串口 CLI 中输入命令触发对应实验
 *    c1~c5: M1 校准 1~5
 *    d1~d5: M2 校准 1~5
 *    s:     打印当前校准参数
 *    q:     中止正在进行的校准
 *
 *  注意: PC14 由两个 MP6536 共用，校准任一台电机时另一台会被强停。
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include "foc.h"

#define CALIB_MAGIC          0xCA1B0001u
#define CALIB_VERSION        1
#define CALIB_FLASH_ADDR     0x0801F800u   // STM32G431CB Flash 最后一页
#define CALIB_FLASH_PAGE     63
#define CALIB_PAGE_SIZE      0x800u

typedef enum {
    CALIB_FUNC_CURRENT_OFFSET  = 0,
    CALIB_FUNC_PHASE_WIRE_MAP  = 1,
    CALIB_FUNC_ENCODER_DIR     = 2,
    CALIB_FUNC_ENCODER_OFFSET  = 3,
    CALIB_FUNC_MOTOR_PARAMS    = 4,
} CalibFunc_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t  reserved[2];

    int8_t   enc_direction[2];
    float    phase_comp[2];
    uint16_t zero_offset[4];
    uint8_t  ch_u_mapping[2];
    uint8_t  ch_v_mapping[2];
    float    phase_resistance[2];

    uint32_t crc32;
} CalibParams_t;

extern CalibParams_t g_calib;

// ===== Flash 持久化 =====
HAL_StatusTypeDef CALIB_FlashSave(const CalibParams_t *params);
HAL_StatusTypeDef CALIB_FlashLoad(CalibParams_t *params);
uint8_t          CALIB_FlashIsValid(const CalibParams_t *params);

// ===== 并发控制 =====
uint8_t CALIB_TryLock(void);
void    CALIB_Unlock(void);

// ===== 5 项校准实验 =====
void CALIB_CurrentOffset(void);
void CALIB_PhaseWireMap(Motor_t *motor);
void CALIB_EncoderDir(Motor_t *motor);
void CALIB_EncoderOffset(Motor_t *motor);
void CALIB_MotorParams(Motor_t *motor);

void CALIB_ApplyToMotor(const CalibParams_t *params, uint8_t motor_index);

void    CALIB_Abort(void);
uint8_t CALIB_IsAborted(void);

void CALIB_PrintParams(const CalibParams_t *params);
void CALIB_ReportProgress(CalibFunc_t func, uint8_t motor_id, float progress);

#endif /* CALIBRATION_H */
