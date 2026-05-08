/*
 * calibration.c
 *
 *  FOC 校准模块实现
 *  5 项独立校准实验 + Flash 持久化 + 并发安全锁
 */

#include "calibration.h"
#include "main.h"
#include "ina240.h"
#include "mt6701.h"
#include "motor_hal.h"
#include "pi.h"
#include "svpwm.h"
#include "encoder_cache.h"
#include "comm_protocol.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

CalibParams_t g_calib;
uint8_t g_calib_mode = 0;

// ===== 并发安全 =====
static volatile uint8_t calib_busy = 0;
static volatile uint8_t calib_abort_flag = 0;

uint8_t CALIB_TryLock(void)
{
    if (calib_busy) return 0;
    calib_busy = 1;
    return 1;
}

void CALIB_Unlock(void) { calib_busy = 0; }
uint8_t CALIB_IsBusy(void) { return calib_busy; }

void CALIB_Abort(void) { calib_abort_flag = 1; }
uint8_t CALIB_IsAborted(void) { return calib_abort_flag; }

// ===== 辅助函数 =====

static inline float normalize_angle(float rad)
{
    while (rad >= 6.283185307f)  rad -= 6.283185307f;
    while (rad < 0.0f)           rad += 6.283185307f;
    return rad;
}

// 安全停止：先改 mode 阻止 ISR 调 CurrentCtrl_Run → 再停 PWM → 禁能
static void calib_safe_stop(Motor_t *motor)
{
    if (motor) {
        motor->mode = MOTOR_MODE_OFF;
        motor->use_virtual_angle = 0;
        Motor_StopPWM(motor);
    }
    Motor_Disable();
}

// 所有校准函数入口/出口宏（需在函数引用前定义）
#define CALIB_ENTRY() \
    do { \
        calib_abort_flag = 0; \
    } while(0)

#define CALIB_EXIT() \
    do { \
        CALIB_Unlock(); \
    } while(0)

// abort 时统一清理：停电机 + 恢复默认 PI + 解锁
static void calib_abort_cleanup(Motor_t *motor)
{
    calib_safe_stop(motor);
    if (motor) {
        PI_Init(&motor->id_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
        PI_Init(&motor->iq_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
    }
    printf("CALIB ABORTED\r\n");
    CALIB_EXIT();
}

// ===== CRC32 =====

static uint32_t calib_crc32(const CalibParams_t *params)
{
    const uint8_t *data = (const uint8_t *)params;
    size_t len = sizeof(CalibParams_t) - sizeof(uint32_t);
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

// ===== Flash 持久化 =====

uint8_t CALIB_FlashIsValid(const CalibParams_t *params)
{
    if (params->magic != CALIB_MAGIC) return 0;
    if (params->version != CALIB_VERSION) return 0;
    return (calib_crc32(params) == params->crc32) ? 1 : 0;
}

HAL_StatusTypeDef CALIB_FlashLoad(CalibParams_t *params)
{
    uint32_t *src = (uint32_t *)CALIB_FLASH_ADDR;
    uint32_t *dst = (uint32_t *)params;
    size_t words = (sizeof(CalibParams_t) + 3) / 4;
    for (size_t i = 0; i < words; i++) {
        dst[i] = src[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef CALIB_FlashSave(const CalibParams_t *params)
{
    CalibParams_t write_copy = *params;
    write_copy.crc32 = calib_crc32(&write_copy);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_1,
        .Page      = CALIB_FLASH_PAGE,
        .NbPages   = 1
    };
    uint32_t page_error;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }

    uint64_t *src64 = (uint64_t *)&write_copy;
    uint32_t addr = CALIB_FLASH_ADDR;
    size_t dwords = (sizeof(CalibParams_t) + 7) / 8;

    for (size_t i = 0; i < dwords; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src64[i]);
        if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }
        addr += 8;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

// ===== 应用校准参数 =====

void CALIB_ApplyToMotor(const CalibParams_t *params, uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return;
    Motor_t *m = &g_motor[motor_index];

    m->phase_comp = params->phase_comp[motor_index];
    m->ch_u = (INA240_Channel_t)params->ch_u_mapping[motor_index];
    m->ch_v = (INA240_Channel_t)params->ch_v_mapping[motor_index];
}

// ===== 上报 =====

void CALIB_PrintParams(const CalibParams_t *params)
{
    printf("=== CALIB PARAMS ===\r\n");
    printf("enc_dir:   M1=%+d M2=%+d\r\n", params->enc_direction[0], params->enc_direction[1]);
    printf("phase_comp: M1=%.3f M2=%.3f rad\r\n", params->phase_comp[0], params->phase_comp[1]);
    printf("zero_offs: %u %u %u %u\r\n",
           params->zero_offset[0], params->zero_offset[1],
           params->zero_offset[2], params->zero_offset[3]);
    printf("ch_u_map:  M1=%u M2=%u\r\n", params->ch_u_mapping[0], params->ch_u_mapping[1]);
    printf("ch_v_map:  M1=%u M2=%u\r\n", params->ch_v_mapping[0], params->ch_v_mapping[1]);
    printf("R_phase:   M1=%.3f M2=%.3f ohm\r\n", params->phase_resistance[0], params->phase_resistance[1]);
    printf("valid: %s\r\n", CALIB_FlashIsValid(params) ? "YES" : "NO");
}

void CALIB_ReportProgress(CalibFunc_t func, uint8_t motor_id, float progress)
{
    printf("CALIB[%d] M%d: %.0f%%\r\n", func, motor_id + 1, progress * 100.0f);
}

// ================================================================
//  实验 1: 电流采样零漂校准（全局，不区分电机）
//  安全等级：高 — 不涉及电机运转，仅读 ADC
// ================================================================

#define CALIB_OFFSET_SAMPLES    1000
#define CALIB_OFFSET_TRIM_PCT   10

void CALIB_CurrentOffset(void)
{
    CALIB_ENTRY();

    printf("=== 零漂校准 START ===\r\n");

    Motor_Disable();
    osDelay(200);

    static uint16_t raw_data[INA240_NUM_CHANNELS][CALIB_OFFSET_SAMPLES];

    for (uint32_t n = 0; n < CALIB_OFFSET_SAMPLES; n++)
    {
        if (CALIB_IsAborted()) {
            printf("CALIB ABORTED\r\n");
            CALIB_EXIT();
            return;
        }
        for (uint32_t ch = 0; ch < INA240_NUM_CHANNELS; ch++) {
            raw_data[ch][n] = adc_buffer[ch];
        }
        osDelay(1);
    }

    for (uint32_t ch = 0; ch < INA240_NUM_CHANNELS; ch++)
    {
        uint16_t *arr = raw_data[ch];
        for (uint32_t i = 0; i < CALIB_OFFSET_SAMPLES - 1; i++) {
            for (uint32_t j = 0; j < CALIB_OFFSET_SAMPLES - 1 - i; j++) {
                if (arr[j] > arr[j + 1]) {
                    uint16_t tmp = arr[j]; arr[j] = arr[j + 1]; arr[j + 1] = tmp;
                }
            }
        }

        uint32_t trim = CALIB_OFFSET_SAMPLES * CALIB_OFFSET_TRIM_PCT / 100;
        uint32_t trimmed_sum = 0;
        uint32_t trimmed_count = 0;
        for (uint32_t i = trim; i < CALIB_OFFSET_SAMPLES - trim; i++) {
            trimmed_sum += arr[i];
            trimmed_count++;
        }

        uint16_t offset = (uint16_t)(trimmed_sum / trimmed_count);
        float mean_f = (float)trimmed_sum / (float)trimmed_count;
        float var = 0.0f;
        for (uint32_t i = trim; i < CALIB_OFFSET_SAMPLES - trim; i++) {
            float d = (float)arr[i] - mean_f;
            var += d * d;
        }
        float stddev = sqrtf(var / (float)trimmed_count);

        g_calib.zero_offset[ch] = offset;
        INA240_SetZeroOffset((INA240_Channel_t)ch, offset);
        printf(" CH%lu: offset=%u noise=%.2f LSB\r\n", (unsigned long)ch, offset, stddev);
    }

    if (CALIB_FlashSave(&g_calib) == HAL_OK) {
        printf("=== 零漂校准 OK (saved) ===\r\n");
    } else {
        printf("=== 零漂校准 OK (flash FAILED) ===\r\n");
    }
    CALIB_EXIT();
}

// ================================================================
//  实验 2: 相线及电流传感器对应关系辨识
//  安全等级：中 — 小电压 DC 注入，电流 ~100mA
//  注意：PC14 共用，另一台电机被强停
// ================================================================

void CALIB_PhaseWireMap(Motor_t *motor)
{
    CALIB_ENTRY();
    if (motor == NULL) { CALIB_EXIT(); return; }

    uint8_t mid = motor->motor_id;

    printf("=== 相线映射校准 M%d START ===\r\n", mid + 1);

    Motor_Disable();
    osDelay(100);
    Motor_Enable();  // PC14=HIGH + TIM3 计数器使能（校准模式电机初始均停转，无需 Motor_StopPWM）
    Motor_StartPWM(motor);

    const float dc_center = 0.50f;
    const float dc_high   = 0.58f;

    // 激励 A 相 + 打印全部 4 通道读数
    float all_cur[INA240_NUM_CHANNELS];
    uint8_t phase_u_ch = 0xFF;
    float max_current;
    Motor_SetDuty(motor, dc_high, dc_center, dc_center);
    osDelay(50);  // EMA 时间常数 ~1.6ms, 50ms > 30τ 已完全稳定
    max_current = 0.0f;
    for (uint32_t ch = 0; ch < INA240_NUM_CHANNELS; ch++) {
        float cur = INA240_GetCurrentFast((INA240_Channel_t)ch);
        all_cur[ch] = cur;
        float abs_cur = cur > 0 ? cur : -cur;
        if (abs_cur > max_current) { max_current = abs_cur; phase_u_ch = (uint8_t)ch; }
    }
    printf(" Phase A: CH0=%.3f CH1=%.3f CH2=%.3f CH3=%.3fA → max=CH%lu\r\n",
           all_cur[0], all_cur[1], all_cur[2], all_cur[3], (unsigned long)phase_u_ch);
    Motor_SetDuty(motor, dc_center, dc_center, dc_center);
    osDelay(20);

    // 激励 B 相 + 打印全部 4 通道读数
    uint8_t phase_v_ch = 0xFF;
    Motor_SetDuty(motor, dc_center, dc_high, dc_center);
    osDelay(50);
    max_current = 0.0f;
    for (uint32_t ch = 0; ch < INA240_NUM_CHANNELS; ch++) {
        float cur = INA240_GetCurrentFast((INA240_Channel_t)ch);
        all_cur[ch] = cur;
        float abs_cur = cur > 0 ? cur : -cur;
        if (abs_cur > max_current) { max_current = abs_cur; phase_v_ch = (uint8_t)ch; }
    }
    printf(" Phase B: CH0=%.3f CH1=%.3f CH2=%.3f CH3=%.3fA → max=CH%lu\r\n",
           all_cur[0], all_cur[1], all_cur[2], all_cur[3], (unsigned long)phase_v_ch);
    Motor_SetDuty(motor, dc_center, dc_center, dc_center);

    if (phase_u_ch == 0xFF || phase_v_ch == 0xFF || phase_u_ch == phase_v_ch) {
        // 附加诊断：检查是否有任何通道有显著电流
        if (max_current < 0.050f) {
            printf("  (所有通道 < 50mA — 电机未连接或 PC14 未使能?)\r\n");
        }
        printf("=== 相线映射 FAILED (ambiguous) ===\r\n");
        calib_safe_stop(motor);
        CALIB_EXIT();
        return;
    }

    g_calib.ch_u_mapping[mid] = phase_u_ch;
    g_calib.ch_v_mapping[mid] = phase_v_ch;
    motor->ch_u = (INA240_Channel_t)phase_u_ch;
    motor->ch_v = (INA240_Channel_t)phase_v_ch;

    calib_safe_stop(motor);

    if (CALIB_FlashSave(&g_calib) == HAL_OK) {
        printf("=== 相线映射 M%d OK: U→CH%lu, V→CH%lu (saved) ===\r\n",
               mid + 1, (unsigned long)phase_u_ch, (unsigned long)phase_v_ch);
    } else {
        printf("=== 相线映射 M%d OK (flash FAILED) ===\r\n", mid + 1);
    }
    CALIB_EXIT();
}

// ================================================================
//  实验 3: 编码器方向匹配
//  安全等级：中 — 开环低压旋转
// ================================================================

void CALIB_EncoderDir(Motor_t *motor)
{
    CALIB_ENTRY();
    if (motor == NULL) { CALIB_EXIT(); return; }

    uint8_t mid = motor->motor_id;

    printf("=== 编码器方向 M%d START ===\r\n", mid + 1);

    Motor_Disable();
    osDelay(100);
    Motor_Enable();

    motor->mode = MOTOR_MODE_VOLTAGE_SINE;
    Motor_StartPWM(motor);

    const float Vm = 0.5f;
    const float freq = 0.5f;
    const float dt = 0.001f;
    const uint32_t steps = (uint32_t)(2.0f / dt);

    float last_mech = g_enc[mid].mech_angle;
    float accum_delta = 0.0f;

    for (uint32_t n = 0; n < steps; n++)
    {
        if (CALIB_IsAborted()) {
            calib_safe_stop(motor);
            printf("CALIB ABORTED\r\n");
            CALIB_EXIT();
            return;
        }
        float t = (float)n * dt;
        float theta = 2.0f * 3.14159265f * freq * t;
        float v_alpha =  Vm * sinf(theta);
        float v_beta  = -Vm * cosf(theta);
        SVPWM_SetVab(v_alpha, v_beta, motor);

        float cur_mech = g_enc[mid].mech_angle;
        float delta = cur_mech - last_mech;
        if (delta > 3.14159265f)  delta -= 6.283185307f;
        if (delta < -3.14159265f) delta += 6.283185307f;
        accum_delta += delta;
        last_mech = cur_mech;

        osDelay(1);
    }

    calib_safe_stop(motor);

    float abs_delta = accum_delta > 0.0f ? accum_delta : -accum_delta;
    if (abs_delta < 0.1f) {
        printf("=== 编码器方向 M%d FAILED: 电机未转动 (|delta|=%.3f) ===\r\n", mid + 1, abs_delta);
        printf(" 检查: Vm 是否过低, 电机是否卡死, 编码器是否正常\r\n");
        CALIB_EXIT();
        return;
    }
    int8_t dir = (accum_delta > 0.0f) ? 1 : -1;
    printf(" accum_delta=%.3f rad → enc_direction=%+d\r\n", accum_delta, dir);

    g_calib.enc_direction[mid] = dir;
    MT6701_SetEncDirection(mid, dir);

    if (CALIB_FlashSave(&g_calib) == HAL_OK) {
        printf("=== 编码器方向 M%d OK: enc_dir=%+d (saved) ===\r\n", mid + 1, dir);
    } else {
        printf("=== 编码器方向 M%d OK (flash FAILED) ===\r\n", mid + 1);
    }
    CALIB_EXIT();
}

// ================================================================
//  实验 4: 编码器零位校准（Iq 旋转 + Id 精锁）
//  安全等级：中 — 电流闭环，Iq=0.3A/Id=0.8A
//  ISR 交互：利用 ADC ISR 中的 CurrentCtrl_Run 作为电流控制引擎
//            任务写入 virtual_angle/id_ref，ISR 读取 (Cortex-M4 32-bit 原子安全)
// ================================================================

#define CALIB_ZERO_IQ       0.3f
#define CALIB_ZERO_ID_LOCK  0.8f
#define CALIB_ZERO_FREQ     0.5f
#define CALIB_ZERO_DURATION 2.0f
#define CALIB_ZERO_LOCK_MS  500
#define CALIB_ZERO_SAMPLES  5

void CALIB_EncoderOffset(Motor_t *motor)
{
    CALIB_ENTRY();
    if (motor == NULL) { CALIB_EXIT(); return; }

    uint8_t mid = motor->motor_id;

    printf("=== 编码器零位校准 M%d START ===\r\n", mid + 1);

    Motor_Disable();
    osDelay(100);
    Motor_Enable();

    motor->mode = MOTOR_MODE_CURRENT_LOOP;
    motor->use_virtual_angle = 1;
    motor->virtual_angle = 0.0f;
    motor->id_ref = 0.0f;
    motor->iq_ref = 0.0f;

    // 临时低带宽 PI
    PI_Init(&motor->id_pi, 0.2f, 2.0f, 5.0f, -5.0f);
    PI_Init(&motor->iq_pi, 0.2f, 2.0f, 5.0f, -5.0f);

    Motor_StartPWM(motor);
    osDelay(50);

    // Phase 1 — Iq 旋转 (ISR 读取 virtual_angle 执行电流闭环)
    printf(" Phase 1: Iq rotation @%.1fHz, Iq=%.1fA (ISR-driven)\r\n", CALIB_ZERO_FREQ, CALIB_ZERO_IQ);
    motor->iq_ref = CALIB_ZERO_IQ;
    motor->id_ref = 0.0f;

    const float dt = 0.001f;
    uint32_t steps = (uint32_t)(CALIB_ZERO_DURATION / dt);

    for (uint32_t n = 0; n < steps; n++)
    {
        if (CALIB_IsAborted()) { calib_abort_cleanup(motor); return; }
        float t = (float)n * dt;
        motor->virtual_angle = normalize_angle(2.0f * 3.14159265f * CALIB_ZERO_FREQ * t);
        osDelay(1);
    }

    // Phase 2 — Id 精锁（分包延时，每 50ms 检查 abort）
    printf(" Phase 2: Id lock @%.1fA, %lums\r\n", CALIB_ZERO_ID_LOCK, (unsigned long)CALIB_ZERO_LOCK_MS);
    motor->iq_ref = 0.0f;
    motor->id_ref = CALIB_ZERO_ID_LOCK;
    motor->virtual_angle = 0.0f;
    for (uint32_t lock_ms = 0; lock_ms < CALIB_ZERO_LOCK_MS; lock_ms += 50) {
        osDelay(50);
        if (CALIB_IsAborted()) { calib_abort_cleanup(motor); return; }
    }

    // Phase 3 — 读偏移
    printf(" Phase 3: read encoder offset\r\n");
    float offset_sum = 0.0f;
    for (uint32_t i = 0; i < CALIB_ZERO_SAMPLES; i++)
    {
        osDelay(20);
        float enc_elec = g_enc[mid].elec_angle;
        float this_offset = normalize_angle(-enc_elec);
        offset_sum += this_offset;
        printf("  sample %lu: enc_elec=%.3f → offset=%.3f rad\r\n",
               (unsigned long)(i + 1), enc_elec, this_offset);
    }
    float phase_comp = offset_sum / (float)CALIB_ZERO_SAMPLES;

    // Phase 4 — 验证（Id 锁轴，phase_comp 正确时 d 轴对齐 → 电机不转）
    printf(" Phase 4: verification (Id=%.1fA hold)\r\n", CALIB_ZERO_IQ);
    motor->use_virtual_angle = 0;
    motor->phase_comp = phase_comp;
    motor->id_ref = CALIB_ZERO_IQ;  // 纯 d 轴电流 → 对齐力矩，不应旋转
    motor->iq_ref = 0.0f;

    // 分包延时 + abort 检查
    for (uint32_t hold_ms = 0; hold_ms < 500; hold_ms += 100) {
        osDelay(100);
        if (CALIB_IsAborted()) { calib_abort_cleanup(motor); return; }
    }

    float start_angle = g_enc[mid].mech_angle;
    osDelay(200);
    if (CALIB_IsAborted()) { calib_abort_cleanup(motor); return; }
    float end_angle = g_enc[mid].mech_angle;
    float drift = end_angle - start_angle;
    if (drift > 3.14159265f)  drift -= 6.283185307f;
    if (drift < -3.14159265f) drift += 6.283185307f;
    float drift_rpm = drift / 0.2f / 6.283185307f * 60.0f;
    printf("  drift: %.3f rad (%.1f RPM)\r\n", drift, drift_rpm);

    int pass = (fabsf(drift_rpm) < 30.0f) ? 1 : 0;

    calib_safe_stop(motor);
    g_calib.phase_comp[mid] = phase_comp;

    // 恢复 PI
    PI_Init(&motor->id_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
    PI_Init(&motor->iq_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);

    if (CALIB_FlashSave(&g_calib) == HAL_OK) {
        printf("=== 编码器零位 M%d %s: phase_comp=%.3f rad (%.1f deg) (saved) ===\r\n",
               mid + 1, pass ? "OK" : "WARN", phase_comp, phase_comp * 57.29578f);
    } else {
        printf("=== 编码器零位 M%d %s (flash FAILED) ===\r\n", mid + 1, pass ? "OK" : "WARN");
    }
    CALIB_EXIT();
}

// ================================================================
//  实验 5: 电机参数辨识（相电阻）
//  安全等级：中 — 电流闭环 Id 注入
// ================================================================

void CALIB_MotorParams(Motor_t *motor)
{
    CALIB_ENTRY();
    if (motor == NULL) { CALIB_EXIT(); return; }

    uint8_t mid = motor->motor_id;

    printf("=== 电机参数辨识 M%d START ===\r\n", mid + 1);

    Motor_Disable();
    osDelay(100);
    Motor_Enable();

    motor->mode = MOTOR_MODE_CURRENT_LOOP;
    motor->use_virtual_angle = 1;
    motor->virtual_angle = 0.0f;
    motor->iq_ref = 0.0f;

    PI_Init(&motor->id_pi, 0.1f, 1.0f, 3.0f, -3.0f);
    PI_Init(&motor->iq_pi, 0.1f, 1.0f, 3.0f, -3.0f);

    Motor_StartPWM(motor);
    osDelay(100);

    const float I1 = 0.3f;
    const float I2 = 0.6f;

    motor->id_ref = I1;
    for (uint32_t settle = 0; settle < 500; settle += 50) {
        osDelay(50);
        if (CALIB_IsAborted()) { calib_abort_cleanup(motor); return; }
    }
    float sum_vd = 0.0f;
    for (int i = 0; i < 100; i++) { sum_vd += motor->vd; osDelay(5); }
    float Vd1 = sum_vd / 100.0f;
    printf(" Id=%.1fA → Vd=%.3fV\r\n", I1, Vd1);

    motor->id_ref = I2;
    for (uint32_t settle = 0; settle < 500; settle += 50) {
        osDelay(50);
        if (CALIB_IsAborted()) { calib_abort_cleanup(motor); return; }
    }
    sum_vd = 0.0f;
    for (int i = 0; i < 100; i++) { sum_vd += motor->vd; osDelay(5); }
    float Vd2 = sum_vd / 100.0f;
    printf(" Id=%.1fA → Vd=%.3fV\r\n", I2, Vd2);

    motor->id_ref = 0.0f;
    calib_safe_stop(motor);

    float R_d = (Vd2 - Vd1) / (I2 - I1);
    // 幅值不变 Clarke: Ia=Id, Ib=-Id/2, Ic=-Id/2 → 铜损 1.5*Id²*R
    // dq 功率 1.5*Vd*Id → 能量守恒 Vd=Id*R → R_phase = R_d
    float R_phase = R_d;

    g_calib.phase_resistance[mid] = R_phase;

    PI_Init(&motor->id_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);
    PI_Init(&motor->iq_pi, 0.5f, 20.0f, FOC_VBUS, -FOC_VBUS);

    if (CALIB_FlashSave(&g_calib) == HAL_OK) {
        printf("=== 电机参数 M%d OK: R_phase=%.3f ohm (saved) ===\r\n", mid + 1, R_phase);
    } else {
        printf("=== 电机参数 M%d OK (flash FAILED) ===\r\n", mid + 1);
    }
    CALIB_EXIT();
}
