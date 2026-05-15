#include "cli.h"
#include "cli_parser.h"
#include "comm.h"
#include "foc.h"
#include "motor_hal.h"
#include "speed_ctrl.h"
#include "speed_capture.h"
#include "pos_ctrl.h"
#include "calibration.h"
#include "debug_capture.h"
#include "buzzer.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 全局实例（供 FreeRTOS 状态机访问）
LoadTest_t g_load_test;
StepTest_t g_step_test;

// 遥测开关 — 默认关闭
static uint8_t g_telem_enabled = 0;

// 外部引用
extern SpeedCtrl_t g_speed[2];
extern PosCtrl_t g_pos[2];
extern uint8_t g_test_mode;
extern uint8_t g_calib_mode;

void CLI_Init(void)
{
    g_telem_enabled = 0;
}

uint8_t CLI_TelemetryEnabled(void)
{
    return g_telem_enabled;
}

// ===== CLI 辅助函数 =====

static void CMD_Help(void)
{
    printf("\r\n=== STATUS ===\r\n");
    for (int i = 0; i < 2; i++) {
        const char *mode_str;
        if (g_pos[i].active) mode_str = "POS";
        else if (g_motor[i].speed_mode) mode_str = "SPEED";
        else mode_str = "CURRENT";
        if (g_pos[i].active) {
            float pos_deg = g_speed[i].pos_est * 57.29578f;
            float ref_deg = g_pos[i].pos_ref * 57.29578f;
            printf("M%d %s ref=%.1f° pos=%.1f° iq=%.3fA\r\n",
                   i+1, mode_str, ref_deg, pos_deg, g_motor[i].iq);
        } else if (g_motor[i].speed_mode) {
            printf("M%d %s ref=%.0fRPM fb=%.0fRPM iq=%.3fA\r\n",
                   i+1, mode_str, g_speed[i].speed_ref, g_speed[i].speed_fb, g_motor[i].iq);
        } else {
            printf("M%d %s iq_ref=%.3fA iq=%.3fA\r\n",
                   i+1, mode_str, g_motor[i].iq_ref, g_motor[i].iq);
        }
        printf("  Speed PI: Kp=%.3f Ki=%.3f | Current PI: Kp=%.1f Ki=%.0f | Pos Kp=%.0f\r\n",
               g_speed[i].kp, g_speed[i].ki, g_motor[i].iq_pi.kp, g_motor[i].iq_pi.ki, g_pos[i].kp);
    }
    printf("\r\n=== COMMANDS ===\r\n");
    printf("R<A> / L<A>      电流模式 (A)\r\n");
    printf("RS<RPM> / LS<RPM>    速度模式 [ramp]\r\n");
    printf("RP<deg> / LP<deg>    位置模式 (绝对角度 °)\r\n");
    printf("RT<RPM> / LT<RPM>    阶跃测试 [no ramp] [burst]\r\n");
    printf("RT<A> <B> / LT<A> <B>    阶跃 A→B\r\n");
    printf("RE<RPM> / LE<RPM>    负载实验\r\n");
    printf("  再次执行 RT/LT/RE/LE  停止测试\r\n");
    printf("PRS / PLS / PS     查询速度 PI (PS=两电机)\r\n");
    printf("PRC / PLC / PC     查询电流 PI (PC=两电机)\r\n");
    printf("P                  查询两电机全部 PI\r\n");
    printf("PRS P=X I=Y        设置速度 PI\r\n");
    printf("PRC P=X I=Y        设置电流 PI\r\n");
    printf("PP / PP Kp=X        查询/设置位置环 Kp\r\n");
    printf("PS P=X I=Y         两电机同时设置\r\n");
    printf("T                  开关遥测输出\r\n");
    printf("?                  帮助\r\n\r\n");
}

/** @brief 电流模式. first_char 是 R/L 后的第一个已读取字符（数字/小数点/符号 或二进制首字节）
 *  @note  支持堆叠命令 R0.2L0.3: 扫描到下一个 R/L 时返回 1 */
static uint8_t CMD_Current(uint8_t motor_idx, uint8_t first_char)
{
    float val;

    // 二进制路径: 首字节非 ASCII 数字 → 4 字节 LE float
    if (!((first_char >= '0' && first_char <= '9') || first_char == '.' ||
          first_char == '-' || first_char == '+')) {
        uint8_t bytes[4];
        bytes[0] = first_char;
        for (uint8_t i = 1; i < 4; i++) {
            uint8_t got = 0;
            for (uint8_t w = 0; w < 5; w++) {
                if (COMM_Available() > 0) { bytes[i] = COMM_ReadByte(); got = 1; break; }
                osDelay(1);
            }
            if (!got) {
                printf("M%d: 二进制参数不完整\r\n", motor_idx + 1);
                return 0;
            }
        }
        memcpy(&val, bytes, 4);
    } else {
        // ASCII 路径: atof 文本解析
        char buf[16]; uint8_t pos = 0;
        buf[pos++] = (char)first_char;
        for (uint8_t w = 0; w < 30 && pos < 15; w++) {
            if (COMM_Available() == 0) { osDelay(1); continue; }
            uint8_t c = COMM_ReadByte();
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                buf[pos++] = (char)c;
            else {
                buf[pos] = '\0';
                val = (float)atof(buf);
                if (g_motor[motor_idx].speed_mode) {
                    SpeedCtrl_ExitMode(&g_speed[motor_idx]);
                    g_motor[motor_idx].speed_mode = 0;
                }
                if (g_pos[motor_idx].active) {
                    PosCtrl_ExitMode(&g_pos[motor_idx]);
                }
                Motor_SetIqRef(&g_motor[motor_idx], val);
                printf("M%d CURRENT iq_ref=%.3fA\r\n", motor_idx + 1, val);
                if (c == 'R' || c == 'r' || c == 'L' || c == 'l') return 1;
                return 0;
            }
        }
        buf[pos] = '\0';
        if (pos > 0) {
            val = (float)atof(buf);
        } else {
            return 0;  // 无有效字符
        }
    }

    // 应用电流给定
    if (g_motor[motor_idx].speed_mode) {
        SpeedCtrl_ExitMode(&g_speed[motor_idx]);
        g_motor[motor_idx].speed_mode = 0;
    }
    if (g_pos[motor_idx].active) {
        PosCtrl_ExitMode(&g_pos[motor_idx]);
    }
    Motor_SetIqRef(&g_motor[motor_idx], val);
    printf("M%d CURRENT iq_ref=%.3fA\r\n", motor_idx + 1, val);
    return 0;
}

/** @brief 速度模式 */
static void CMD_Speed(uint8_t motor_idx)
{
    float rpm;
    if (!CLI_ReadFloat(&rpm)) { printf("RS/LS: 需要转速值 (RPM)\r\n"); return; }
    SpeedCtrl_EnterMode(&g_speed[motor_idx], rpm);
    g_motor[motor_idx].speed_mode = 1;
    printf("M%d SPEED %.0fRPM [ramp=%.0f RPM/s]\r\n", motor_idx + 1, rpm, SPEED_RAMP_MAX);
}

/** @brief 位置模式 — 绝对角度 (°) */
static void CMD_Position(uint8_t motor_idx)
{
    float deg;
    if (!CLI_ReadFloat(&deg)) { printf("RP/LP: 需要角度值 (°)\r\n"); return; }
    float rad = deg * 0.01745329252f;  // deg → rad
    // 进入位置模式前退出速度模式, 重置 PI
    if (g_motor[motor_idx].speed_mode) {
        SpeedCtrl_ExitMode(&g_speed[motor_idx]);
        g_motor[motor_idx].speed_mode = 0;
    }
    // 目标位置设为当前位置 + 相对偏移 (pos_est 已是连续展开值)
    // 绝对角度: 取最近的 2π 整周 + 目标角度
    float cur = g_speed[motor_idx].pos_est;
    float cur_raw = fmodf(cur, 6.283185307f);
    if (cur_raw < 0.0f) cur_raw += 6.283185307f;
    float base = cur - cur_raw;  // 当前最近的 2π 整周
    // 取离当前位置最近的解 (越界时选最近方向)
    float target0 = base + rad;                     // 方向 1
    float target1 = base + rad + 6.283185307f;      // 方向 2 (正转多一圈)
    float target2 = base + rad - 6.283185307f;      // 方向 3 (反转多一圈)
    float err0 = fabsf(target0 - cur);
    float err1 = fabsf(target1 - cur);
    float err2 = fabsf(target2 - cur);
    float target;
    if (err0 <= err1 && err0 <= err2) target = target0;
    else if (err1 <= err2)            target = target1;
    else                             target = target2;
    PosCtrl_EnterMode(&g_pos[motor_idx], target);
    g_motor[motor_idx].speed_mode = 1;
    printf("M%d POS %.1f° (target=%.2frad cur=%.2frad)\r\n", motor_idx + 1, deg, target, cur);
}

/** @brief 阶跃测试（当前转速→目标 或 A→B） — 再次调用可停止 */
static void CMD_Step(uint8_t motor_idx)
{
    if (g_step_test.active) {
        uint8_t mi = g_step_test.motor_idx;
        SpeedCtrl_ExitMode(&g_speed[mi]);
        g_motor[mi].speed_mode = 0;
        g_speed[mi].no_ramp = 0;
        Motor_SetIqRef(&g_motor[mi], 0.0f);
        g_step_test.active = 0;
        printf("M%d STEP 已停止\r\n", mi + 1);
        return;
    }
    if (SpeedCapture_IsBusy()) { printf("M%d STEP capture 忙\r\n", motor_idx + 1); return; }

    float from_rpm, to_rpm;
    if (!CLI_ReadFloat(&to_rpm)) { printf("RT/LT: 需要转速值 (RPM)\r\n"); return; }
    from_rpm = g_speed[motor_idx].speed_fb;

    /* 检查是否有第二个参数 (空格后跟数字) */
    uint8_t next = CLI_ReadChar(5);
    if (next == ' ' || (next >= '0' && next <= '9') || next == '.' || next == '-' || next == '+') {
        if (next == ' ') next = CLI_ReadChar(5);
        if ((next >= '0' && next <= '9') || next == '.' || next == '-' || next == '+') {
            char buf2[16]; uint8_t p2 = 0;
            buf2[p2++] = (char)next;
            for (uint8_t w = 0; w < 30 && p2 < 15; w++) {
                if (COMM_Available() == 0) { osDelay(1); continue; }
                uint8_t c = COMM_ReadByte();
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                    buf2[p2++] = (char)c;
                else break;
            }
            buf2[p2] = '\0';
            from_rpm = to_rpm;
            to_rpm = (float)atof(buf2);
        }
    }

    g_speed[motor_idx].no_ramp = 1;
    SpeedCtrl_EnterMode(&g_speed[motor_idx], from_rpm);
    g_motor[motor_idx].speed_mode = 1;
    g_speed[motor_idx].speed_ref_ramp = from_rpm;
    g_step_test.active = 1;
    g_step_test.motor_idx = motor_idx;
    g_step_test.phase = 0;
    g_step_test.phase_start = xTaskGetTickCount();
    g_step_test.from_rpm = from_rpm;
    g_step_test.to_rpm = to_rpm;
    printf("M%d STEP %d→%dRPM [no ramp] [burst %df]\r\n",
           motor_idx + 1, (int)from_rpm, (int)to_rpm, SC_BURST_SAMPLES);
}

/** @brief 负载实验 — 再次调用可停止 */
static void CMD_Load(uint8_t motor_idx)
{
    if (g_load_test.active) {
        uint8_t mi = g_load_test.motor_idx;
        SpeedCtrl_ExitMode(&g_speed[mi]);
        g_motor[mi].speed_mode = 0;
        Motor_SetIqRef(&g_motor[mi], 0.0f);
        Buzzer_Stop();
        g_load_test.active = 0;
        printf("M%d LOAD 已停止\r\n", mi + 1);
        return;
    }
    float rpm;
    if (!CLI_ReadFloat(&rpm)) { printf("RE/LE: 需要转速值 (RPM)\r\n"); return; }

    SpeedCtrl_EnterMode(&g_speed[motor_idx], rpm);
    g_motor[motor_idx].speed_mode = 1;
    g_load_test.active = 1;
    g_load_test.motor_idx = motor_idx;
    g_load_test.phase = 0;
    g_load_test.phase_start = xTaskGetTickCount();
    g_load_test.rpm = rpm;
    Buzzer_Beep(1000, 80);
    printf("M%d LOAD %.0fRPM [2s ramp→3s buzz→3s idle]\r\n", motor_idx + 1, rpm);
}

/** @brief 位置环参数查询/设置: PP / PP Kp=X */
static void CMD_PosParam(void)
{
    uint8_t peek = CLI_ReadChar(5);
    while (peek == ' ') peek = CLI_ReadChar(5);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        for (int mi = 0; mi < 2; mi++) {
            printf("M%d Pos Kp=%.0f RPM/rad  SpeedMax=±%.0fRPM  (BW~%.1fHz)\r\n",
                   mi + 1, g_pos[mi].kp, g_pos[mi].speed_max,
                   g_pos[mi].kp * 0.0167f);
        }
        return;
    }

    float kp = 0;
    uint8_t kp_set = 0;

    if (peek == 'K' || peek == 'k') {
        uint8_t p2 = CLI_ReadChar(10);
        if (p2 == 'p' || p2 == 'P') {
            uint8_t eq = CLI_ReadChar(10);
            if (eq == '=') {
                if (CLI_ReadFloat(&kp)) kp_set = 1;
            }
        }
    } else if ((peek >= '0' && peek <= '9') || peek == '.' || peek == '-' || peek == '+') {
        char buf[16]; uint8_t p = 0;
        buf[p++] = (char)peek;
        for (uint8_t w = 0; w < 30 && p < 15; w++) {
            if (COMM_Available() == 0) { osDelay(1); continue; }
            uint8_t c = COMM_ReadByte();
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                buf[p++] = (char)c;
            else break;
        }
        buf[p] = '\0';
        kp = (float)atof(buf); kp_set = 1;
    }

    if (!kp_set) { printf("PP: Kp=X 或直接数值\r\n"); return; }
    if (kp < 1.0f) kp = 1.0f;
    if (kp > 500.0f) kp = 500.0f;

    for (int mi = 0; mi < 2; mi++) {
        g_pos[mi].kp = kp;
        printf("M%d Pos Kp=%.0f RPM/rad  (BW~%.1fHz)\r\n", mi + 1, kp, kp * 0.0167f);
    }
}

/** @brief PI 参数查询/设置: PRS/PS/PRC/PC + 可选 P=X I=Y 或位置值
 *  @note  PS/PC 无 R/L 前缀 → 同时应用到两个电机 */
static void CMD_PI_Param(void)
{
    uint8_t ch = CLI_ReadChar(20);
    uint8_t motor_start, motor_end, sub;

    if (ch == 'R' || ch == 'r') {
        motor_start = motor_end = 0;
        sub = CLI_ReadChar(20);
    } else if (ch == 'L' || ch == 'l') {
        motor_start = motor_end = 1;
        sub = CLI_ReadChar(20);
    } else if (ch == 'P' || ch == 'p') {
        /* PP: 位置环参数 */
        CMD_PosParam();
        return;
    } else if (ch == 'S' || ch == 's' || ch == 'C' || ch == 'c') {
        /* PS/PC: 两电机同时操作 */
        motor_start = 0; motor_end = 1;
        sub = ch;
    } else if (ch == 0 || ch == '\r' || ch == '\n') {
        /* P 单独: 查询两电机全部 PI + Pos */
        for (int mi = 0; mi < 2; mi++) {
            printf("M%d Speed  PI: Kp=%.3f Ki=%.3f Out=±%.1fA\r\n",
                   mi + 1, g_speed[mi].kp, g_speed[mi].ki, 2.0f);
            printf("M%d Current PI: Kp=%.1f Ki=%.0f Out=±%.1fV\r\n",
                   mi + 1, g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki, FOC_VBUS);
            printf("M%d Pos    P : Kp=%.0f RPM/rad  Max=±%.0fRPM\r\n",
                   mi + 1, g_pos[mi].kp, g_pos[mi].speed_max);
        }
        return;
    } else {
        printf("P: R/L 前缀 (PRS/PLC) 或 S/C/P (PS/PC/PP 两电机)\r\n");
        return;
    }

    if (sub == 0 || sub == '\r' || sub == '\n') {
        /* PR 或 PL: 查询全部 PI + Pos */
        for (int mi = motor_start; mi <= motor_end; mi++) {
            printf("M%d Speed  PI: Kp=%.3f Ki=%.3f Out=±%.1fA\r\n",
                   mi + 1, g_speed[mi].kp, g_speed[mi].ki, 2.0f);
            printf("M%d Current PI: Kp=%.1f Ki=%.0f Out=±%.1fV\r\n",
                   mi + 1, g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki, FOC_VBUS);
            printf("M%d Pos    P : Kp=%.0f RPM/rad  Max=±%.0fRPM\r\n",
                   mi + 1, g_pos[mi].kp, g_pos[mi].speed_max);
        }
        return;
    }

    uint8_t is_speed;
    if (sub == 'S' || sub == 's') is_speed = 1;
    else if (sub == 'C' || sub == 'c') is_speed = 0;
    else { printf("P: 未知子系统 '%c', 使用 S=Speed C=Current\r\n", sub); return; }

    /* 检查是否有参数 */
    uint8_t peek = CLI_ReadChar(5);
    while (peek == ' ') peek = CLI_ReadChar(5);  /* 跳过空格 */
    if (peek == 0 || peek == '\r' || peek == '\n') {
        /* 纯查询 */
        for (int mi = motor_start; mi <= motor_end; mi++) {
            if (is_speed) {
                float kp = g_speed[mi].kp, ki = g_speed[mi].ki;
                if (kp > 0.0001f)
                    printf("M%d Speed PI: Kp=%.3f Ki=%.3f ω0=%.2fHz Out=±%.1fA\r\n",
                           mi + 1, kp, ki, ki / kp / 6.283f, 2.0f);
                else
                    printf("M%d Speed PI: Kp=%.3f Ki=%.3f ω0=N/A Out=±%.1fA\r\n",
                           mi + 1, kp, ki, 2.0f);
            } else {
                float kp = g_motor[mi].iq_pi.kp, ki = g_motor[mi].iq_pi.ki;
                if (kp > 0.0001f)
                    printf("M%d Current PI: Kp=%.1f Ki=%.0f ω0=%.1fHz Out=±%.1fV\r\n",
                           mi + 1, kp, ki, ki / kp / 6.283f, FOC_VBUS);
                else
                    printf("M%d Current PI: Kp=%.1f Ki=%.0f ω0=N/A Out=±%.1fV\r\n",
                           mi + 1, kp, ki, FOC_VBUS);
            }
        }
        return;
    }

    /* 解析设置值: P=X I=Y (标签式) 或纯数值 Kp Ki (位置式) */
    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;

    if (peek == 'P' || peek == 'p' || peek == 'I' || peek == 'i') {
        do {
            uint8_t ch2 = peek;
            uint8_t eq = CLI_ReadChar(10);
            if (eq != '=') break;
            float val;
            if (!CLI_ReadFloat(&val)) break;
            if (ch2 == 'P' || ch2 == 'p') { kp = val; kp_set = 1; }
            if (ch2 == 'I' || ch2 == 'i') { ki = val; ki_set = 1; }
            peek = CLI_ReadChar(5);
            if (peek == ' ') peek = CLI_ReadChar(5);
        } while (peek == 'P' || peek == 'p' || peek == 'I' || peek == 'i');
    } else if ((peek >= '0' && peek <= '9') || peek == '.' || peek == '-' || peek == '+') {
        char buf[16]; uint8_t p = 0;
        buf[p++] = (char)peek;
        for (uint8_t w = 0; w < 30 && p < 15; w++) {
            if (COMM_Available() == 0) { osDelay(1); continue; }
            uint8_t c = COMM_ReadByte();
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                buf[p++] = (char)c;
            else break;
        }
        buf[p] = '\0';
        kp = (float)atof(buf); kp_set = 1;
        if (CLI_ReadFloat(&ki)) ki_set = 1;
    }

    if (!kp_set && !ki_set) { printf("P: 需要 P=X I=Y 或直接跟 Kp Ki 值\r\n"); return; }

    /* 应用到目标电机 */
    for (int mi = motor_start; mi <= motor_end; mi++) {
        float use_kp = kp_set ? kp : (is_speed ? g_speed[mi].kp : g_motor[mi].iq_pi.kp);
        float use_ki = ki_set ? ki : (is_speed ? g_speed[mi].ki : g_motor[mi].iq_pi.ki);

        if (is_speed) {
            SpeedCtrl_SetGains(&g_speed[mi], use_kp, use_ki);
        } else {
            Motor_SetCurrentPI(&g_motor[mi], use_kp, use_ki);
        }

        printf("M%d %s PI: Kp=%.3f Ki=%.3f",
               mi + 1, is_speed ? "Speed" : "Current", use_kp, use_ki);
        if (use_kp > 0.0001f) {
            if (is_speed)
                printf(" ω0=%.2fHz Out=±%.1fA\r\n", use_ki / use_kp / 6.283f, 2.0f);
            else
                printf(" ω0=%.1fHz Out=±%.1fV\r\n", use_ki / use_kp / 6.283f, FOC_VBUS);
        } else {
            if (is_speed)
                printf(" ω0=N/A Out=±%.1fA\r\n", 2.0f);
            else
                printf(" ω0=N/A Out=±%.1fV\r\n", FOC_VBUS);
        }
    }
}

void CLI_Process(void)
{
    while (COMM_Available() > 0)
    {
      uint8_t ch = COMM_ReadByte();

      // ===== 校准模式 CLI =====
      if (g_calib_mode) {
          if (ch == '?' || ch == 'h' || ch == 'H') {
              printf("\r\n=== 校准模式 ===\r\n");
              printf("R1-5  M1校准  L1-5  M2校准\r\n");
              printf("RS    查看校准参数\r\n");
              printf("q     中止校准\r\n");
              printf("?     帮助\r\n\r\n");
          } else if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
              uint8_t motor_idx = (ch == 'L' || ch == 'l') ? 1 : 0;
              uint8_t func = CLI_ReadChar(20);
              if (func >= '1' && func <= '5') {
                  if (!CALIB_TryLock()) {
                      printf("校准忙, 等待或按 'q' 中止\r\n");
                  } else {
                      printf("=== M%d 校准 #%c ===\r\n", motor_idx + 1, func);
                      switch (func) {
                      case '1': CALIB_CurrentOffset(); break;
                      case '2': CALIB_PhaseWireMap(&g_motor[motor_idx]); break;
                      case '3': CALIB_EncoderDir(&g_motor[motor_idx]); break;
                      case '4': CALIB_EncoderOffset(&g_motor[motor_idx]); break;
                      case '5': CALIB_MotorParams(&g_motor[motor_idx]); break;
                      }
                  }
              } else if (func == 's' || func == 'S') {
                  CALIB_PrintParams(&g_calib);
              }
          } else if (ch == 'q' || ch == 'Q') {
              CALIB_Abort();
              printf("校准中止\r\n");
          }
          continue;
      }

      // ===== 阶跃测试模式 CLI =====
      if (g_test_mode) {
          if (ch == '?' || ch == 'h' || ch == 'H') {
              printf("\r\n=== 阶跃测试模式 ===\r\n");
              printf("R<A>   M1 电流阶跃 (A)\r\n");
              printf("L<A>   M2 电流阶跃 (A)\r\n");
              printf("r     重发上次采集\r\n");
              printf("?     帮助\r\n\r\n");
          } else if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
              uint8_t motor_idx = (ch == 'L' || ch == 'l') ? 1 : 0;
              // peek 下一个字符判断是 resend 还是电流阶跃
              uint8_t peek = CLI_ReadChar(5);
              if (peek == 0 || peek == '\r' || peek == '\n') {
                  // 裸 r → resend
                  DebugCapture_Resend();
              } else if ((peek >= '0' && peek <= '9') || peek == '.' || peek == '-' || peek == '+') {
                  char buf[16]; uint8_t p = 0;
                  buf[p++] = (char)peek;
                  for (uint8_t w = 0; w < 30 && p < 15; w++) {
                      if (COMM_Available() == 0) { osDelay(1); continue; }
                      uint8_t c = COMM_ReadByte();
                      if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                          buf[p++] = (char)c;
                      else break;
                  }
                  buf[p] = '\0';
                  DebugCapture_Start(motor_idx, (float)atof(buf));
              }
          }
          continue;
      }

      if (ch == '?' || ch == 'H' || ch == 'h') {
          CMD_Help();
          continue;
      }

      if (ch == 'T' || ch == 't') {
          g_telem_enabled = !g_telem_enabled;
          printf("TELEMETRY %s\r\n", g_telem_enabled ? "ON" : "OFF");
          continue;
      }

      if (ch == 'P' || ch == 'p') {
          CMD_PI_Param();
          continue;
      }

      if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
          uint8_t motor_idx = (ch == 'L' || ch == 'l') ? 1 : 0;
          uint8_t ch2 = CLI_ReadChar(30);
          if (ch2 == 0) continue;

          if ((ch2 >= '0' && ch2 <= '9') || ch2 == '.' || ch2 == '-' || ch2 == '+') {
              while (CMD_Current(motor_idx, ch2)) {
                  ch = CLI_ReadChar(5);
                  if (ch == 'R' || ch == 'r') motor_idx = 0;
                  else if (ch == 'L' || ch == 'l') motor_idx = 1;
                  else break;
                  ch2 = CLI_ReadChar(30);
                  if (!((ch2 >= '0' && ch2 <= '9') || ch2 == '.' ||
                        ch2 == '-' || ch2 == '+'))
                      break;
              }
          } else {
              switch (ch2) {
              case 'S': case 's': CMD_Speed(motor_idx);  break;
              case 'P': case 'p': CMD_Position(motor_idx); break;
              case 'T': case 't': CMD_Step(motor_idx);   break;
              case 'E': case 'e': CMD_Load(motor_idx);   break;
              default:
                  // 非 ASCII 字节 → 尝试二进制浮点电流参数
                  if (!((ch2 >= '0' && ch2 <= '9') || ch2 == '.' ||
                        ch2 == '-' || ch2 == '+')) {
                      CMD_Current(motor_idx, ch2);
                  } else {
                      printf("M%d: 未知子命令 '%c'. 使用 P=Position S=Speed T=Step E=Load\r\n",
                             motor_idx + 1, ch2);
                      CLI_FlushLine();
                  }
                  break;
              }
          }
          continue;
      }

      if (ch != '\r' && ch != '\n') {
          // 未识别字符 — 静默忽略（避免遥测数据误触发）
      }
    }
}
