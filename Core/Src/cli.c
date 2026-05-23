#include "cli.h"
#include "cli_parser.h"
#include "comm.h"
#include "foc.h"
#include "motor_hal.h"
#include "speed_ctrl.h"
#include "balance_ctrl.h"
#include "calibration.h"
#include "buzzer.h"
#include "vbus.h"
#include "bt_comm.h"
#include "cmsis_os.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 遥测开关
static uint8_t g_telem_enabled = 0;

// AT 桥模式状态
static uint8_t g_at_bridge = 0;

// 蓝牙遥测开关 — 默认关闭, 手机控制帧 bit2 控制
uint8_t g_bt_telem_enabled = 0;

// USART2 半帧保护: 收到 0xA5 但不足 7 字节时暂存, 下次凑齐再处理
static uint8_t     g_bt_pending      = 0;
static TickType_t  g_bt_pending_tick = 0;  // 记录 pending 开始时刻, 用于100ms超时

// 蓝牙控制帧丢弃计数器 — 超时放弃的半帧数
uint32_t g_bt_frame_drop = 0;

/** @brief 从 g_cli_active_port 对应端口读取一个字节, 无数据返回0 */
static uint8_t cli_port_read_byte(void)
{
    if (g_cli_active_port == 1) {
        return BT_COMM_Available() > 0 ? BT_COMM_ReadByte() : 0;
    }
    return COMM_Available() > 0 ? COMM_ReadByte() : 0;
}

// 速度外环 PI 参数 (运行时通过 PS 命令调整)
float g_speed_outer_kp = SPEED_OUTER_KP;
float g_speed_outer_ki = SPEED_OUTER_KI;

// 偏航 PI 参数 (运行时通过 PY 命令调整)
float g_yaw_kp = YAW_PI_KP;
float g_yaw_ki = YAW_PI_KI;

// 外部引用
extern SpeedCtrl_t g_speed[2];
extern BalanceCtrl_t g_balance;
extern uint8_t g_calib_mode;
extern osThreadId TaskCLIHandle;
extern osThreadId TaskBalanceLoopHandle;
extern uint32_t g_imu_fault_total;
extern uint32_t g_nan_fault_cnt;
extern uint32_t g_cycle_overrun_cnt;

void CLI_Init(void)
{
    g_telem_enabled = 0;
}

uint8_t CLI_TelemetryEnabled(void)
{
    return g_telem_enabled;
}

// ===== 帮助 =====
static void CMD_Help(void)
{
    uint32_t uptime = xTaskGetTickCount() / 1000;
    UBaseType_t cli_stack = uxTaskGetStackHighWaterMark(TaskCLIHandle);
    UBaseType_t bal_stack = TaskBalanceLoopHandle
        ? uxTaskGetStackHighWaterMark(TaskBalanceLoopHandle) : 0;

    printf("\r\n=== BALANCE CAR — uptime %lus ===\r\n", (unsigned long)uptime);
    printf("Stack: CLI %lu/384  Balance %lu/896\r\n",
           (unsigned long)cli_stack, (unsigned long)bal_stack);

    // CPU 占用率 (任务级, 不含 ISR — 见 docs/DEBUG.md)
    {
        char stats[512];
        vTaskGetRunTimeStats(stats);
        // 扫描 IDLE 行, 提取百分比: "...IDLE   xxx   yy%\r\n..." → 100-yy = CPU usage
        uint32_t cpu_used = 0;
        char *line = stats;
        while (*line) {
            // 找行尾 (FreeRTOS stats 用 \r\n 分隔)
            char *eol = strchr(line, '\r');
            if (!eol) eol = line + strlen(line);  // 最后一行无 \r
            if ((line[0] == 'I' && line[1] == 'D') ||
                (line[0] == 'T' && line[1] == 'm' && line[2] == 'r')) {
                // 从行尾反向扫描找 % 或 > (<1% 的行显示为 "<1%")
                char *p = eol - 1;
                while (p > line && *p != '%' && *p != '>') p--;
                if (p > line) {
                    char *q = p - 1;
                    while (q > line && *q >= '0' && *q <= '9') q--;
                    uint32_t idle_pct = (uint32_t)strtoul(q + 1, NULL, 10);
                    if (idle_pct < 100) cpu_used = 100 - idle_pct;
                }
                break;
            }
            line = eol;
            if (*line == '\r') line++;
            if (*line == '\n') line++;
        }
        printf("CPU: %lu%% used (不含 ISR, 见 docs/DEBUG.md)\r\n", (unsigned long)cpu_used);
    }

    // ── Balance ──
    if (g_balance.active) {
        printf("\r\n── Balance ON ──\r\n");
        printf("Kp=%.1f Kd=%.1f  Tilt=%.2f° Gyro=%.1f°/s  Out=%.0fRPM\r\n",
               g_balance.kp_angle, g_balance.kd_gyro,
               g_balance.tilt_angle, g_balance.gyro_rate,
               g_balance.balance_out);
        printf("TargetSpeed=%.0fRPM  TargetYaw=%.0f°/s  Steer=%.0fRPM\r\n",
               g_balance.target_speed, g_balance.target_yaw_rate, g_balance.steer);
        printf("Speed L=%.0f/%.0fRPM  R=%.0f/%.0fRPM\r\n",
               g_balance.speed_ref_l, g_speed[0].speed_fb,
               g_balance.speed_ref_r, g_speed[1].speed_fb);
    } else {
        printf("\r\n── Balance OFF — Tilt=%.2f° ──\r\n", g_balance.tilt_angle);
    }

    // ── Power & Faults ──
    printf("\r\n── Power & Faults ──\r\n");
    printf("VBUS=%.2fV %s | Faults: IMU %lu  NaN %lu  Overrun %lu | printf→USART%d",
           VBUS_Read(), VBUS_IsCalibrated() ? "cal" : "uncal",
           (unsigned long)g_imu_fault_total, (unsigned long)g_nan_fault_cnt,
           (unsigned long)g_cycle_overrun_cnt, CLI_GetOutputPort() + 1);
    if (g_at_bridge) printf(" [AT bridge]");
    if (g_bt_telem_enabled) printf(" [BT telem]");
    if (g_bt_frame_drop > 0) printf(" [BT drop:%lu]", (unsigned long)g_bt_frame_drop);
    printf("\r\n");

    // ── PI Gains ──
    printf("\r\n── PI Gains ──\r\n");
    printf("SpeedOuter P=%.3f I=%.3f   Yaw P=%.1f I=%.1f\r\n",
           g_speed_outer_kp, g_speed_outer_ki, g_yaw_kp, g_yaw_ki);
    for (int i = 0; i < 2; i++) {
        printf("M%d Spd P=%.3f I=%.3f   Cur P=%.1f I=%.0f\r\n",
               i + 1, g_speed[i].kp, g_speed[i].ki,
               g_motor[i].iq_pi.kp, g_motor[i].iq_pi.ki);
    }

    // ── 运行命令 ──
    printf("\r\n── 运行 ──\r\n");
    printf("B           激活平衡    STOP        紧急停止\r\n");
    printf("S<RPM>      前向速度    Y<deg/s>     目标偏航\r\n");

    // ── 参数命令 ──
    printf("\r\n── 参数 ──\r\n");
    printf("PK [KEY=VAL] 平衡 PID    PS [P=X I=Y] 速度外环 PI\r\n");
    printf("PY [P=X I=Y] 偏航 PI     PC [P=X I=Y] 电流 PI\r\n");
    printf("P            查询全部参数\r\n");

    // ── 系统命令 ──
    printf("\r\n── 系统 ──\r\n");
    printf("T            遥测开关    VCAL <V>     电压校准\r\n");
    printf("CAL          校准模式    CLI          切换输出\r\n");
    printf("AT           AT 桥模式   ?            帮助\r\n");
    printf("\r\n蓝牙帧协议见 docs/COMM_PROTOCOL.md\r\n\r\n");
}

// ===== 运行指令 =====

static void CMD_Balance(void)
{
    if (!g_balance.active) {
        g_balance.active = 1;
        g_balance.gyro_filt = 0.0f;  // 清空上次残留, 避免误触保护
        for (int i = 0; i < 2; i++) {
            g_motor[i].mode = MOTOR_MODE_CURRENT_LOOP;  // 恢复FOC电流环 (STOP时被Neutralize设为OFF)
            PI_Reset(&g_motor[i].id_pi);
            PI_Reset(&g_motor[i].iq_pi);
            SpeedCtrl_EnterMode(&g_speed[i], 0.0f);
            g_motor[i].speed_mode = 1;
        }
        printf("BALANCE ON\r\n");
    }
}

static void CMD_Stop(void)
{
    g_balance.active = 0;
    for (int i = 0; i < 2; i++) {
        SpeedCtrl_ExitMode(&g_speed[i]);
        g_motor[i].speed_mode = 0;
        Motor_SetIqRef(&g_motor[i], 0.0f);
    }
    g_balance.target_speed = 0.0f;
    g_balance.steer = 0.0f;
    Motor_Neutralize(&g_motor[0]);
    Motor_Neutralize(&g_motor[1]);
    Buzzer_Beep(1000, 80);
    printf("STOP\r\n");
}

static void CMD_Speed(uint8_t first_char)
{
    char buf[16]; uint8_t p = 0;
    buf[p++] = (char)first_char;
    for (uint8_t w = 0; w < 30 && p < 15; w++) {
        uint8_t c = cli_port_read_byte();
        if (c == 0) { osDelay(1); continue; }
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
            buf[p++] = (char)c;
        else break;
    }
    buf[p] = '\0';
    if (p == 0) { printf("S: 需要转速值(RPM)\r\n"); return; }
    float rpm = (float)atof(buf);
    g_balance.target_speed = rpm;
    printf("SPEED %.0fRPM\r\n", rpm);
}

// ===== 母线电压校准 =====

static void CMD_VbusCal(void)
{
    float val;
    if (!CLI_ReadFloat(&val)) {
        printf("VCAL: 需要电压值 (V), 例: VCAL 6.00\r\n");
        printf("  用法: 设置电源到已知电压, 输入 VCAL <V>\r\n");
        printf("  重复两次 (不同电压) 后自动计算 slope/offset 并保存\r\n");
        return;
    }
    if (val < 3.0f || val > 12.0f) {
        printf("VCAL: 电压 %.2fV 超出范围 (3-12V)\r\n", val);
        return;
    }
    VBUS_CalSample(val);
}

// ===== 参数查询/设置 =====

static void CMD_BalanceParam(void)
{
    uint8_t first = 1;
    do {
        uint8_t peek = CLI_ReadChar(5);
        while (peek == ' ') peek = CLI_ReadChar(5);

        if (first && (peek == 0 || peek == '\r' || peek == '\n')) {
            printf("Balance: Kp=%.1f Kd=%.1f TargetAngle=%.1f Max=%.0fRPM\r\n",
                   g_balance.kp_angle, g_balance.kd_gyro,
                   g_balance.target_angle, g_balance.output_max);
            return;
        }
        if (peek == 0 || peek == '\r' || peek == '\n') return;  // 后续循环结束

        char keybuf[8] = {0};
        uint8_t kp = 0;
        uint8_t eq_consumed = 0;
        keybuf[kp++] = (char)peek;
        for (uint8_t w = 0; w < 5 && kp < 7; w++) {
            uint8_t c = CLI_ReadChar(5);
            if (c == '=') { eq_consumed = 1; break; }
            if (c == 0 || c == '\r' || c == '\n' || c == ' ') break;
            if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
                keybuf[kp++] = (char)c;
            else break;
        }
        keybuf[kp] = '\0';

        if (!eq_consumed) {
            uint8_t eq = CLI_ReadChar(10);
            while (eq == ' ') eq = CLI_ReadChar(10);
            if (eq != '=' && eq != 0 && eq != '\r' && eq != '\n') {
                printf("PK: 需要 KEY=VAL 格式 (got '%c')\r\n", eq);
                return;
            }
        }

        float val;
        if (!CLI_ReadFloat(&val)) {
            printf("PK: 需要数值\r\n");
            return;
        }

        // 写 g_balance 成员 (平衡任务 1ms 读取, 编译器屏障确保写入可见)
        if (strcmp(keybuf, "ANG") == 0 || strcmp(keybuf, "ang") == 0) {
            g_balance.kp_angle = val;
            printf("Balance Kp_angle=%.1f\r\n", val);
        } else if (strcmp(keybuf, "GYR") == 0 || strcmp(keybuf, "gyr") == 0) {
            g_balance.kd_gyro = val;
            printf("Balance Kd_gyro=%.1f\r\n", val);
        } else if (strcmp(keybuf, "MAX") == 0 || strcmp(keybuf, "max") == 0) {
            g_balance.output_max = val;
            printf("Balance OutputMax=%.0f\r\n", val);
        } else if (strcmp(keybuf, "ANG0") == 0 || strcmp(keybuf, "ang0") == 0) {
            g_balance.target_angle = val;
            printf("Balance TargetAngle=%.1f\r\n", val);
        } else {
            printf("PK: 未知键'%s', 可用: ANG GYR MAX ANG0\r\n", keybuf);
        }
        COMPILER_BARRIER();  // 确保写入对平衡任务可见
        first = 0;
    // 检查是否有更多 KEY=VALUE 对
    } while (cli_port_read_byte() != 0);
}

static void CMD_AllParams(void)
{
    printf("Balance: Kp=%.1f Kd=%.1f TargetAngle=%.1f Max=%.0fRPM\r\n",
           g_balance.kp_angle, g_balance.kd_gyro,
           g_balance.target_angle, g_balance.output_max);
    printf("SpeedOuter PI: Kp=%.3f Ki=%.3f TargetSpeed=%.0fRPM\r\n",
           g_speed_outer_kp, g_speed_outer_ki, g_balance.target_speed);
    printf("Yaw PI: Kp=%.1f Ki=%.1f TargetYawRate=%.0f deg/s Steer=%.0fRPM\r\n",
           g_yaw_kp, g_yaw_ki,
           g_balance.target_yaw_rate, g_balance.steer);
    for (int mi = 0; mi < 2; mi++) {
        float skp = g_speed[mi].kp, ski = g_speed[mi].ki;
        float ckp = g_motor[mi].iq_pi.kp, cki = g_motor[mi].iq_pi.ki;
        printf("M%d Speed  PI: Kp=%.3f Ki=%.3f ", mi+1, skp, ski);
        if (skp > 0.0001f) printf("0=%.2fHz", ski/skp/6.283f);
        printf(" Out=2.0A\r\n");
        printf("M%d Current PI: Kp=%.1f Ki=%.0f ", mi+1, ckp, cki);
        if (ckp > 0.0001f) printf("0=%.1fHz", cki/ckp/6.283f);
        printf(" Out=7.4V\r\n");
    }
    float vbus = VBUS_Read();
    printf("VBUS: %.2fV (slope=%.6f offset=%.3f %s)\r\n",
           vbus, VBUS_GetSlope(), VBUS_GetOffset(),
           VBUS_IsCalibrated() ? "cal" : "uncal");
}

static void CMD_SetSpeedPI(void)
{
    uint8_t ch = CLI_ReadChar(10);
    uint8_t motor_start = 0, motor_end = 1;
    uint8_t is_outer = 1;  // 默认速度外环
    if (ch == 'R' || ch == 'r') { motor_start = motor_end = 0; is_outer = 0; }
    else if (ch == 'L' || ch == 'l') { motor_start = motor_end = 1; is_outer = 0; }
    else if (ch == ' ' || ch == 'P' || ch == 'p' || ch == 'I' || ch == 'i') {
        // 无电机前缀 → 速度外环, 需要把 ch 放回去给参数解析
        // peek 会重新读, 直接走外环路径
    }

    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;
    uint8_t peek = ch;
    while (peek == ' ') peek = CLI_ReadChar(20);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        if (is_outer) {
            printf("SpeedOuter PI: Kp=%.3f Ki=%.3f\r\n",
                   g_speed_outer_kp, g_speed_outer_ki);
        } else {
            for (int mi = motor_start; mi <= motor_end; mi++) {
                printf("M%d Speed PI: Kp=%.3f Ki=%.3f\r\n",
                       mi+1, g_speed[mi].kp, g_speed[mi].ki);
            }
        }
        return;
    }

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
    }

    if (is_outer) {
        if (kp_set) g_speed_outer_kp = kp;
        if (ki_set) g_speed_outer_ki = ki;
        COMPILER_BARRIER();  // 确保写入对平衡任务可见
        printf("SpeedOuter PI: Kp=%.3f Ki=%.3f\r\n",
               g_speed_outer_kp, g_speed_outer_ki);
    } else {
        for (int mi = motor_start; mi <= motor_end; mi++) {
            if (kp_set) { g_speed[mi].kp = kp; g_speed[mi].pi.kp = kp; }
            if (ki_set) { g_speed[mi].ki = ki; g_speed[mi].pi.ki = ki; }
            printf("M%d Speed PI: Kp=%.3f Ki=%.3f\r\n", mi+1,
                   g_speed[mi].kp, g_speed[mi].ki);
        }
    }
}

static void CMD_SetYawPI(void)
{
    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;
    uint8_t peek = CLI_ReadChar(20);
    while (peek == ' ') peek = CLI_ReadChar(20);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        printf("Yaw PI: Kp=%.1f Ki=%.1f\r\n", g_yaw_kp, g_yaw_ki);
        return;
    }

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
    }

    if (kp_set) g_yaw_kp = kp;
    if (ki_set) g_yaw_ki = ki;
    COMPILER_BARRIER();  // 确保写入对平衡任务可见
    printf("Yaw PI: Kp=%.1f Ki=%.1f\r\n", g_yaw_kp, g_yaw_ki);
}

static void CMD_SetCurrentPI(void)
{
    uint8_t ch = CLI_ReadChar(10);
    uint8_t motor_start = 0, motor_end = 1;
    if (ch == 'R' || ch == 'r') { motor_start = motor_end = 0; }
    else if (ch == 'L' || ch == 'l') { motor_start = motor_end = 1; }

    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;
    uint8_t peek = CLI_ReadChar(20);
    while (peek == ' ') peek = CLI_ReadChar(20);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        for (int mi = motor_start; mi <= motor_end; mi++) {
            printf("M%d Current PI: Kp=%.1f Ki=%.0f\r\n",
                   mi+1, g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki);
        }
        return;
    }

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
    }

    for (int mi = motor_start; mi <= motor_end; mi++) {
        if (kp_set || ki_set) {
            float use_kp = kp_set ? kp : g_motor[mi].iq_pi.kp;
            float use_ki = ki_set ? ki : g_motor[mi].iq_pi.ki;
            Motor_SetCurrentPI(&g_motor[mi], use_kp, use_ki);
        }
        printf("M%d Current PI: Kp=%.1f Ki=%.0f\r\n", mi+1,
               g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki);
    }
}

// ===== 主 CLI 循环 =====

/** @brief 处理一个 CLI 字符 (校准/正常模式分发, 端口无关) */
static void CLI_DispatchChar(uint8_t ch)
{
    // 校准模式 CLI
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
                    printf("校准忙, 等待或按'q'中止\r\n");
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
            g_calib_mode = 0;
            printf("校准中止, 返回正常模式\r\n");
            vTaskResume(TaskBalanceLoopHandle);
        }
        return;
    }

    // 正常模式 CLI
    if (ch == '?' || ch == 'H' || ch == 'h') {
        CMD_Help(); return;
    }

    if (ch == 'B' || ch == 'b') {
        uint8_t nxt = CLI_ReadChar(5);
        if (nxt == 0 || nxt == '\r' || nxt == '\n') {
            CMD_Balance();
        }
        return;
    }

    if (ch == 'S' || ch == 's') {
        uint8_t nxt = CLI_ReadChar(20);
        if (nxt == 'T' || nxt == 't') {
            uint8_t nxt2 = CLI_ReadChar(5);
            if (nxt2 == 'O' || nxt2 == 'o') {
                uint8_t nxt3 = CLI_ReadChar(5);
                if (nxt3 == 'P' || nxt3 == 'p') {
                    CMD_Stop();
                    return;
                }
            }
        } else if (nxt == 0 || nxt == '\r' || nxt == '\n') {
            printf("S: 用 S<RPM> 或 STOP\r\n");
        } else if ((nxt >= '0' && nxt <= '9') || nxt == '.' || nxt == '-' || nxt == '+') {
            CMD_Speed(nxt);
        } else {
            printf("S: 未知子命令'%c'\r\n", nxt);
        }
        return;
    }

    if (ch == 'T' || ch == 't') {
        // 避免 "TM+" "TN+" 等 boot 消息被误读, 仅在单独 T 后跟行尾时触发
        uint8_t nxt = cli_port_read_byte();
        if (nxt == 0 || nxt == '\r' || nxt == '\n') {
            g_telem_enabled = !g_telem_enabled;
            printf("TELEMETRY %s\r\n", g_telem_enabled ? "ON" : "OFF");
        }
        return;
    }

    if (ch == 'Y' || ch == 'y') {
        uint8_t nxt = CLI_ReadChar(5);
        if ((nxt >= '0' && nxt <= '9') || nxt == '.' || nxt == '-' || nxt == '+') {
            char buf[16]; uint8_t p = 0;
            buf[p++] = (char)nxt;
            for (uint8_t w = 0; w < 30 && p < 15; w++) {
                uint8_t c = cli_port_read_byte();
                if (c == 0) { osDelay(1); continue; }
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                    buf[p++] = (char)c;
                else break;
            }
            buf[p] = '\0';
            g_balance.target_yaw_rate = (float)atof(buf);
            printf("YAW %.1f deg/s\r\n", g_balance.target_yaw_rate);
        }
        return;
    }

    if (ch == 'V' || ch == 'v') {
        uint8_t nxt1 = CLI_ReadChar(20);
        uint8_t nxt2 = CLI_ReadChar(20);
        uint8_t nxt3 = CLI_ReadChar(20);
        if ((nxt1 == 'C' || nxt1 == 'c') &&
            (nxt2 == 'A' || nxt2 == 'a') &&
            (nxt3 == 'L' || nxt3 == 'l')) {
            CMD_VbusCal();
        }
        return;
    }

    if (ch == 'P' || ch == 'p') {
        uint8_t nxt = CLI_ReadChar(10);
        if (nxt == 'K' || nxt == 'k') {
            CMD_BalanceParam();
        } else if (nxt == 'S' || nxt == 's') {
            CMD_SetSpeedPI();
        } else if (nxt == 'Y' || nxt == 'y') {
            CMD_SetYawPI();
        } else if (nxt == 'C' || nxt == 'c') {
            CMD_SetCurrentPI();
        } else if (nxt == 0 || nxt == '\r' || nxt == '\n') {
            CMD_AllParams();
        }
        return;
    }

    if (ch == 'C' || ch == 'c') {
        uint8_t nxt1 = CLI_ReadChar(10);
        uint8_t nxt2 = CLI_ReadChar(10);
        // CLI 切换输出端口 (在 CAL 之前检测, 避免被 CAL 误吞)
        if ((nxt1 == 'L' || nxt1 == 'l') && (nxt2 == 'I' || nxt2 == 'i')) {
            CLI_SetOutputPort(g_cli_active_port);
            printf("CLI output -> USART%d\r\n", g_cli_active_port + 1);
            return;
        }
        // CAL 进入校准模式
        if ((nxt1 == 'A' || nxt1 == 'a') && (nxt2 == 'L' || nxt2 == 'l')) {
            printf("Entering calibration mode...\r\n");
            vTaskSuspend(TaskBalanceLoopHandle);
            g_balance.active = 0;
            for (int i = 0; i < 2; i++) {
                SpeedCtrl_ExitMode(&g_speed[i]);
                g_motor[i].speed_mode = 0;
                Motor_SetIqRef(&g_motor[i], 0.0f);
            }
            Motor_Neutralize(&g_motor[0]);
            Motor_Neutralize(&g_motor[1]);
            g_calib_mode = 1;
            printf("=== CALIBRATION MODE ===\r\n");
            printf("Commands: R1-5=M1 L1-5=M2 RS=params q=abort\r\n");
        }
        return;
    }
}

/** @brief 检测 USART1 "AT\r\n" 序列, 触发 AT 桥模式 */
static uint8_t CLI_DetectAT(void)
{
    uint8_t c1 = COMM_Available() > 0 ? COMM_ReadByte() : 0;
    if (c1 != 'T' && c1 != 't') return 0;

    uint8_t end = COMM_Available() > 0 ? COMM_ReadByte() : 0;
    if (end != '\r' && end != '\n') return 0;

    // 消耗 \r\n 另一半 (\r 后可能有 \n, 反之亦然)
    COMM_Available() > 0 ? COMM_ReadByte() : 0;

    // V2.0 固件模块始终接受 AT 指令, 直接进入桥模式转发
    g_at_bridge = 1;
    printf("AT bridge ON (send AT+QT to test, EXIT to quit)\r\n");
    return 1;
}

/** @brief AT 桥模式: USART1↔USART2 透传, 检测 EXIT 退出
 *  @note  模块仅在 BLE 连接后处理 AT 指令, 使用时需保持手机蓝牙连接
 */
static void CLI_ATBridgeRun(void)
{
    // USART1 RX → USART2 TX, 同时检测 "EXIT" 序列
    while (COMM_Available() > 0) {
        uint8_t ch = COMM_ReadByte();
        BT_COMM_SendByte(ch);
        // 4字节环形缓冲检测 "EXIT" (大小写不敏感)
        {
            static uint8_t ebuf[4] = {0};
            static uint8_t eidx = 0;
            ebuf[eidx] = ch;
            eidx = (eidx + 1) & 3;
            if ((ebuf[eidx] == 'E' || ebuf[eidx] == 'e') &&
                (ebuf[(eidx + 1) & 3] == 'X' || ebuf[(eidx + 1) & 3] == 'x') &&
                (ebuf[(eidx + 2) & 3] == 'I' || ebuf[(eidx + 2) & 3] == 'i') &&
                (ebuf[(eidx + 3) & 3] == 'T' || ebuf[(eidx + 3) & 3] == 't')) {
                g_at_bridge = 0;
                printf("\r\nAT bridge OFF\r\n");
                return;
            }
        }
    }
    // USART2 RX → USART1 TX (蓝牙模块回复)
    while (BT_COMM_Available() > 0) {
        COMM_SendByte(BT_COMM_ReadByte());
    }
}

void CLI_Process(void)
{
    // === AT 桥模式 ===
    if (g_at_bridge) {
        CLI_ATBridgeRun();
        return;
    }

    // === 处理 USART1 ===
    while (COMM_Available() > 0)
    {
        uint8_t ch = COMM_ReadByte();

        // "AT" 序列检测 (仅在 USART1 支持)
        if (ch == 'A' || ch == 'a') {
            if (CLI_DetectAT()) return;
            // 不是 AT, 丢弃 'A' 继续
            continue;
        }

        g_cli_active_port = 0;
        CLI_DispatchChar(ch);
    }

    // === 处理 USART2 ===

    // 上次遗留的半帧续传 (0xA5 已消费, 但 payload 不足 7 字节)
    if (g_bt_pending) {
        if (BT_COMM_Available() >= 7) {
            g_bt_pending = 0;
            goto parse_bt_frame;  // 直接跳到帧解析, 不再读 header
        }
        // 超时 100ms: 放弃残缺帧, 恢复正常 ASCII 处理
        if ((xTaskGetTickCount() - g_bt_pending_tick) > 100) {
            g_bt_pending = 0;
            g_bt_frame_drop++;
            // 继续执行下面的正常 ASCII 处理, 不 goto usart2_done
        } else {
            goto usart2_done;
        }
    }

    while (BT_COMM_Available() > 0)
    {
        uint8_t ch = BT_COMM_ReadByte();

        // 二进制控制帧检测
        if (ch == BT_FRAME_HEADER) {  // 0xA5
            if (BT_COMM_Available() < 7) {
                // 不足 7 字节: header 已消费, 标记 pending, 等下次凑齐
                g_bt_pending = 1;
                g_bt_pending_tick = xTaskGetTickCount();
                break;
            }
parse_bt_frame:
            {
                uint8_t buf[7];
                for (int i = 0; i < 7; i++) buf[i] = BT_COMM_ReadByte();

                // 校验帧尾
                if (buf[6] != BT_FRAME_FOOTER) continue;

                // 校验和: payload(5B) 求和低8位 (不含0xA5包头)
                uint8_t csum = 0;
                for (int i = 0; i < 5; i++) csum += buf[i];
                if (csum != buf[5]) continue;

                uint8_t  flags     = buf[0];
                int16_t  speed_pct = (int16_t)(buf[1] | ((int16_t)buf[2] << 8));
                int16_t  steer_pct = (int16_t)(buf[3] | ((int16_t)buf[4] << 8));

                if (flags & 0x02) { CMD_Stop(); continue; }

                if ((flags & 0x01) && !g_balance.active) {
                    CMD_Balance();
                } else if (!(flags & 0x01) && g_balance.active) {
                    CMD_Stop();
                }

                g_bt_telem_enabled = (flags & 0x04) ? 1 : 0;

                g_balance.target_speed = (float)speed_pct * BALANCE_OUTPUT_MAX / 1000.0f;
                g_balance.steer = (float)steer_pct * BALANCE_STEER_MAX / 1000.0f;

                continue;
            }
        }

        // ASCII 文本
        g_cli_active_port = 1;
        CLI_DispatchChar(ch);
    }
usart2_done:;
}
