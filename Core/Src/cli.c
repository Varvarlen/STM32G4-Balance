#include "cli.h"
#include "cli_parser.h"
#include "comm.h"
#include "foc.h"
#include "motor_hal.h"
#include "calibration.h"
#include "encoder_cache.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ===== 全局实例（供状态机访问） ===== */
LoadTest_t g_load_test;
StepTest_t g_step_test;

/* 遥测开关 */
static uint8_t g_telem_enabled = 0;

/* 外部引用 */
extern uint8_t g_calib_mode;

/* ===== 初始化 ===== */

/**
 * @brief  CLI 模块初始化
 */
void CLI_Init(void)
{
    g_telem_enabled = 0;
}

/**
 * @brief  查询遥测是否启用
 * @retval 0=禁用, 1=启用
 */
uint8_t CLI_TelemetryEnabled(void)
{
    return g_telem_enabled;
}

/* ===== CLI 辅助函数 ===== */

/**
 * @brief  显示帮助信息
 */
static void CMD_Help(void)
{
    printf("=== FOC CLI ===\r\n");
    printf("?/H       帮助\r\n");
    printf("T         切换遥测 ON/OFF\r\n");
    printf("P         PI 参数设置\r\n");
    printf("R<值>     M1 iq_ref (例: R0.5)\r\n");
    printf("L<值>     M2 iq_ref (例: L-0.3)\r\n");
    printf("R<值>L<值> 同时设置两电机\r\n");
    printf("RS/RT/RE  M1 转速/阶跃/负载\r\n");
    printf("LS/LT/LE  M2 转速/阶跃/负载\r\n");
}

/**
 * @brief  处理电流给定命令（R/L + 数值）
 * @param  motor_idx  电机索引 (0=M1, 1=M2)
 * @param  first_char 已从缓冲区读出的首字符（数字/小数点/符号）
 * @retval 1=下一个字符是 R/L（链式命令继续）, 0=结束
 */
static uint8_t CMD_Current(uint8_t motor_idx, uint8_t first_char)
{
    char buf[16];
    uint8_t pos = 0;
    buf[pos++] = (char)first_char;

    /* 读取剩余数值字符 */
    for (int w = 0; w < 30 && pos < 15; w++) {
        if (COMM_Available() == 0) { osDelay(1); continue; }
        uint8_t c = COMM_ReadByte();
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
            buf[pos++] = (char)c;
        } else {
            buf[pos] = '\0';
            Motor_SetIqRef(&g_motor[motor_idx], (float)atof(buf));
            /* 检查终止符是否为另一电机字母（链式命令） */
            return (c == 'R' || c == 'r' || c == 'L' || c == 'l') ? 1 : 0;
        }
    }
    buf[pos] = '\0';
    Motor_SetIqRef(&g_motor[motor_idx], (float)atof(buf));
    return 0;
}

/**
 * @brief  显示指定电机的当前编码器角度
 * @param  motor_idx  电机索引 (0=M1, 1=M2)
 */
static void CMD_Speed(uint8_t motor_idx)
{
    /* g_enc 由编码器 ISR 更新，此处读取快照 */
    printf("M%d: 电角度=%.2f rad, 机械角度=%.2f rad, 原始=%u\r\n",
           motor_idx + 1,
           g_motor[motor_idx].elec_angle,
           g_enc[motor_idx].mech_angle,
           (unsigned int)g_enc[motor_idx].raw_angle);
}

/**
 * @brief  阶跃测试命令：启动/停止
 * @param  motor_idx  电机索引 (0=M1, 1=M2)
 */
static void CMD_Step(uint8_t motor_idx)
{
    if (!g_step_test.active) {
        /* 启动阶跃测试 */
        float from_rpm, to_rpm;
        printf("M%d 阶跃测试——起始 RPM: ", motor_idx + 1);
        if (!CLI_ReadFloat(&from_rpm)) {
            printf("超时取消\r\n");
            return;
        }
        printf("目标 RPM: ");
        if (!CLI_ReadFloat(&to_rpm)) {
            printf("超时取消\r\n");
            return;
        }
        g_step_test.active      = 1;
        g_step_test.motor_idx   = motor_idx;
        g_step_test.phase       = 0;
        g_step_test.phase_start = osKernelSysTick();
        g_step_test.from_rpm    = from_rpm;
        g_step_test.to_rpm      = to_rpm;
        printf("M%d 阶跃测试已启动: %.0f → %.0f RPM\r\n",
               motor_idx + 1, from_rpm, to_rpm);
    } else {
        /* 停止阶跃测试 */
        printf("M%d 阶跃测试已停止\r\n", g_step_test.motor_idx + 1);
        g_step_test.active = 0;
    }
}

/**
 * @brief  负载实验命令：启动/停止
 * @param  motor_idx  电机索引 (0=M1, 1=M2)
 */
static void CMD_Load(uint8_t motor_idx)
{
    if (!g_load_test.active) {
        /* 启动负载实验 */
        float rpm;
        printf("M%d 负载实验——目标 RPM: ", motor_idx + 1);
        if (!CLI_ReadFloat(&rpm)) {
            printf("超时取消\r\n");
            return;
        }
        g_load_test.active      = 1;
        g_load_test.motor_idx   = motor_idx;
        g_load_test.phase       = 0;
        g_load_test.phase_start = osKernelSysTick();
        g_load_test.rpm         = rpm;
        printf("M%d 负载实验已启动: %.0f RPM\r\n", motor_idx + 1, rpm);
    } else {
        /* 停止负载实验 */
        printf("M%d 负载实验已停止\r\n", g_load_test.motor_idx + 1);
        g_load_test.active = 0;
    }
}

/**
 * @brief  PI 参数设置（交互式 key=value）
 */
static void CMD_PI_Param(void)
{
    char key[8];
    float val;

    printf("=== PI 参数设置 ===\r\n");
    printf("可用键: Kp_d Ki_d Kp_q Ki_q (格式: Kp_d=12.0)\r\n");
    printf("输入参数 (空行退出): ");

    while (CLI_ReadKeyValue(key, sizeof(key), &val)) {
        /* 匹配键名并应用到对应 PI 控制器 */
        uint8_t applied = 0;
        for (int m = 0; m < 2; m++) {
            if (strcmp(key, "Kp_d") == 0) {
                g_motor[m].id_pi.kp = val;
                printf("M%d id kp=%.4f\r\n", m + 1, val);
                applied = 1;
            } else if (strcmp(key, "Ki_d") == 0) {
                g_motor[m].id_pi.ki = val;
                printf("M%d id ki=%.4f\r\n", m + 1, val);
                applied = 1;
            } else if (strcmp(key, "Kp_q") == 0) {
                g_motor[m].iq_pi.kp = val;
                printf("M%d iq kp=%.4f\r\n", m + 1, val);
                applied = 1;
            } else if (strcmp(key, "Ki_q") == 0) {
                g_motor[m].iq_pi.ki = val;
                printf("M%d iq ki=%.4f\r\n", m + 1, val);
                applied = 1;
            }
        }
        if (!applied) {
            printf("未知键 '%s', 可用: Kp_d Ki_d Kp_q Ki_q\r\n", key);
        }
        printf("输入参数 (空行退出): ");
    }
    printf("PI 参数设置完成\r\n");
}

/* ===== CLI 主处理函数 ===== */

/**
 * @brief  CLI 命令处理（由 StartCLITask 每 tick 调用一次）
 * @note   非阻塞：每次调用处理串口缓冲区中所有可用字节
 */
void CLI_Process(void)
{
    while (COMM_Available() > 0) {
        uint8_t ch = COMM_ReadByte();

        /* ===== 校准模式 CLI ===== */
        if (g_calib_mode) {
            switch (ch) {
            case 'c': case 'C':
            case 'd': case 'D': {
                uint8_t motor_idx = (ch == 'd' || ch == 'D') ? 1 : 0;
                for (int wait = 0; wait < 50 && COMM_Available() == 0; wait++)
                    osDelay(1);
                ch = COMM_ReadByte();
                if (ch < '1' || ch > '5') break;
                if (!CALIB_TryLock()) {
                    printf("Calibration busy, wait or press 'q' to abort\r\n");
                    break;
                }
                printf("=== M%d calib #%c ===\r\n", motor_idx + 1, ch);
                switch (ch) {
                case '1': CALIB_CurrentOffset(); break;
                case '2': CALIB_PhaseWireMap(&g_motor[motor_idx]); break;
                case '3': CALIB_EncoderDir(&g_motor[motor_idx]); break;
                case '4': CALIB_EncoderOffset(&g_motor[motor_idx]); break;
                case '5': CALIB_MotorParams(&g_motor[motor_idx]); break;
                }
                break;
            }
            case 's': case 'S':
                CALIB_PrintParams(&g_calib);
                break;
            case 'q': case 'Q':
                CALIB_Abort();
                printf("CALIB ABORT requested\r\n");
                break;
            case '\r': case '\n':
                break;
            default:
                printf("calib: c1-5=M1 d1-5=M2 s=params q=abort\r\n");
                break;
            }
            continue;
        }

        /* ===== 正常模式 CLI ===== */

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
                /* 数值输入 → 电流给定 */
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
                /* 字母子命令 */
                switch (ch2) {
                case 'S': case 's': CMD_Speed(motor_idx);  break;
                case 'T': case 't': CMD_Step(motor_idx);   break;
                case 'E': case 'e': CMD_Load(motor_idx);   break;
                default:
                    printf("M%d: 未知子命令 '%c'. 使用 S=Speed T=Step E=Load\r\n",
                           motor_idx + 1, ch2);
                    CLI_FlushLine();
                    break;
                }
            }
            continue;
        }

        /* 空白字符静默忽略 */
        if (ch != '\r' && ch != '\n') {
            /* 未识别字符 — 静默忽略 */
        }
    }
}
