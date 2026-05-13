#include "cli_parser.h"
#include "comm.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

/** @brief 判断字符是否属于浮点数的一部分 */
static inline uint8_t is_float_char(uint8_t c)
{
    return (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+';
}

/** @brief 判断字符是否为 Token 分隔符（空格/回车/换行） */
static inline uint8_t is_sep(uint8_t c)
{
    return c == ' ' || c == '\r' || c == '\n';
}

/** @brief 超时时间内从串口读取一个字符，超时返回 0 */
uint8_t CLI_ReadChar(uint8_t timeout_ticks)
{
    for (uint8_t w = 0; w < timeout_ticks; w++) {
        if (COMM_Available() > 0)
            return COMM_ReadByte();
        osDelay(1);
    }
    return 0;
}

/** @brief 超时时间内从串口读取一个浮点数，成功返回 1 */
uint8_t CLI_ReadFloat(float *out)
{
    char buf[16];
    uint8_t pos = 0;

    for (uint8_t w = 0; w < 50 && pos < 15; w++) {
        if (COMM_Available() == 0) { osDelay(1); continue; }
        uint8_t c = COMM_ReadByte();
        if (is_float_char(c)) {
            buf[pos++] = (char)c;
        } else {
            break;  // 遇到分隔符或其他字符，结束
        }
    }
    if (pos == 0) return 0;
    buf[pos] = '\0';
    *out = (float)atof(buf);
    return 1;
}

/** @brief 超时时间内从串口读取一个整数，成功返回 1 */
uint8_t CLI_ReadInt(int *out)
{
    char buf[12];
    uint8_t pos = 0;

    for (uint8_t w = 0; w < 30 && pos < 10; w++) {
        if (COMM_Available() == 0) { osDelay(1); continue; }
        uint8_t c = COMM_ReadByte();
        if (c == '-' || (c >= '0' && c <= '9')) {
            buf[pos++] = (char)c;
        } else {
            break;
        }
    }
    if (pos == 0) return 0;
    buf[pos] = '\0';
    *out = atoi(buf);
    return 1;
}

/** @brief 从串口读取 "KEY=float" 格式，成功返回 1 */
uint8_t CLI_ReadKeyValue(char *key_out, uint8_t key_max, float *val_out)
{
    uint8_t pos = 0;
    // 读取 key 直到 '='
    for (uint8_t w = 0; w < 30 && pos < key_max - 1; w++) {
        if (COMM_Available() == 0) { osDelay(1); continue; }
        uint8_t c = COMM_ReadByte();
        if (c == '=') break;
        if (c == ' ' || c == '\r' || c == '\n') continue;  // 跳过空白
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
            key_out[pos++] = (char)c;
        else break;
    }
    key_out[pos] = '\0';
    if (pos == 0) return 0;

    // 读取 value
    return CLI_ReadFloat(val_out);
}

/** @brief 丢弃串口缓冲区中直到行尾的所有字符 */
void CLI_FlushLine(void)
{
    for (uint8_t i = 0; i < 64; i++) {
        if (COMM_Available() == 0) break;
        uint8_t c = COMM_ReadByte();
        if (c == '\r' || c == '\n') break;
    }
}
