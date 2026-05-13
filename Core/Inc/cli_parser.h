#ifndef CLI_PARSER_H
#define CLI_PARSER_H

#include <stdint.h>

/** @brief 超时时间内从串口读取一个字符，超时返回 0 */
uint8_t CLI_ReadChar(uint8_t timeout_ticks);

/** @brief 超时时间内从串口读取一个浮点数，成功返回 1 */
uint8_t CLI_ReadFloat(float *out);

/** @brief 超时时间内从串口读取一个整数，成功返回 1 */
uint8_t CLI_ReadInt(int *out);

/**
 * @brief 从串口读取 "KEY=float" 格式（如 "P=0.015"），成功返回 1
 * @param key_out 存放键名的缓冲区
 * @param key_max 键名缓冲区最大字节数
 * @param val_out 存放解析出的浮点数值
 */
uint8_t CLI_ReadKeyValue(char *key_out, uint8_t key_max, float *val_out);

/** @brief 丢弃串口缓冲区中直到行尾的所有字符 */
void CLI_FlushLine(void);

#endif
