#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void CLI_Init(void);
void CLI_Process(void);
uint8_t CLI_TelemetryEnabled(void);

#endif
