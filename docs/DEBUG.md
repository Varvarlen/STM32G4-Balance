# 调试指南

## GDB 服务器

启动 GDB 服务器（需另开终端）：
```bash
tools/start_gdb_server.bat
```
端口: **61234**（SWD 接口）。

## 烧录并运行

`tools/flash.gdb` 脚本内容：
```
target remote localhost:61234
monitor halt
load
monitor reset
continue
```

执行：
```bash
timeout 5 arm-none-eabi-gdb build/Debug/STM32G431Demo.elf -x tools/flash.gdb
```

`timeout` 在 `continue` 后自动终止 GDB，MCU 继续运行。

## GDB 常用指令（batch 模式）

### 读取寄存器
```bash
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "info registers"
```

### 读取内存（查看外设寄存器）
```bash
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "x/4xw 0x40012400"
```

### 读取变量（task 上下文可能不准确）
```bash
arm-none-eabi-gdb build/Debug/STM32G431Demo.elf \
  -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "print xTickCount" \
  -ex "print xPortGetFreeHeapSize()"
```

## HardFault 诊断

```bash
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  build/Debug/STM32G431Demo.elf \
  -ex "bt" \
  -ex "print/x SCB->CFSR" \
  -ex "print/x SCB->HFSR" \
  -ex "print/x SCB->BFAR"
```

CFSR 常见值：`0x8200` = PRECISERR + BFARVALID（精确总线错误，BFAR 指向非法地址，通常是栈溢出导致）。

## 调试注意事项

- **烧录后确认复位**：烧录完成后应听到初始化提示音（蜂鸣器 2000Hz），若无声说明系统未启动
- **FreeRTOS 调试**：设断点时注意任务切换，当前任务上下文由 GDB 管理，其他任务仍然在运行
- **中断调试**：设断点于中断服务函数时，避免长时间停留（可能触发看门狗或外设超时）
- **ADC DMA 调试**：单步执行会暂停 DMA 传输，恢复运行后 DMA 会自动恢复（Circular 模式）
- **串口输出**：调试信息可通过 USART1（230400）查看，GDB 不干扰串口通信
