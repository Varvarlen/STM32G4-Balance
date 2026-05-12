# STM32G431 Demo 项目规范

## 开发环境

| 项目 | 内容 |
|------|------|
| 目标芯片 | **STM32G431CBU6**（Cortex-M4F, 170MHz, UFQFPN48） |
| 固件包 | STM32Cube FW_G4 **V1.6.2** |
| IDE / 构建 | **CLion** + **CMake** + **Ninja** |
| 编译器 | **arm-none-eabi-gcc** 13.3.1（GNU Tools for STM32） |
| 调试器 | **ST-LINK**（SWD 接口），`ST-LINK_gdbserver.exe` + `arm-none-eabi-gdb` |
| 调试端口 | **61234**（SWD 接口） |
| 工具链路径 | `D:\program\embedded\STM\tool\STM32CubeCLT\STM32CubeCLT_1.18.0\` |
| 串口 | USART1, 波特率 **230400** |
| RTOS | **FreeRTOS** V10.3.1 (CMSIS_V1) |

### 构建命令

```bash
cmake --preset Debug          # 配置
cmake --build --preset Debug   # 编译
```

### 数据手册

查阅外设寄存器或 HAL 用法时，参见 `datasheet/` 目录下的 PDF：
- **rm0440** — STM32G4 系列参考手册（外设寄存器级说明）
- **stm32g431cb.pdf** — 芯片数据手册（引脚定义/电气特性）
- **STM32G4 HAL编程手册.pdf** — HAL 库函数说明

## 调试

### GDB 服务器

启动 GDB 服务器（需另开终端）：
```bash
tools/start_gdb_server.bat
```

### 烧录并运行（batch 模式）

**关键**：`monitor reset` 后必须接 `continue`，否则 MCU 停在复位向量不运行。

使用脚本 `tools/flash.gdb`：
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

### GDB 常用指令（batch 模式）

```bash
# 读取寄存器
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "info registers"

# 读取内存（查看外设寄存器）
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "x/4xw 0x40012400"

# 读取变量（task 上下文可能不准确）
arm-none-eabi-gdb build/Debug/STM32G431Demo.elf \
  -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "print xTickCount" \
  -ex "print xPortGetFreeHeapSize()"
```

### HardFault 诊断

```bash
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  build/Debug/STM32G431Demo.elf \
  -ex "bt" \
  -ex "print/x SCB->CFSR" \
  -ex "print/x SCB->HFSR" \
  -ex "print/x SCB->BFAR"
```

CFSR 常见值：`0x8200` = PRECISERR + BFARVALID（精确总线错误，BFAR 指向非法地址，通常是栈溢出导致）

### 注意事项

- **烧录后确认复位**：烧录完成后应听到初始化提示音（蜂鸣器 2000Hz），若无声说明系统未启动
- **FreeRTOS 调试**：设断点时注意任务切换，当前任务上下文由 GDB 管理，其他任务仍然在运行
- **中断调试**：设断点于中断服务函数时，避免长时间停留（可能触发看门狗或外设超时）
- **ADC DMA 调试**：单步执行会暂停 DMA 传输，恢复运行后 DMA 会自动恢复（Circular 模式）
- **串口输出**：调试信息可通过 USART1（230400）查看，GDB 不干扰串口通信

## 注释语言

所有代码注释（包括单行 `//` 注释、多行 `/* */` 注释、Doxygen 文档注释 `/** */` 等）必须使用**中文**书写。Doxygen 关键字（如 `@brief`、`@param`、`@retval` 等）保留英文不变。

## 片上外设配置

当需要修改 STM32 单片机片上外设配置（包括但不限于 GPIO、定时器、ADC、DAC、SPI、I2C、USART、DMA、中断优先级、时钟树等）时，**不要直接修改代码**，而是提醒用户通过 STM32CubeMX（`.ioc` 文件）进行配置，重新生成代码后再适配。

提供 CubeMX 配置指导时，必须以**表格形式**列出每个参数的**完整路径**（如 `Parameter Settings → ADC_Settings → Scan Conversion Mode`）和**准确的参数值**（如 `8 Bit` 而非 `8`，`Full-Duplex Master` 而非 `Master`）。对非默认值的参数要特别标注说明。

## FreeRTOS 配置

当需要修改 FreeRTOS 内核配置（包括但不限于 `FreeRTOSConfig.h` 中的堆大小、任务优先级、tick 频率等参数，以及任务创建、信号量、队列等 CMSIS RTOS 对象）时，**不要直接修改代码**，而是提醒用户在 STM32CubeMX 的 FREERTOS 选项卡中进行配置，重新生成代码后再适配。`app_freertos.c` 中 `USER CODE` 保护区内的业务逻辑代码可以修改。

## 项目文档

以下文档文件必须与实际代码保持同步，修改相关代码时须同步更新：

- **`docs/FREERTOS_TASKS.md`** — FreeRTOS 任务列表（名称、职责、周期、优先级、栈大小）。新增/修改任务时必须更新。
- **`docs/COMM_PROTOCOL.md`** — 串口通信协议说明。修改协议或通道定义时必须更新。

## Git 提交

**每次代码修改完成后必须立即提交到 Git**，防止代码丢失。提交信息使用中文，简洁描述修改内容和原因。在大型调试/重构任务中，每完成一个有意义的阶段性成果就提交一次。

```bash
# 提交示例
git add -A
git commit -m "fix: 修正 M2 编码器方向为 enc_dir=+1"
```

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **STM32G431Demo** (1087 symbols, 1522 relationships, 20 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> If any GitNexus tool warns the index is stale, run `npx gitnexus analyze` in terminal first.

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `gitnexus_impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `gitnexus_detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `gitnexus_query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `gitnexus_context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `gitnexus_impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename` which understands the call graph.
- NEVER commit changes without running `gitnexus_detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/STM32G431Demo/context` | Codebase overview, check index freshness |
| `gitnexus://repo/STM32G431Demo/clusters` | All functional areas |
| `gitnexus://repo/STM32G431Demo/processes` | All execution flows |
| `gitnexus://repo/STM32G431Demo/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
