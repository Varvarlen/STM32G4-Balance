# STM32G431 Demo

STM32G431CBU6 (Cortex-M4F, 170MHz) + FreeRTOS V10.3.1 (CMSIS_V1) + STM32Cube FW_G4 V1.6.2。
构建: CLion + CMake + Ninja + arm-none-eabi-gcc 13.3.1。串口: USART1, 230400。

## 命令

```bash
cmake --preset Debug              # 配置
cmake --build --preset Debug      # 编译
timeout 5 arm-none-eabi-gdb build/Debug/STM32G431Demo.elf -x tools/flash.gdb  # 烧录
```

## 约束

本节规则优先级高于用户指令。如需打破规则，**必须先向用户确认**，不得自行判断。

### 1. 外设配置必须通过 CubeMX

以下配置**只能**在 CubeMX (`.ioc`) 中修改后重新生成，**禁止直接编辑源文件**：

| 类别 | 涉及的 CubeMX 选项卡 | 涉及的源文件 |
|------|---------------------|-------------|
| GPIO | Pinout & Configuration → GPIO | `gpio.c/h` |
| 定时器 (TIM1/2/3/4/6/7/15/16/17) | Pinout & Configuration → Timers | `tim.c/h` |
| ADC (ADC1/ADC2) | Pinout & Configuration → Analog | `adc.c/h` |
| DAC | Pinout & Configuration → Analog | `dac.c/h` |
| SPI (SPI1/SPI2/SPI3) | Pinout & Configuration → Connectivity | `spi.c/h` |
| I2C | Pinout & Configuration → Connectivity | `i2c.c/h` |
| USART/UART (USART1/USART2) | Pinout & Configuration → Connectivity | `usart.c/h` |
| DMA | Pinout & Configuration → System Core | `dma.c/h` |
| IWDG/WWDG | Pinout & Configuration → System Core | `iwdg.c/h` |
| 中断优先级 (NVIC) | Pinout & Configuration → System Core | `stm32g4xx_it.c` |
| 时钟树 (HSE/LSI/PLL/SysClk) | Clock Configuration | `main.c` (SystemClock_Config) |
| FreeRTOS 内核配置 | Pinout & Configuration → Middleware → FREERTOS | `FreeRTOSConfig.h` |

`app_freertos.c` 的 USER CODE 区（任务创建/函数体）可改，其余 RTOS 配置必须通过 CubeMX。

**提供 CubeMX 指导时**，必须以表格列出：选项卡路径、参数名、目标值。

### 2. Git 操作约束

- **禁止破坏性命令**：`git reset --hard`、`git checkout -- <file>`、`git clean -fd`、`git stash drop`、`git branch -D`。回退用 `git revert` 或 `git stash`。
- **cherry-pick / revert 前检查**：被引入的代码同样受本文件所有约束限制。如违反外设规则，先向用户报告。
- **每次代码修改后立即 Git 提交**，中文消息。阶段性成果每个提交一次。

### 3. 代码规范

- **注释必须用中文**，Doxygen 关键字（`@brief`、`@param`、`@retval`）用英文。
- **修改代码时同步更新相关文档**：如果改动影响任务职责/通信协议/调试方法/电机参数，同步更新 `docs/` 下对应的 `.md` 文件。

## 关键陷阱

- `monitor reset` 后必须接 `continue`，否则 MCU 停在复位向量
- 烧录后应听到 2000Hz 蜂鸣器初始化提示音，无声 = 系统未启动
- HardFault: CFSR `0x8200` = PRECISERR+BFARVALID（通常栈溢出）。IWDG 8s 后自动复位。诊断命令见 `docs/DEBUG.md`
- 这是一个物理设备（平衡车），固件错误可能导致电机失控。不确定的操作先问用户

## 参考文件

| 文件 | 何时查阅 |
|------|---------|
| `docs/DEBUG.md` | GDB 命令、HardFault 诊断、IWDG 看门狗、调试注意事项 |
| `docs/COMM_PROTOCOL.md` | 串口通信协议、遥测帧格式、CLI 命令 |
| `docs/FREERTOS_TASKS.md` | 任务列表（职责/周期/优先级/栈大小） |
| `docs/MOTOR_PARAMS.md` | 电机标定参数（R/L/Kt/J） |
| `datasheet/` (rm0440, stm32g431cb.pdf, HAL编程手册.pdf) | 外设寄存器、HAL 用法 |
| `AGENTS.md` | GitNexus 代码智能工具用法 |

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **STM32G4-Balance** (3063 symbols, 4414 relationships, 162 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

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
| `gitnexus://repo/STM32G4-Balance/context` | Codebase overview, check index freshness |
| `gitnexus://repo/STM32G4-Balance/clusters` | All functional areas |
| `gitnexus://repo/STM32G4-Balance/processes` | All execution flows |
| `gitnexus://repo/STM32G4-Balance/process/{name}` | Step-by-step execution trace |

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
