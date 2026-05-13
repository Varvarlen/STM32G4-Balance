# STM32G431 Demo

STM32G431CBU6 (Cortex-M4F, 170MHz) + FreeRTOS V10.3.1 (CMSIS_V1) + STM32Cube FW_G4 V1.6.2。
构建: CLion + CMake + Ninja + arm-none-eabi-gcc 13.3.1。串口: USART1, 230400。

## 命令

```bash
cmake --preset Debug           # 配置
cmake --build --preset Debug    # 编译
timeout 5 arm-none-eabi-gdb build/Debug/STM32G431Demo.elf -x tools/flash.gdb  # 烧录
```

## 约束

- **注释必须用中文**。Doxygen 关键字（`@brief`、`@param`、`@retval`）保留英文。
- **不要直接改外设配置**。GPIO/定时器/ADC/DAC/SPI/I2C/USART/DMA/中断优先级/时钟树 → 提醒用户通过 STM32CubeMX (`.ioc`) 配置后重新生成。提供 CubeMX 指导时以表格列出完整参数路径和准确值。
- **不要直接改 FreeRTOS 内核配置**。堆大小/任务优先级/tick 频率/RTOS 对象 → 通过 CubeMX FREERTOS 选项卡配置。`app_freertos.c` USER CODE 区可改。
- **修改代码时同步更新相关文档**（`docs/` 目录下所有 `.md` 文件），保持代码与文档一致，避免腐化。
- **每次代码修改后立即 Git 提交**，中文消息。大型任务每个阶段性成果提交一次。

## 关键陷阱

- `monitor reset` 后必须接 `continue`，否则 MCU 停在复位向量
- 烧录后应听到 2000Hz 蜂鸣器初始化提示音，无声 = 系统未启动
- HardFault: CFSR `0x8200` = PRECISERR+BFARVALID（通常栈溢出）。诊断命令见 `docs/DEBUG.md`

## 参考文件

| 文件 | 何时查阅 |
|------|---------|
| `docs/DEBUG.md` | GDB 命令、HardFault 诊断、调试注意事项 |
| `datasheet/` (rm0440, stm32g431cb.pdf, HAL编程手册.pdf) | 外设寄存器、HAL 用法 |
| `AGENTS.md` | GitNexus 代码智能工具用法 |
