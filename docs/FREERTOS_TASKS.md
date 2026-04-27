# FreeRTOS 任务列表

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| defaultTask | StartDefaultTask | 1ms | 128 | Normal | 串口回显 |
| encoderTask | TaskEncoderReport | 500ms | 64 | BelowNormal | SPI 读 MT6701 |
| adcTask | TaskADCMonitor | 100ms | 64 | Normal | ADC DMA 读取 INA240 |
| mpuTask | TaskMPU6500 | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |
| sixStepTask | TaskSixStep | 1ms | 128 | Normal | 6 步换相驱动 M1（已注释，切换模式时使用） |
| voltageSineTask | TaskVoltageSine | 1ms | 256 | Normal | SVPWM 电压模式正弦波开环驱动 M1（已注释，切换模式时使用） |
| currentLoopTask | TaskCurrentLoop | 100ms | 128 | Normal | FOC 电流闭环监控（实际 FOC 由 ADC ISR 10kHz 驱动） |
