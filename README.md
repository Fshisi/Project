# TC300PGS 手写空工程

Keil入口：[Project/TC300PGS.uvprojx](Project/TC300PGS.uvprojx)，目标名 `TC300PGS`。
VSCode入口：工作区根目录的 `TC300PGS.code-workspace`。

本工程从TC300PG_GJB8114的编译和启动环境建立，保留独立的CMSIS、HAL、启动向量、异常处理、HAL SysTick及HSI/PLL标称64MHz初始化。STM32F103C8、Keil ARMCC 5与原工程一致。只初始化HAL和时钟，**没有采样、串口输出、EEPROM写入、TIM3业务或自动启动看门狗**。默认不会占用PA10做测时输出。

从 `User/Core/main.c` 两处TODO开始手写。`User/App`、`User/Drivers`、`User/Storage`为空目录，新增C/H后应添加到对应Keil分组。`stm32f1xx_it.c`负责内核异常及HAL SysTick，自己的USART1/TIM3中断可放在驱动中，避免同名重复定义。

芯片物理Flash为64KB，工程应用链接区按F版任务书4.6限制为 `0x08000000` 起、大小 `0x7800`（30KB），RAM物理上限20KB。Keil的输出为 `Project/Objects/TC300PGS.hex`；这是空框架镜像，不能用作完整TC300PGS业务固件。

手写顺序建议：UART/I2C驱动 → 可靠的时间基准与主循环喂狗 → NSA2860初始化/读数 → F版压高及滤波 → 17字节组帧 → 定时发送/共模BIT → 三包标定及事务存储。对应参考实现单独放在 `../TC300PGS_Reference`，本目录不包含其业务源码。
