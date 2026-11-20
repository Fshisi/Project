# TC300PGS DMA/ADC/串口 HAL 模板代码

适用工程：本目录 Keil 工程（STM32F103C8，HAL 库，主频 64MHz，PCLK2 = 64MHz）。
`stm32f1xx_hal_conf.h` 中 `HAL_ADC_MODULE_ENABLED`、`HAL_DMA_MODULE_ENABLED`、
`HAL_UART_MODULE_ENABLED` 均已默认打开，无需再改。

**代码风格遵循 `TC300PG_GJB8114` 约定**：

- 函数命名：模块前缀 + 帕斯卡（同 `UART_Init`/`NSA2860_ReadSample`），静态变量 `s_` 前缀；
- 所有 `if/else` 带大括号，else 分支显式写出（无事可做写 `; /* no deal with */`）；
- 单出口：状态变量收集结果，函数末尾统一返回，void 函数末尾显式 `return;`；
- 局部变量声明置于函数顶部（C89 习惯）；字面量带 `U` 后缀；宏定义带显式强转；
- 忽略返回值处 `(void)` 显式消除；头文件横幅注释 + 每个对外函数一段说明注释；
- 中断向量全部放自己的驱动文件，不与 `stm32f1xx_it.c` 同名重复定义。

文件规划（放 `User/Drivers`，与既有 uart.c 等并列；为避免与本工程已有 uart.c 冲突，
串口驱动沿用 usart_dma 文件名）：

| 文件 | 职责 |
|------|------|
| `hw_dma.c/h` | 公共 DMA 封装：时钟统一使能、通道占用登记、统一配置入口 |
| `hw_adc.c/h` | ADC 驱动：DMA / 非 DMA 两方案，用 `HW_ADC_USE_DMA` 切换 |
| `usart_dma.c/h` | 串口驱动：TX / RX 可独立选 DMA，用 `HW_USART_TX_USE_DMA`、`HW_USART_RX_USE_DMA` 切换 |

---

## 1. 时钟约束（先看这个，容易踩坑）

ADC 时钟来自 PCLK2，**不得超过 14MHz**。本工程 PCLK2 = 64MHz，只能选：

| 分频 | ADC 时钟 | 结论 |
|------|---------|------|
| PCLK2/2 | 32MHz | 超限，禁止 |
| PCLK2/4 | 16MHz | 超限，禁止 |
| PCLK2/6 | 10.67MHz | **推荐** |
| PCLK2/8 | 8MHz | 可用，转换略慢 |

模板统一使用 `ADC_CLOCK_SYNC_PCLK_DIV6`。

F1 的 DMA1 七个通道与外设是**硬绑定**的，分配不自由：

| DMA1 通道 | 本工程的使用者 |
|-----------|---------------|
| Ch1 | ADC1（hw_adc） |
| Ch2/Ch3 | 空闲（SPI1、TIM2/TIM4 备用） |
| Ch4/Ch5 | USART1_TX / USART1_RX（usart_dma） |
| Ch6/Ch7 | 空闲（USART2/3、TIM3 备用） |

---

## 2. 公共 DMA 封装：hw_dma.c/h

**定位：薄分发层，不做通用传输 API。** F1 通道与外设硬绑定，能抽象的只有三件事：
时钟使能（一处）、通道占用登记（防两个驱动抢同一通道）、统一配置入口。
方向/对齐/模式差异太大，仍由各驱动自己填 `DMA_InitTypeDef`。

### hw_dma.h

```c
#ifndef HW_DMA_H
#define HW_DMA_H

/* ==========================================================================
 * hw_dma.h - DMA1 公共封装（时钟统一使能 + 通道占用登记 + 统一配置入口）
 * ==========================================================================
 * F1 的 DMA1 七个通道与外设硬绑定：ADC1 用 Ch1，USART1_TX 用 Ch4，
 * USART1_RX 用 Ch5，详见各驱动。本模块不做通用传输 API；
 * 方向/对齐/模式等差异由属主驱动自行填写。
 * 初始化与登记实现见 hw_dma.c。
 * ========================================================================== */

#include "main.h"

/** @brief 配置并登记一个 DMA1 通道。
 * @note ch_instance 为 DMA1_Channel1~7，哪个外设用哪个通道由硬件绑定决定；
 *       handle 为属主驱动自己的句柄（须 static 保证生命周期）；
 *       init 由属主填好方向/对齐/模式。
 *       返回 HAL_BUSY 表示通道已被其他驱动登记，禁止重复占用。
 */
HAL_StatusTypeDef HWDMA_Config(DMA_Channel_TypeDef *ch_instance,
                               DMA_HandleTypeDef *handle,
                               const DMA_InitTypeDef *init);

/** @brief 取某通道当前登记的句柄，供 IRQHandler 分发使用。
 * @note 未登记或实例非法时返回 NULL。
 */
DMA_HandleTypeDef *HWDMA_GetHandle(DMA_Channel_TypeDef *ch_instance);

#endif
```

### hw_dma.c

```c
/* DMA1 公共封装：时钟统一使能，通道占用登记防冲突。 */

#include "main.h"
#include "hw_dma.h"

#define HWDMA_CH_COUNT ((uint8_t)7U)   /* DMA1 共 7 个通道 */

typedef struct hw_dma_chan {
    DMA_Channel_TypeDef *instance;
    DMA_HandleTypeDef *handle;         /* 指向属主驱动的句柄，不拥有 */
} HWDMA_Chan;

static HWDMA_Chan s_chan_map[HWDMA_CH_COUNT] = {0};

/* 通道实例转登记表下标；非法实例返回 HWDMA_CH_COUNT。 */
static uint8_t HWDMA_ChanIndex(DMA_Channel_TypeDef *ch)
{
    uint8_t idx = HWDMA_CH_COUNT;      /* 默认按非法处理 */

    if (ch == DMA1_Channel1)
    {
        idx = 0U;
    }
    else if (ch == DMA1_Channel2)
    {
        idx = 1U;
    }
    else if (ch == DMA1_Channel3)
    {
        idx = 2U;
    }
    else if (ch == DMA1_Channel4)
    {
        idx = 3U;
    }
    else if (ch == DMA1_Channel5)
    {
        idx = 4U;
    }
    else if (ch == DMA1_Channel6)
    {
        idx = 5U;
    }
    else if (ch == DMA1_Channel7)
    {
        idx = 6U;
    }
    else
    {
        ; /* no deal with */
    }

    return idx;
}

HAL_StatusTypeDef HWDMA_Config(DMA_Channel_TypeDef *ch_instance,
                               DMA_HandleTypeDef *handle,
                               const DMA_InitTypeDef *init)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t idx = HWDMA_ChanIndex(ch_instance);

    if ((handle == NULL) || (init == NULL) || (idx >= HWDMA_CH_COUNT))
    {
        status = HAL_ERROR;            /* 参数或实例不合法 */
    }
    else if (s_chan_map[idx].handle != NULL)
    {
        status = HAL_BUSY;             /* 通道已被占用，硬件绑定关系冲突 */
    }
    else
    {
        __HAL_RCC_DMA1_CLK_ENABLE();   /* 幂等，多次调用无害，统一收口在此 */

        handle->Instance = ch_instance; /* 先定实例再 init */
        if (HAL_DMA_Init(handle) != HAL_OK)
        {
            status = HAL_ERROR;
        }
        else
        {
            s_chan_map[idx].instance = ch_instance;
            s_chan_map[idx].handle = handle;
        }
    }

    return status;
}

DMA_HandleTypeDef *HWDMA_GetHandle(DMA_Channel_TypeDef *ch_instance)
{
    DMA_HandleTypeDef *handle = NULL;
    uint8_t idx = HWDMA_ChanIndex(ch_instance);

    if (idx < HWDMA_CH_COUNT)
    {
        handle = s_chan_map[idx].handle;
    }
    else
    {
        ; /* no deal with */
    }

    return handle;
}
```

谁的通道谁开中断：IRQHandler 里用 `HWDMA_GetHandle(DMA1_Channelx)` 拿到句柄再调
`HAL_DMA_IRQHandler`，中断向量仍由属主驱动实现（不与 `stm32f1xx_it.c` 重名）。

---

## 3. ADC 统一模板：hw_adc.h 一个宏选 DMA / 非DMA

- `HW_ADC_USE_DMA = 1`：ADC+DMA 连续多通道（PA3/CH3 + PA4/CH4），DMA 配置走 `HWDMA_Config`。
- `HW_ADC_USE_DMA = 0`：不加 DMA，阻塞方式读单通道。

两方案共用 `ADC_Init()` 和 `ADC_ReadRaw/ADC_ReadMv(ch_index)`，上层切方案只改一个宏。

### hw_adc.h

```c
#ifndef HW_ADC_H
#define HW_ADC_H

/* ==========================================================================
 * hw_adc.h - ADC1 采样（12 位，右对齐，PA3=CH3 / PA4=CH4）
 * ==========================================================================
 * HW_ADC_USE_DMA 选方案：1 = DMA 连续多通道，init 后硬件后台自动刷新；
 *                        0 = 阻塞单通道，读一次转一次。
 * 两方案接口同名同参，上层切换只改本宏。实现见 hw_adc.c。
 * ========================================================================== */

#include "main.h"

/* ============ 方案选择：1=ADC+DMA多通道；0=不加DMA，阻塞单通道 ============ */
#define HW_ADC_USE_DMA      0U

/* 公共配置：VREF+ 实测更准的话改这里（单位 mV）。C8 封装 VREF+ 内部接 VDDA。 */
#define HW_ADC_VREF_MV      ((uint32_t)3300U)
#define HW_ADC_SAMPLING     ADC_SAMPLETIME_239CYCLES_5

#if HW_ADC_USE_DMA

/* ---- DMA方案：通道表，引脚/通道/Rank 一一对应，按需增删 ---- */
#define HW_ADC_DMA_CH_NUM   ((uint8_t)2U)
#define HW_ADC_CH1_PORT     GPIOA
#define HW_ADC_CH1_PIN      GPIO_PIN_3
#define HW_ADC_CH1          ADC_CHANNEL_3
#define HW_ADC_CH2_PORT     GPIOA
#define HW_ADC_CH2_PIN      GPIO_PIN_4
#define HW_ADC_CH2          ADC_CHANNEL_4

#else

/* ---- 阻塞方案：单通道。换通道时同步改这三个宏。 ---- */
#define HW_ADC_CH1_PORT     GPIOA
#define HW_ADC_CH1_PIN      GPIO_PIN_3
#define HW_ADC_CH1          ADC_CHANNEL_3

#endif /* HW_ADC_USE_DMA */

/** @brief 初始化 ADC1。DMA 方案 init 返回后即开始后台连续采集。
 * @note 内部完成 GPIO/ADC/DMA 配置与 F1 必做的上电校准；
 *       DMA 方案要求 DMA1_Channel1 未被其他驱动占用，否则返回 HAL_BUSY。
 */
HAL_StatusTypeDef ADC_Init(void);

/** @brief 读原始码值 0~4095。
 * @note ch_index 从 0 起对应 CH1/CH2...（DMA 方案）；
 *       阻塞方案忽略该参数（传 0U），每次调用现场转换约 24us，越界返回 0。
 */
uint16_t ADC_ReadRaw(uint8_t ch_index);

/** @brief 原始码值换算毫伏，基准为 HW_ADC_VREF_MV。 */
uint32_t ADC_ReadMv(uint8_t ch_index);

#endif
```

### hw_adc.c

```c
/* ADC1 采样，PA3/PA4。DMA 方案后台连续采集，阻塞方案读一次转一次。 */

#include "main.h"
#include "hw_adc.h"
#include "hw_dma.h"

static ADC_HandleTypeDef s_hadc1 = {0};

#if HW_ADC_USE_DMA
static DMA_HandleTypeDef s_hdma_adc = {0};
/* volatile：该数组由 DMA 硬件后台改写，CPU 只读。 */
static volatile uint16_t s_adc_raw[HW_ADC_DMA_CH_NUM] = {0}; /* 下标即通道序号-1 */
#endif

/* 原始码值换算毫伏。4095*3300=13513500，uint32 不会溢出。 */
static uint32_t ADC_RawToMv(uint16_t raw)
{
    return ((uint32_t)raw * HW_ADC_VREF_MV) / 4095U;
}

#if HW_ADC_USE_DMA

/* ================== 方案A：ADC+DMA 连续多通道 ================== */

/* 使能通道所在 GPIO 端口时钟。C8 封装 ADC 引脚只有 PA0~PA7/PB0/PB1，
 * 所以只覆盖 A/B 两个端口，其余端口视为配置错误。 */
static void ADC_GpioClkEnable(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else
    {
        ; /* 非法端口，静默忽略；检查清单第 3 条已约束引脚范围 */
    }

    return;
}

/* 每个通道的引脚各自初始化：通道表允许 CH1/CH2 在不同端口（如 PA3/PB0），
 * 不能合并成一次 HAL_GPIO_Init。同端口重复调用一次也无害（同参数重配同一引脚）。 */
static void ADC_GpioSetup(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC_GpioClkEnable(HW_ADC_CH1_PORT);
    ADC_GpioClkEnable(HW_ADC_CH2_PORT);

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = HW_ADC_CH1_PIN;
    HAL_GPIO_Init(HW_ADC_CH1_PORT, &gpio);

    gpio.Pin = HW_ADC_CH2_PIN;
    HAL_GPIO_Init(HW_ADC_CH2_PORT, &gpio);

    return;
}

/* ADC1 硬绑定 DMA1_Channel1，登记与配置交给 HWDMA_Config。 */
static HAL_StatusTypeDef ADC_DmaSetup(void)
{
    HAL_StatusTypeDef status;
    DMA_InitTypeDef dinit = {0};

    dinit.Direction = DMA_PERIPH_TO_MEMORY;
    dinit.PeriphInc = DMA_PINC_DISABLE;
    dinit.MemInc = DMA_MINC_ENABLE;          /* 内存地址递增，按通道存。 */
    dinit.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dinit.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dinit.Mode = DMA_CIRCULAR;               /* 循环模式，采满自动覆盖。 */
    dinit.Priority = DMA_PRIORITY_HIGH;

    status = HWDMA_Config(DMA1_Channel1, &s_hdma_adc, &dinit);
    if (status == HAL_OK)
    {
        __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc);
    }
    else
    {
        ; /* HAL_BUSY=通道登记冲突，沿用 status */
    }

    return status;
}

/* ADC 本体 + 通道序列 + F1 上电校准。 */
static HAL_StatusTypeDef ADC_AdcSetup(void)
{
    HAL_StatusTypeDef status;
    ADC_ChannelConfTypeDef ch = {0};

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;         /* 多通道必须扫描。 */
    s_hadc1.Init.ContinuousConvMode = ENABLE;    /* 一轮序列转完硬件自动开下一轮，无需软件再触发，配合DMA循环实现后台采集。 */
    s_hadc1.Init.DiscontinuousConvMode = DISABLE; /* 不拆散扫描序列：一次触发转完全部Rank，本工程不用断续模式。 */
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;        /* 低位对齐，0~4095。 */
    s_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;  /* 软件触发。 */
    s_hadc1.Init.NbrOfConversion = HW_ADC_DMA_CH_NUM;

    status = HAL_ADC_Init(&s_hadc1);
    if (status == HAL_OK)
    {
        /* Rank = 转换序列中的槽位号，扫描模式下ADC按Rank1→Rank2依次转换，
         * 转换结果按此顺序落入s_adc_raw，所以Rank顺序必须与数组下标一致。 */
        ch.Channel = HW_ADC_CH1;
        ch.Rank = ADC_REGULAR_RANK_1;                    /* CH3排第1个转换，结果落s_adc_raw[0]。 */
        ch.SamplingTime = HW_ADC_SAMPLING;
        status = HAL_ADC_ConfigChannel(&s_hadc1, &ch);
    }
    else
    {
        ; /* no deal with */
    }
    if (status == HAL_OK)
    {
        ch.Channel = HW_ADC_CH2;
        ch.Rank = ADC_REGULAR_RANK_2;                    /* CH4排第2个转换，结果落s_adc_raw[1]。 */
        status = HAL_ADC_ConfigChannel(&s_hadc1, &ch);
    }
    else
    {
        ; /* no deal with */
    }
    if (status == HAL_OK)
    {
        status = HAL_ADCEx_Calibration_Start(&s_hadc1);  /* F1 必做的上电校准。 */
    }
    else
    {
        ; /* no deal with */
    }

    return status;
}

HAL_StatusTypeDef ADC_Init(void)
{
    HAL_StatusTypeDef status;

    ADC_GpioSetup();
    status = ADC_DmaSetup();
    if (status == HAL_OK)
    {
        status = ADC_AdcSetup();
    }
    else
    {
        ; /* no deal with */
    }
    if (status == HAL_OK)
    {
        /* 启动后台连续采集，此后 s_adc_raw 由硬件持续刷新。 */
        status = HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_raw,
                                   (uint32_t)HW_ADC_DMA_CH_NUM);
    }
    else
    {
        ; /* no deal with */
    }

    return status;
}

uint16_t ADC_ReadRaw(uint8_t ch_index)
{
    uint16_t raw = 0U;

    if (ch_index < HW_ADC_DMA_CH_NUM)
    {
        raw = s_adc_raw[ch_index];
    }
    else
    {
        ; /* 越界，返回 0 */
    }

    return raw;
}

#else

/* ================== 方案B：不加DMA，阻塞单通道 ================== */

static HAL_StatusTypeDef ADC_AdcSetup(void)
{
    HAL_StatusTypeDef status;
    GPIO_InitTypeDef gpio = {0};
    ADC_ChannelConfTypeDef ch = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = HW_ADC_CH1_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HW_ADC_CH1_PORT, &gpio);

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;        /* 单通道不扫描。 */
    s_hadc1.Init.ContinuousConvMode = DISABLE;           /* 读一次转一次：转完一轮停下等软件Start。 */
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;        /* 单通道无序列可拆，固定关闭。 */
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    s_hadc1.Init.NbrOfConversion = 1U;

    status = HAL_ADC_Init(&s_hadc1);
    if (status == HAL_OK)
    {
        ch.Channel = HW_ADC_CH1;
        ch.Rank = ADC_REGULAR_RANK_1;   /* 单通道也固定填第1个槽位，结果从Rank1出。 */
        ch.SamplingTime = HW_ADC_SAMPLING;
        status = HAL_ADC_ConfigChannel(&s_hadc1, &ch);
    }
    else
    {
        ; /* no deal with */
    }
    if (status == HAL_OK)
    {
        status = HAL_ADCEx_Calibration_Start(&s_hadc1);
    }
    else
    {
        ; /* no deal with */
    }

    return status;
}

HAL_StatusTypeDef ADC_Init(void)
{
    HAL_StatusTypeDef status;

    status = ADC_AdcSetup();
    if (status == HAL_OK)
    {
        ; /* 初始化完成，等待读取。 */
    }
    else
    {
        ; /* no deal with */
    }

    return status;
}

uint16_t ADC_ReadRaw(uint8_t ch_index)
{
    uint16_t raw = 0U;

    (void)ch_index;                          /* 单通道方案无通道表，忽略该参数。 */

    if (HAL_ADC_Start(&s_hadc1) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&s_hadc1, 10U) == HAL_OK)
        {
            raw = (uint16_t)HAL_ADC_GetValue(&s_hadc1);
        }
        else
        {
            ; /* 等待超时，保持 0 */
        }
        (void)HAL_ADC_Stop(&s_hadc1);
    }
    else
    {
        ; /* 启动失败，保持 0 */
    }

    return raw;
}

#endif /* HW_ADC_USE_DMA */

uint32_t ADC_ReadMv(uint8_t ch_index)
{
    return ADC_RawToMv(ADC_ReadRaw(ch_index));
}
```

### 扩展：缓冲放大做硬件滑动平均（F 版压高滤波可用）

默认缓冲 `s_adc_raw[2]` 只存"最新一轮"快照，循环模式每轮覆盖。需要历史采样做滑动平均时，
把缓冲放大为 `通道数 × 平均深度`，DMA 写满一轮缓冲才算一轮，历史就由硬件自动攒好，CPU 零开销：

```c
/* hw_adc.h 的 DMA 方案配置里增加一个宏： */
#define HW_ADC_AVG_DEPTH    ((uint8_t)8U)   /* 滑动平均深度，轮数 */

/* hw_adc.c 的 DMA 方案里：缓冲和启动长度同步放大。 */
static volatile uint16_t s_adc_raw[HW_ADC_DMA_CH_NUM * HW_ADC_AVG_DEPTH] = {0};

/* ADC_Init 末尾的启动改为： */
status = HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_raw,
                           (uint32_t)(HW_ADC_DMA_CH_NUM * HW_ADC_AVG_DEPTH));
```

内存布局（CH_NUM=2、DEPTH=8 时共 16 格）：

```
下标:  [0]  [1] | [2]  [3] | [4]  [5] | ...
通道:  CH3  CH4 | CH3  CH4 | CH3  CH4 | ...
       └─第1轮─┘ └─第2轮─┘ └─第3轮─┘
```

CH3 的历史点在偶数下标 `[0][2][4]...[14]`，CH4 在奇数下标 `[1][3][5]...[15]`。
读取与平均：

```c
/* 取某通道最近 depth 个原始码值并滑动平均（禁止除法之外的浮点，GJB 同款整数运算）。 */
uint16_t ADC_ReadAvgRaw(uint8_t ch_index)
{
    uint32_t sum = 0U;
    uint16_t avg = 0U;
    uint8_t i;
    uint16_t read_pos;

    if (ch_index < HW_ADC_DMA_CH_NUM)
    {
        for (i = 0U; i < HW_ADC_AVG_DEPTH; ++i)
        {
            read_pos = (uint16_t)ch_index + ((uint16_t)i * HW_ADC_DMA_CH_NUM);
            sum += s_adc_raw[read_pos];
        }
        avg = (uint16_t)(sum / HW_ADC_AVG_DEPTH);
    }
    else
    {
        ; /* 越界，返回 0 */
    }

    return avg;
}
```

要点：

1. **布局是按轮交错，不是按通道分块**：DMA 按 Rank 顺序连续写，CH3/CH4 永远相邻交替，
   取历史点必须用 `ch_index + i*CH_NUM` 跨步索引，不能按半段切数组；
2. **"一轮"的定义变了**：转满 `CH_NUM*DEPTH` 格才算一轮，半传输/全传输中断时机随之变化；
   纯轮询读数不受影响，用了转换完成中断的要重新对齐节奏；
3. **RAM 账**：2通道×8深×uint16_t = 32 字节，相对 20KB 上限无压力；深度按滤波需求定，
   不要拍脑袋放大——比如 256 深就是 1KB；
4. **滑动平均的局限**：它压随机噪声有效，对阶跃（真实压力突变）会平滑掉前沿，压高算法
   若需要快速跟随，改用一阶 IIR 或限幅滤波，不在这套 DMA 缓冲上硬做。

---

## 4. 串口模板：TX / RX 可独立选 DMA（usart_dma.c/h）

头文件里两个开关独立控制收发方向：

- `HW_USART_TX_USE_DMA = 1`：发送走 DMA1_Channel4，`USART_Send` 立即返回；
  设 0 则回退为阻塞发送 `HAL_UART_Transmit`。
- `HW_USART_RX_USE_DMA = 1`：接收走 DMA1_Channel5 循环模式 + 空闲中断切帧（变长帧适用）；
  设 0 则回退为逐字节中断接收（RXNE + 回调重挂），同样不用阻塞等。

概念提醒：**空闲中断不是用来"调用 DMA"的，DMA 一直在收，IDLE 只负责报告"一帧收完了"**。
只有 DMA 没有 IDLE，变长帧不知道何时算完；只有 IDLE 没有 DMA，每字节仍进中断，CPU 没省到。
**定长帧**（如固定 17 字节）即使开 DMA 也可不用 IDLE，见本节末尾。

### usart_dma.h

```c
#ifndef USART_DMA_H
#define USART_DMA_H

/* ==========================================================================
 * usart_dma.h - USART1 串口（PA9=TX / PA10=RX，8N1，波特率由 Init 传入）
 * ==========================================================================
 * HW_USART_TX_USE_DMA 选发送方式：1 = DMA1_Channel4 发送，立即返回；
 *                                  0 = 阻塞发送（同 GJB8114 uart.c 方式）。
 * HW_USART_RX_USE_DMA 选接收方式：1 = DMA1_Channel5 循环接收 + IDLE 切帧；
 *                                  0 = 逐字节中断接收，回调重挂。
 * 四种组合共用同一套接口，实现见 usart_dma.c。
 * ========================================================================== */

#include "main.h"

/* ============ 收发方向各自选：1=用DMA；0=不用DMA（阻塞发/中断收） ============ */
#define HW_USART_TX_USE_DMA     1U
#define HW_USART_RX_USE_DMA     0U

/* RX 用 DMA 时的缓冲长度，须大于最长帧。 */
#define USART_DMA_RX_BUF_SIZE   ((uint16_t)64U)

/** @brief 初始化 GPIO / USART1 / NVIC，并按宏配置发送通道。
 * @note 8N1；接收通道（DMA 或中断）由 USART_StartReceive 单独启动。
 */
HAL_StatusTypeDef USART_Init(uint32_t baud);

/** @brief 发送 n 字节。DMA 方案立即返回（上一帧未发完返回 HAL_BUSY）；
 * @note 非 DMA 方案阻塞发送，100ms 超时保护。buf 为 NULL 返回 HAL_ERROR。
 */
HAL_StatusTypeDef USART_Send(const uint8_t *data, uint16_t len);

/** @brief 启动接收。DMA 方案启动循环接收并使能 IDLE；非 DMA 方案挂 1 字节中断。 */
HAL_StatusTypeDef USART_StartReceive(void);

#endif
```

### usart_dma.c

```c
/* USART1，PA9/PA10，8N1。收发方向各自可选 DMA。 */

#include "main.h"
#include "usart_dma.h"
#include "hw_dma.h"

static UART_HandleTypeDef s_huart1 = {0};

#if HW_USART_TX_USE_DMA
static DMA_HandleTypeDef s_hdma_tx = {0};            /* DMA1_Channel4 */
#endif
#if HW_USART_RX_USE_DMA
static DMA_HandleTypeDef s_hdma_rx = {0};            /* DMA1_Channel5 */
static uint8_t s_rx_buf[USART_DMA_RX_BUF_SIZE] = {0};
#else
static uint8_t s_rx_byte = 0U;                       /* 不用DMA：逐字节中断收 */
#endif

static HAL_StatusTypeDef USART_GpioUartInit(uint32_t baud)
{
    HAL_StatusTypeDef status;
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_9;                           /* TX：复用推挽 50MHz。 */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;                          /* RX：浮空输入。 */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_huart1.Instance = USART1;
    s_huart1.Init.BaudRate = baud;
    s_huart1.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart1.Init.StopBits = UART_STOPBITS_1;
    s_huart1.Init.Parity = UART_PARITY_NONE;
    s_huart1.Init.Mode = UART_MODE_TX_RX;
    s_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    status = HAL_UART_Init(&s_huart1);
    if (status == HAL_OK)
    {
        HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else
    {
        ; /* no deal with */
    }

    return status;
}

#if HW_USART_TX_USE_DMA
/* USART1_TX 硬绑定 DMA1_Channel4，登记与配置交给 HWDMA_Config。 */
static HAL_StatusTypeDef USART_TxDmaSetup(void)
{
    HAL_StatusTypeDef status;
    DMA_InitTypeDef dinit = {0};

    dinit.Direction = DMA_MEMORY_TO_PERIPH;          /* 内存→DR，发帧用。 */
    dinit.PeriphInc = DMA_PINC_DISABLE;
    dinit.MemInc = DMA_MINC_ENABLE;
    dinit.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dinit.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dinit.Mode = DMA_NORMAL;                         /* 发完即止，普通模式。 */
    dinit.Priority = DMA_PRIORITY_MEDIUM;

    status = HWDMA_Config(DMA1_Channel4, &s_hdma_tx, &dinit);
    if (status == HAL_OK)
    {
        __HAL_LINKDMA(&s_huart1, hdmatx, s_hdma_tx);
    }
    else
    {
        ; /* HAL_BUSY=通道登记冲突，沿用 status */
    }

    return status;
}
#endif

HAL_StatusTypeDef USART_Init(uint32_t baud)
{
    HAL_StatusTypeDef status;

    status = USART_GpioUartInit(baud);
    if (status == HAL_OK)
    {
#if HW_USART_TX_USE_DMA
        status = USART_TxDmaSetup();
#else
        ; /* 发送走阻塞，无需 DMA。 */
#endif
    }
    else
    {
        ; /* no deal with */
    }

    return status;
}

/* 发送：按宏自动选 DMA 或阻塞，调用方式不变。 */
HAL_StatusTypeDef USART_Send(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (data == NULL)
    {
        status = HAL_ERROR;
    }
#if HW_USART_TX_USE_DMA
    else if ((HAL_UART_GetState(&s_huart1) & HAL_UART_STATE_BUSY_TX) != 0U)
    {
        status = HAL_BUSY;               /* 上一帧没发完，防重入 */
    }
    else
    {
        status = HAL_UART_Transmit_DMA(&s_huart1, (uint8_t *)data,
                                       (uint16_t)len);
    }
#else
    else
    {
        status = HAL_UART_Transmit(&s_huart1, (uint8_t *)data,
                                   (uint16_t)len, 100U); /* 100ms 超时 */
    }
#endif

    return status;
}

/* 启动接收：按宏自动选 DMA 或逐字节中断。 */
HAL_StatusTypeDef USART_StartReceive(void)
{
    HAL_StatusTypeDef status = HAL_OK;

#if HW_USART_RX_USE_DMA
    {
        DMA_InitTypeDef dinit = {0};

        dinit.Direction = DMA_PERIPH_TO_MEMORY;
        dinit.PeriphInc = DMA_PINC_DISABLE;
        dinit.MemInc = DMA_MINC_ENABLE;
        dinit.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        dinit.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        dinit.Mode = DMA_CIRCULAR;       /* 循环收，不停不重启。 */
        dinit.Priority = DMA_PRIORITY_HIGH;

        status = HWDMA_Config(DMA1_Channel5, &s_hdma_rx, &dinit);
        if (status == HAL_OK)
        {
            __HAL_LINKDMA(&s_huart1, hdmarx, s_hdma_rx);
            status = HAL_UART_Receive_DMA(&s_huart1, s_rx_buf,
                                          USART_DMA_RX_BUF_SIZE);
        }
        else
        {
            ; /* HAL_BUSY=通道登记冲突，沿用 status */
        }
    }
#else
    status = HAL_UART_Receive_IT(&s_huart1, &s_rx_byte, 1U); /* 逐字节中断收。 */
#endif

    return status;
}

/* RX 用 DMA 时需要 IDLE 切帧，中断向量按宏二选一编译。 */
void USART1_IRQHandler(void)
{
#if HW_USART_RX_USE_DMA
    uint16_t frame_len = 0U;

    if (__HAL_UART_GET_FLAG(&s_huart1, UART_FLAG_IDLE) != RESET)
    {
        /* F1 清 IDLE：先读 SR 再读 DR，动作要连续，别插其他判断。 */
        __HAL_UART_CLEAR_IDLEFLAG(&s_huart1);

        /* 本帧实际长度 = 缓冲区总长 - DMA 剩余计数。 */
        frame_len = USART_DMA_RX_BUF_SIZE -
                    (uint16_t)__HAL_DMA_GET_COUNTER(s_huart1.hdmarx);
        if (frame_len > 0U)
        {
            /* TODO：拷走 s_rx_buf[0..frame_len) 处理，中断里只拷贝不做业务。 */
        }
        else
        {
            ; /* no deal with */
        }

        /* Abort+重启：每帧从缓冲区头开始写，避免循环跨界拼接。 */
        (void)HAL_UART_AbortReceive(&s_huart1);
        (void)HAL_UART_Receive_DMA(&s_huart1, s_rx_buf, USART_DMA_RX_BUF_SIZE);
    }
    else
    {
        ; /* no deal with */
    }

    HAL_UART_IRQHandler(&s_huart1);      /* 溢出等错误交给 HAL 统一处理。 */
#else
    HAL_UART_IRQHandler(&s_huart1);      /* RXNE 路径全由 HAL 处理。 */
#endif

    return;
}

#if !HW_USART_RX_USE_DMA
/* 逐字节中断收：每收 1 字节回调一次，重挂继续收。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t byte = s_rx_byte;

        /* TODO：喂给协议解析状态机，不要在中断里做耗时处理。 */
        (void)byte;
        (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1U);
    }
    else
    {
        ; /* no deal with */
    }

    return;
}
#endif
```

### main.c 里怎么用（四个宏怎么组合都同一套调用）

```c
#include "hw_adc.h"
#include "usart_dma.h"

if (ADC_Init() != HAL_OK)
{
    Error_Handler();
}
else
{
    ; /* no deal with */
}
if (USART_Init(115200U) != HAL_OK)
{
    Error_Handler();
}
else
{
    ; /* no deal with */
}
if (USART_StartReceive() != HAL_OK)
{
    Error_Handler();
}
else
{
    ; /* no deal with */
}

/* 主循环： */
(void)USART_Send(frame, frame_len);      /* TX 宏决定 DMA 还是阻塞 */
uint32_t mv = ADC_ReadMv(0U);            /* ADC 宏决定 DMA 还是阻塞 */
```

### 四个宏的组合说明

| TX宏 | RX宏 | 行为 | 占用DMA通道 |
|------|------|------|------------|
| 1 | 1 | DMA发 + DMA收（IDLE切帧） | Ch4 + Ch5 |
| 1 | 0 | DMA发 + 逐字节中断收 | Ch4 |
| 0 | 1 | 阻塞发 + DMA收（IDLE切帧） | Ch5 |
| 0 | 0 | 全不用DMA，最省资源 | 无 |

坑与要点（RX 用 DMA 时）：

1. **F1 的 HAL 对 RX DMA + IDLE 支持不完善**：清标志、取长度、重启接收必须自己在
   IRQHandler 里手工做（F4 的教程封装更好，别照搬）。
2. F1 清 IDLE 是"先读 SR 再读 DR"，清标志与取计数之间别插其他判断。
3. RX 用 `DMA_CIRCULAR` + Abort 重启，保证每帧从缓冲区头开始，判断简单可靠。
4. 定长帧省掉 IDLE 整套：`HAL_UART_Receive_DMA` 长度设帧长，收完在
   `HAL_UART_RxCpltCallback` 里处理并重启一次接收即可（此时自己实现回调需与
   逐字节方案 `#if` 区分开，避免重定义）。
5. 发送用 `HAL_BUSY` 防重入比死等稳；真要等发完在 `HAL_UART_TxCpltCallback` 置标志。

---

## 5. 扩展：ADC 中断方式（单次采样，不阻塞CPU）

不想阻塞又暂不上 DMA 时，在阻塞方案基础上改启动+回调：

```c
(void)HAL_ADC_Start_IT(&s_hadc1);   /* 启动后立即返回。 */

/* 转换完成回调（放自己的驱动文件里，不要放 stm32f1xx_it.c）。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        uint16_t raw = (uint16_t)HAL_ADC_GetValue(hadc);

        /* TODO：存入队列/做滤波，不要在回调里做耗时处理。 */
        (void)raw;
    }
    else
    {
        ; /* no deal with */
    }

    return;
}
```

需自己提供向量并开 NVIC：

```c
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&s_hadc1);

    return;
}
```

`HAL_NVIC_SetPriority(ADC1_2_IRQn, 5U, 0U); HAL_NVIC_EnableIRQ(ADC1_2_IRQn);`

---

## 6. 内部通道速查（F103C8，中容量）

| 通道 | 内容 | 说明 |
|------|------|------|
| CH16 | 内部温度传感器 | 需 `ADC_TEMPERATURE_SENSOR_ENABLE()` 使能，需长采样时间（≥17.1us） |
| CH17 | 内部参考电压 VREFINT | 约1.20V，可反推实际VDDA：`VDDA = 1200mV * 4095 / vrefint_raw` |

读内部通道时不要把外部通道引脚改掉，直接在序列里追加 Rank 即可。
基准电压说明：C8 封装 VREF⁺ 内部接 VDDA，`HW_ADC_VREF_MV` 填 3300 隐含"3.3V轨准确"假设；
要更准可实测 VDDA 改宏，或运行时读 CH17 自校准。

---

## 7. 集成检查清单

1. `hw_dma.c/h`、`hw_adc.c/h`、`usart_dma.c/h` 新建后加入 Keil 对应分组（放 `User/Drivers`）。
2. 命名核对：`HWDMA_`/`ADC_`/`USART_` 前缀与 GJB8114 既有模块（`UART_`/`NSA2860_`）不冲突。
3. 引脚确认：PA0~PA7、PB0/PB1 是 ADC 输入；PF6~PF10 在 C8 封装上不存在。
4. 转换时间估算：`采样周期 + 12.5周期`，除以 ADC 时钟。239.5+12.5=252 周期 ÷ 10.67MHz ≈ 23.6us/次。
5. `HAL_ADCEx_Calibration_Start` 必须在每次上电后、首次转换前调用一次（F1 必做）。
6. 高阻抗信号源（如直接接分压电阻 >10k）务必用长采样时间，否则读数偏低。
7. 不要在 ADC 时钟未开时读寄存器；`__HAL_RCC_ADC1_CLK_ENABLE()` 放在 init 最前。
8. DMA 通道占用：ADC1→Ch1，USART1_TX→Ch4，USART1_RX→Ch5。`HWDMA_Config` 返回 `HAL_BUSY`
   即通道登记冲突，先查硬件绑定表再定位是谁重复占用。
9. 中断向量全部放自己的驱动文件（USART1_IRQHandler、ADC1_2_IRQHandler 等），避免与
   `stm32f1xx_it.c` 同名重复定义；GJB8114 参考工程里 uart.c 已定义 USART1_IRQHandler，
   两套代码不要同时编译进一个固件。
