#include "main.h"
#include "hw_adc.h"
#include "hw_dma.h"

static ADC_HandleTypeDef s_hadc1 = {0};   /* ADC实例 */

#if HW_ADC_USE_DMA

static DMA_HandleTypeDef s_hdma_adc = {0};   /* DMA通道1实例 */
static volatile uint16_t s_adc_raw[HW_ADC_DMA_CH_NUM * HW_DMA_DEPTH] = {0}; /* 下标即通道序号-1 */

#endif/* HW_ADC_USE_DMA */

/*将原始数据转换为电压值*/
uint32_t ADC_ReadToMV(uint16_t raw)
{
    uint32_t mv = 0U;
    mv = (raw * HW_ADC_VREF_MV) / 4095U;
    return mv;
}

#if HW_ADC_USE_DMA
/*================ ADC+DMA连续多通道 ==================*/

static void ADC_GpioClkEnable(GPIO_TypeDef *port)
{
    if(port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if(port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else
    {
        ; /* 无效GPIO端口 */
    }
    return;
}

/*ADC GPIO初始化*/
static void ADC_GpioSetup(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();  /* 使能ADC1时钟 */
    ADC_GpioClkEnable(HW_ADC_CH1_PORT); /* 使能ADC通道1GPIO时钟 */
    ADC_GpioClkEnable(HW_ADC_CH2_PORT); /* 使能ADC通道2GPIO时钟 */

    gpio.Mode = GPIO_MODE_ANALOG;      /* 模拟输入 */
    gpio.Pull = GPIO_NOPULL;             

    gpio.Pin = HW_ADC_CH1_PIN;
    HAL_GPIO_Init(HW_ADC_CH1_PORT, &gpio);
    
    gpio.Pin = HW_ADC_CH2_PIN;
    HAL_GPIO_Init(HW_ADC_CH2_PORT, &gpio);
    return;
}

/* ADC1 硬绑定 DMA1_Channel1,并登记与配置*/
static HAL_StatusTypeDef ADC_DmaSetup(void)
{
    HAL_StatusTypeDef status;
    DMA_InitTypeDef dinit = {0};

    dinit.Direction = DMA_PERIPH_TO_MEMORY;               /* 从外设到内存 */
    dinit.PeriphInc = DMA_PINC_DISABLE;                   /* 外设地址不递增 */
    dinit.MemInc = DMA_MINC_ENABLE;                       /* 内存地址递增，按通道存。 */
    dinit.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* 从外设侧：每次读16位(半字) */
    dinit.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;     /* 从内存侧：每次写16位(半字) */
    dinit.Mode = DMA_CIRCULAR;                            /* 循环模式，采满自动覆盖。 */
    dinit.Priority = DMA_PRIORITY_HIGH;                    /* 高优先级 */

    /*记录DMA*/
    status = HWDMA_Config(DMA1_Channel1, &s_hdma_adc, &dinit);
    if(status == HAL_OK)
    {
        __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc);
    }
    else
    {
        ; /* 配置失败。 */
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
    s_hadc1.Init.ContinuousConvMode = ENABLE;            /* 连续转换（开）。 */
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;        /* 不连续转换（关）。 */
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;        /* 低位对齐，0~4095。 */
    s_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;  /* 软件触发。 */
    s_hadc1.Init.NbrOfConversion = HW_ADC_DMA_CH_NUM;    /* 转换通道数。 */

    status = HAL_ADC_Init(&s_hadc1);
    if(status == HAL_OK)
    {
        ch.Channel = HW_ADC_CH1;                         /* 通道1 */
        ch.Rank = ADC_REGULAR_RANK_1;                       /* 通道1在转换序列中的排名 */
        ch.SamplingTime = HW_ADC_SAMPLING;                  /* 通道1采样时间 */
        status = HAL_ADC_ConfigChannel(&s_hadc1, &ch);      /* 配置通道1 */
    }
    else
    {
        ; /* no deal with */
    }
    if(status == HAL_OK)
    {
        ch.Channel = HW_ADC_CH2;
        ch.Rank = ADC_REGULAR_RANK_2;                    /* CH4排第2个转换，结果落s_adc_raw[1]。 */
        status = HAL_ADC_ConfigChannel(&s_hadc1, &ch);
    }
    else
    {
        ; /* no deal with */
    }
    if(status == HAL_OK)
    {
        status = HAL_ADCEx_Calibration_Start(&s_hadc1); /* 启动校准 */
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

    ADC_GpioSetup();            /* 初始化ADC GPIO */
    status = ADC_DmaSetup();    /* 初始化DMA通道1 */
    if(status == HAL_OK)
    {
        status = ADC_AdcSetup(); /* 初始化ADC1 */
    }
    else
    {
        ; /* no deal with */
    }
    if(status == HAL_OK)
    {
        status = HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_raw, (uint32_t)HW_ADC_DMA_CH_NUM * HW_DMA_DEPTH); /* 启动DMA转换 */
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
    if(ch_index < HW_ADC_DMA_CH_NUM)
    {
        raw = s_adc_raw[ch_index];              /* 读取DMA缓存 */
    }
    else
    {
        ; /* 无效通道索引 */
    }
    return raw;
}

/*读取平均原始数据（8次采样）*/
uint16_t ADC_ReadAvgRaw(uint8_t ch_index)
{
    uint32_t sum = 0U;
    uint16_t avg = 0U;
    uint8_t i;
    uint16_t read_pos;

    if(ch_index < HW_ADC_DMA_CH_NUM)
    {
        for(i = 0U; i < HW_DMA_DEPTH; ++i)
        {
            read_pos = (uint16_t)ch_index + ((uint16_t)i * HW_ADC_DMA_CH_NUM); /* 计算DMA缓存读取位置 */
            sum = sum + s_adc_raw[read_pos];
        }
        avg = sum / HW_DMA_DEPTH;
    }
    else
    {
        ; /* 无效通道索引 */
    }
    return avg;
}
#else/* HW_ADC_USE_DMA */

/* =============无DMA,单通道配置================== */
/* ADC 本体 + 通道序列 + F1 上电校准。 */
static HAL_StatusTypeDef ADC_AdcSetup(void)
{
    HAL_StatusTypeDef status;
    GPIO_InitTypeDef gpio = {0};
    ADC_ChannelConfTypeDef ch = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();                /* 使能ADC1时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();               /* 使能GPIOA时钟 */

    gpio.Pin = HW_ADC_CH1_PIN;                  /* 通道1引脚 */
    gpio.Mode = GPIO_MODE_ANALOG;               /* 模拟输入 */
    gpio.Pull = GPIO_NOPULL;                    /* 无上拉下拉 */
    HAL_GPIO_Init(HW_ADC_CH1_PORT, &gpio);      /* 初始化通道1引脚 */
    
    s_hadc1.Instance = ADC1;    
    s_hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;        /* 单通道不扫描。 */
    s_hadc1.Init.ContinuousConvMode = DISABLE;           /* 读一次转一次：转完一轮停下等软件Start。 */
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;        /* 单通道无序列可拆，固定关闭。 */
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;        /* 低位对齐，0~4095。 */
    s_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;  /* 软件触发。 */
    s_hadc1.Init.NbrOfConversion = 1U;

    status = HAL_ADC_Init(&s_hadc1);
    if(status == HAL_OK)
    {
        ch.Channel = HW_ADC_CH1;                            /* 通道1 */
        ch.Rank = ADC_REGULAR_RANK_1;                       /* 通道1在转换序列中的排名 */
        ch.SamplingTime = HW_ADC_SAMPLING;                  /* 通道1采样时间 */
        status = HAL_ADC_ConfigChannel(&s_hadc1, &ch);      /* 配置通道1 */
    }
    else
    {
        ; /* no deal with */
    }
    if(status == HAL_OK)
    {
        status = HAL_ADCEx_Calibration_Start(&s_hadc1); /* 启动校准 */
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
    (void)ch_index;

    /* 启动转换 */
    if(HAL_ADC_Start(&s_hadc1) == HAL_OK)
    {
        /* 等待转换完成 */
        if(HAL_ADC_PollForConversion(&s_hadc1, 10U) == HAL_OK)
        {
            raw = (uint16_t)HAL_ADC_GetValue(&s_hadc1); /* 读取转换结果 */
        }
        else
        {
            ; /* no deal with */
        }
        (void)HAL_ADC_Stop(&s_hadc1); /* 停止转换 */
    }
    else
    {
        ; /* no deal with */
    }
    return raw;
}
#endif /* HW_ADC_USE_DMA */
/* 读取电压值 */
uint32_t ADC_ReadMv(uint8_t ch_index)
{
    uint32_t mv = 0U;
    mv = ADC_ReadToMV(ADC_ReadRaw(ch_index));
    return mv;
}
