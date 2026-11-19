#include "uart.h"
#include <stdio.h>

#define RX_Port                 GPIOA
#define RX_Pin                  GPIO_PIN_10
#define TX_Port                 GPIOA
#define TX_Pin                  GPIO_PIN_9
#define UART_RX_BUF_SIZE        256U /* 接收缓冲区大小 */
#define UART_TX_TIMEOUT_MS      100U /* 阻塞式发送超时时间 */

static uint8_t  s_rx_byte;                       /* 接收字节缓存 */
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];    /* 环形接收缓冲区 */
static uint8_t s_txbuf[256];                     /* 中断发送缓冲区 */
static volatile uint16_t s_rx_head = 0U;         /* 写指针：中断里前进 */
static uint16_t s_rx_tail = 0U;                  /* 读指针：主循环前进 */
static volatile uint8_t  s_frame_flag = 0U;      /* 空闲帧标志 */
static volatile uint32_t s_rx_over = 0U;         /* 缓冲满丢弃计数 */

#define LOG_ENABLE  1    /* 1=日志开；0=全工程 LOG 整行删除，零开销 */

#if (LOG_ENABLE != 0)
#define LOG(...)    printf(__VA_ARGS__)
#else
#define LOG(...)    ((void)0)   /* 整条调用连参数一起消失 */
#endif

UART_HandleTypeDef huart1;
void UART1_Init(uint16_t baudrate)
{
    huart1.Instance = USART1;                        /* 串口实例 */
    huart1.Init.BaudRate = baudrate;                 /* 波特率 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;     /* 8 数据位 */
    huart1.Init.StopBits = UART_STOPBITS_1;          /* 停止位 */
    huart1.Init.Parity = UART_PARITY_NONE;           /* 校验位 */
    huart1.Init.Mode = UART_MODE_TX_RX;              /* 收发都开 */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;     /* 硬件流控制 */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; /* 采样率 */
    
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        ; /* 配置成功。 */
    }
    
    s_rx_head = 0U;    /* 初始化环形缓冲区写指针 */
    s_rx_tail = 0U;    /* 初始化环形缓冲区读指针 */
    s_frame_flag = 0U; /* 初始化空闲帧标志 */
    s_rx_over = 0U;    /* 初始化缓冲满丢弃计数 */

    /* 启动接收中断 */
    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        ; /* 接收链已启动。 */
    }

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); /* 使能空闲中断 */
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE(); /* 使能串口时钟 */
        if (TX_Port == GPIOA || RX_Port == GPIOA)
        {
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        else
        {
            ; /* 引脚不在 GPIOA。 */
        }
        if (TX_Port == GPIOB || RX_Port == GPIOB)
        {
            __HAL_RCC_GPIOB_CLK_ENABLE();
        }
        else
        {
            ; /* 引脚不在 GPIOB。 */
        }
        if (TX_Port == GPIOC || RX_Port == GPIOC)
        {
            __HAL_RCC_GPIOC_CLK_ENABLE();
        }
        else
        {
            ; /* 引脚不在 GPIOC。 */
        }

        gpio.Pin   = TX_Pin;
        gpio.Mode  = GPIO_MODE_AF_PP;      /* TX：复用推挽 */
        gpio.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速 */
        HAL_GPIO_Init(TX_Port, &gpio);

        gpio.Pin  = RX_Pin;
        gpio.Mode = GPIO_MODE_INPUT;       /* RX：浮空输入 */
        gpio.Pull = GPIO_NOPULL;           /* 无上下拉 */
        HAL_GPIO_Init(RX_Port, &gpio);

        HAL_NVIC_SetPriority(USART1_IRQn, 1, 3); /* 优先级：1，中断优先级：3 */
        HAL_NVIC_EnableIRQ(USART1_IRQn);         /* 使能中断 */
    }
    else
    {
        ; /* 其它串口实例不处理。 */
    }
}

/* ---- 发送：阻塞版。 */
void UART1_SendStr(const char *s)
{
    uint16_t len = 0U;
    if(s == NULL)
    {
        return;
    }
    else
    {
        while(s[len] != '\0')
        {
            
            len++;
        }
         (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, len, UART_TX_TIMEOUT_MS);
    }
}

/* ---- 发送：中断版。 ---- */
uint8_t UART1_SendStr_IT(const char *s)
{
    uint16_t len = 0U;
    uint8_t ret = 0U;
    if(s == NULL)
    {
        ret = 1U;
    }
    else
    {
        while ((s[len] != '\0') && (len < 255U))
        {
            s_txbuf[len] = s[len];          /* 复制字符到发送缓冲区 */
            len++;
        }
        if(HAL_UART_Transmit_IT(&huart1, s_txbuf, len) != HAL_OK)
        {
            ret = 1U;
        }
        else
        {
            ; /* 已受理，完成时进 TxCpltCallback。 */
        }
    }
    return ret;
}
/* ---- 接收中断处理函数。 ---- */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);/* 处理接收中断 */
    /*如果空闲帧中断标志位被设置*/
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); /* 清除空闲中断标志位 */
        s_frame_flag = 1U;                  /* 空闲帧标志设置为1 */
    }
    else
    {
        ; /* 其它中断标志位不处理。 */
    }
}

/*===========================环形接受缓冲区===========================*/
/* ---- 每字节接收完成：入环形缓冲 + 续挂下一字节 ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next = (uint16_t)(s_rx_head + 1U); /* 下一个写指针 */
    if(huart->Instance == USART1)
    {
        
        if(next >= UART_RX_BUF_SIZE)
        {
            next = 0U; /* 缓冲区已满，重置写指针 */
        }
        else
        {
            ;/* 未到底，正常前进。 */
        }
        if(next != s_rx_tail)
        {
            uart_rx_buf[s_rx_head] = s_rx_byte; /* 存储接收字符到缓冲区 */
            s_rx_head = next;               /* 更新写指针 */
        }
        else
        {
            s_rx_over++;            /* 缓冲区已满，丢弃接收字符，丢弃字符数增加 */
        }
        if(HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK)
        {
            s_rx_over++; /* 接收中断失败，丢弃接收字符，丢弃字符数增加 */
        }
        else
        {
            ; /* 接收链正常。 */
        }
    }
}

/*==================================================================== */

/* ---- 错误回调函数。 ---- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        /*如果接收中断失败*/
        if(HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK)
        {
            ;/* 状态未就绪的极端情况：留待下次错误或主循环恢复 */
        }
        else
        {
            ; /* 接收链正常。 */
        }
    }
}

/* ---- 中断发送完成的定点 ---- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        ; /* 例如双缓冲翻转后发起下一块 */
    }
}

/*======================主循环api===========================*/
/* 取一个字节：0=取到，1=空 */
uint8_t UART1_ReadRx(uint8_t *byte)
{
    uint16_t next; /**/
    uint8_t ret = 1U;

    if(byte == NULL)
    {
        ret = 1U;
    }
    else if(s_rx_tail != s_rx_head)
    {
        *byte = uart_rx_buf[s_rx_tail]; /* 取出接收字符 */
        next = (uint16_t)(s_rx_tail + 1U); /* 下一个读指针 */
        if(next >= UART_RX_BUF_SIZE)
        {
            next = 0U; /* 缓冲区已满，重置读指针 */
        }
        else
        {
            ;/* 未到底，正常前进。 */
        }
        s_rx_tail = next; /* 更新读指针 */
        ret = 0U;
    }
    else
    {
        ;/* 缓冲区为空 */
    }
    return ret;
}

/* 空闲帧标志（读即清）：1=自上次查询后有新的一包进缓冲 */
uint8_t UART1_FrameReady(void)
{
    uint8_t ret = s_frame_flag; 
    s_frame_flag = 0U;
    return ret;
}

/* 读累计丢弃/异常计数：持续增长说明缓冲不够或主循环太慢 */
uint32_t UART1_RxOverflow(void)
{
    return s_rx_over;
}

/* ---- printf 重定向（阻塞逐字符） ---- */
int fputc(int ch, FILE *f)
{
    (void)f;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, 10);
    return ch;
}
