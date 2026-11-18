#include "uart.h"
#include <stdio.h>

#define RX_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define TX_Port GPIOA
#define TX_Pin GPIO_PIN_9
#define UART_BaudRate 115200
#define UART_RX_BUF_SIZE 256 /* 接收缓冲区大小 */

static uint8_t  s_rx_byte;                       /* 接收字节缓存 */
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];    /* 环形接收缓冲区 */
static volatile uint16_t s_rx_head = 0U;         /* 写指针：中断里前进 */
static uint16_t s_rx_tail = 0U;                  /* 读指针：主循环前进 */
static volatile uint8_t  s_frame_flag = 0U;      /* 空闲帧标志 */
static volatile uint32_t s_rx_over = 0U;         /* 缓冲满丢弃计数 */

UART_HandleTypeDef huart1;
void UART_Init(uint16_t baudrate)
{
    huart1.Instance = USART1;                        /* 串口实例 */
    huart1.Init.BaudRate = baudrate;                 /* 波特率 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;     /* 8 数据位 */
    huart1.Init.StopBits = UART_STOPBITS_1;          /* 停止位 */
    huart1.Init.Parity = UART_PARITY_NONE;           /* 校验位 */
    huart1.Init.Mode = UART_MODE_TX_RX;              /* 收发都开 */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;     /* 硬件流控制 */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; /* 采采样率 */
    
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
        UART_CLK_ENABLE(); /* 使能串口时钟 */
        if (TX_Port == GPIOA || TX_Port == GPIOA)
        {
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        else
        {
            ; /* 引脚不在 GPIOA。 */
        }
        if (TX_Port == GPIOB || TX_Port == GPIOB)
        {
            __HAL_RCC_GPIOB_CLK_ENABLE();
        }
        else
        {
            ; /* 引脚不在 GPIOB。 */
        }
        if (TX_Port == GPIOC || TX_Port == GPIOC)
        {
            __HAL_RCC_GPIOC_CLK_ENABLE();
        }
        else
        {
            ; /* 引脚不在 GPIOC。 */
        }

        gpio.Pin   = TX_Pin;
        gpio.Mode  = GPIO_MODE_AF_PP;      /* TX：复用推挽 */
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(TX_Port, &gpio);

        gpio.Pin  = RX_Pin;
        gpio.Mode = GPIO_MODE_INPUT;       /* RX：浮空输入 */
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(RX_Port, &gpio);

        HAL_NVIC_SetPriority(USART1_IRQn, 1, 3);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

/*===========================环形接受缓冲区===========================*/


