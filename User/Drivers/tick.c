#include "tick.h"

TIM_HandleTypeDef htim3;

void Tick_Init(void)
{
    htim3.Instance               = TIM3;                    /* 用哪个定时器 */
    htim3.Init.Prescaler         = 64U - 1U;                /* 64MHz ÷ 64 = 1MHz，1 计数 = 1us */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;      /* 向上计数 */
    htim3.Init.Period            = 1000U - 1U;              /* 数 1000 个 = 1ms */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* 数字滤波采样分频，定时用 DIV1 */
    htim3.Init.RepetitionCounter = 0U;                      /* 仅 TIM1/TIM8 有意义，TIM3 填 0 */
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)                /* 纯时基用 Base 系列即可 */
    {
        Error_Handler();
    }
}

/* 初始化定时器 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE(); /* 使能 TIM3 时钟 */
        HAL_NVIC_SetPriority(TIM3_IRQn, 0U, 3U); /* 设置中断优先级 */
        HAL_NVIC_EnableIRQ(TIM3_IRQn); /* 使能中断 */
    }
}

HAL_TIM_Base_Start_IT(&htim3); /* 启动定时器中断 */

/* 定时器中断服务函数 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);   /* 查标志、清标志、分发回调，全在这一个函数里 */
}

/* 业务代码写回调（所有定时器共用这一个回调，用 Instance 区分） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        s_ms++;                        /* 对应你工程 TIM3_IRQHandler 里的 s_ms++ */
        s_flag_1ms = 1U;
        /* 25ms 分频逻辑照搬 */
    }
}
