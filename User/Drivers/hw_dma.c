#include "hw_dma.h"
#include "main.h"

#define HW_DMA_CH_COUNT     7U  /* DMA通道数量 */

typedef struct
{
    DMA_Channel_TypeDef    *instance;  /* DMA通道实例 */
    DMA_HandleTypeDef   *handle;    /* DMA句柄 */
} HWDMA_Chan;

static HWDMA_Chan s_chan_map[HW_DMA_CH_COUNT] = {0};     /* DMA通道映射表 */

static uint8_t chan_index(DMA_Channel_TypeDef *ch)
{
  uint8_t idx = HW_DMA_CH_COUNT;     

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


HAL_StatusTypeDef HWDMA_Config(DMA_Channel_TypeDef *ch_instance,DMA_HandleTypeDef *handle,const DMA_InitTypeDef *init)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t idx = chan_index(ch_instance);

    if((handle == NULL) || (init == NULL)||(idx >= HW_DMA_CH_COUNT))
    {
        status = HAL_ERROR;
    }
    else if(s_chan_map[idx].handle != NULL)
    {
        status = HAL_BUSY;
    }
    else
    {
        __HAL_RCC_DMA1_CLK_ENABLE();       /* 使能DMA时钟 */
        handle->Instance = ch_instance;    /* DMA通道实例 */
        if(HAL_DMA_Init(handle) != HAL_OK) /* 初始化DMA通道 */
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

/* 获取DMA通道句柄,ch_instance DMA通道实例 */
DMA_HandleTypeDef *HWDMA_GetHandle(DMA_Channel_TypeDef *ch_instance)
{
    DMA_HandleTypeDef *handle = NULL;
    uint8_t idx = chan_index(ch_instance); /* 获取DMA通道索引 */

    if(idx < HW_DMA_CH_COUNT)
    {
        handle = s_chan_map[idx].handle; /* 获取DMA通道句柄 */
    }
    return handle;
}
