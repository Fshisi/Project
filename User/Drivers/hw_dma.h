#ifndef HW_DMA_H
#define HW_DMA_H

#include "main.h"

/*配置并登记一个DMA通道，
 * ch_instance: DMA1_Channel1~7（哪个外设用哪个通道见硬件绑定表，不能自选）。
 * handle:      DMA句柄，
 * init:        参数（方向、对齐、模式等）*/
HAL_StatusTypeDef HWDMA_Config(DMA_Channel_TypeDef *ch_instance,DMA_HandleTypeDef *handle,const DMA_InitTypeDef *init);

/* 取某通道当前登记的句柄，用于IRQHandler分发；未登记返回NULL。 */
DMA_HandleTypeDef *HWDMA_GetHandle(DMA_Channel_TypeDef *ch_instance);
#endif
