#ifndef HW_ADC_H
#define HW_ADC_H

#include "main.h"

/* ============ 方案选择：1=ADC+DMA多通道；0=不加DMA，阻塞单通道 ============ */
#define HW_ADC_USE_DMA      0U

#define HW_ADC_VREF_MV      ((uint32_t)3300U)                      /*ADC基准电压 3.3V */
#define HW_ADC_SAMPLING     ADC_SAMPLETIME_239CYCLES_5             /*采样时间 239周期 */

#if HW_ADC_USE_DMA
/*================== DMA配置================== */
#define HW_ADC_DMA_CH_NUM   ((uint8_t)2U)       /*DMA通道数量 */
#define HW_DMA_DEPTH       ((uint8_t)8U)       /*DMA深度 */
/* 每行一个通道：GPIO端口、GPIO引脚、ADC通道号。 */
#define HW_ADC_CH1_PORT     GPIOA
#define HW_ADC_CH1_PIN      GPIO_PIN_3
#define HW_ADC_CH1          ADC_CHANNEL_3

#define HW_ADC_CH2_PORT     GPIOA   
#define HW_ADC_CH2_PIN      GPIO_PIN_4
#define HW_ADC_CH2          ADC_CHANNEL_4

uint16_t ADC_ReadAvgRaw(uint8_t ch_index); /*读取平均原始数据（8次采样）*/

#else
/*================== 阻塞单通道配置================== */
#define HW_ADC_CH1_PORT     GPIOA
#define HW_ADC_CH1_PIN      GPIO_PIN_3
#define HW_ADC_CH1          ADC_CHANNEL_3

#endif /* HW_ADC_USE_DMA */

HAL_StatusTypeDef ADC_Init(void);           /*ADC初始化*/
uint32_t ADC_ReadToMV(uint16_t raw); /*将原始数据转换为电压值*/
uint16_t ADC_ReadRaw(uint8_t ch_index);     /*读取原始数据*/
uint32_t ADC_ReadMv(uint8_t ch_index);      /*读取电压值*/

#endif
