/* 2026-11-16：沿用TC300PG_GJB8114时钟链，独立为可配置模块。 */
#include "main.h"
#include "app_config.h"
#include "system_clock.h"
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0}; /* HSI和PLL参数。 */
    RCC_ClkInitTypeDef clk = {0}; /* 总线分频。 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = APP_HSI_TRIM;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        ; /* 时钟配置成功。 */
    }
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        ; /* APB1定时器时钟仍为64MHz。 */
    }
}
