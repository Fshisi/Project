#include "main.h"
#include "system_clock.h"
int main(void)
{
    if (HAL_Init() != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        ; /* HAL就绪。 */
    }
    SystemClock_Config();
    /* TODO：在这里初始化自己的驱动和应用。 */
    while (1)
    {
        /* TODO：在这里手写TC300PGS主循环。 */
        __NOP();
    }
}
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        __NOP();
    }
}
