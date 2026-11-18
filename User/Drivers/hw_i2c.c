#include "hw_i2c.h"

#define HWI2C_TIMOUT_MS  20U /*超时时间20ms*/
#define HWI2C1_ADDR(a)   ((uint16_t)((a) << 1)) /*I2C1地址*/

static I2C_HandleTypeDef s_hi2c1;

/*硬件i2c初始化*/
uint8_t HWI2C1_Init(void)
{
    uint16_t ret = 0U;
	GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOB_CLK_ENABLE();   /*使能GPIOB时钟*/
    __HAL_RCC_I2C1_CLK_ENABLE();    /*使能I2C1时钟*/

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7; 
	gpio.Mode  = GPIO_MODE_AF_OD;      /*复用开漏输出*/
    gpio.Speed = GPIO_SPEED_FREQ_HIGH; /*高频率*/
    HAL_GPIO_Init(GPIOB, &gpio);       /*初始化GPIOB*/

    s_hi2c1.Instance             = I2C1;                     /*选择I2C1*/
    s_hi2c1.Init.ClockSpeed      = 100000U;                  /* 100kHz */
    s_hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;          /* 占空比2:1 */
    s_hi2c1.Init.OwnAddress1     = 0U;                       /* 主机地址为0 */
    s_hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;  /* 7位地址 */
    s_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;  /* 不使用双地址 */
    s_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  /* 不使用一般调用 */
    s_hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;    /* 允许从机拉住SCL */

    ret = (HAL_I2C_Init(&s_hi2c1) == HAL_OK) ? 0U : 1U;

    return ret;
}

/*读n字节,从地址addr7,寄存器reg开始,读到buf中,n为读取字节数*/
uint8_t HWI2C1_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint16_t ret = 0U;

    if((buf == NULL) || (n == 0U))
    {
        ret = 1U;
    }
    else
    {
        ret = HAL_I2C_Mem_Read(&s_hi2c1, HWI2C1_ADDR(addr7), reg, I2C_MEMADD_SIZE_8BIT, buf, n, HWI2C_TIMOUT_MS);/*读取n字节*/
        if(ret != HAL_OK)
        {
            ret = 1U;
        }
        else
        {
            ;/*空操作*/
        }
    }
    return ret;
}

/*往寄存器写字节,从地址addr7,寄存器reg开始,value写入的值*/
uint8_t HWI2C1_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint16_t ret = 0U;
    ret = HAL_I2C_Mem_Write(&s_hi2c1, HWI2C1_ADDR(addr7), reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, HWI2C_TIMOUT_MS);/*写入n字节*/
    if(ret != HAL_OK)
    {
        ret = 1U;
    }
    else
    {
        ;/*空操作*/
    }
    return ret;

}

/*修复i2c死锁*/
uint8_t HWI2C1_RecoverBus(void)
{
    GPIO_InitTypeDef gpio; 
    uint16_t ret = 0U;
    uint8_t  i;
    uint32_t d;     /*延时周期数*/

    __HAL_RCC_I2C1_CLK_ENABLE();                                   /*使能I2C1时钟*/
    I2C1->CR1 = 0U;                                                /* PE=0：外设松手，引脚还给 CPU */
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;                               /* 临时变回软 I2C 的形态 */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;                              /*高频率*/
    HAL_GPIO_Init(GPIOB, &gpio);                                    /*初始化GPIOB*/
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);/*拉高SCL和SDA*/

    /* 9 个 SCL 脉冲，逼从机吐完卡住的位 */
    for (i = 0U; i < 9U; i++) 
    {         
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); /*拉低SCL*/
        /*延时300个周期，大约20µs*/
        for (d = 0U; d < 300U; d++) 
        {
             __NOP(); 
        }             
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    /*拉高SCL*/
        /*延时300个周期，大约20µs*/
        for (d = 0U; d < 300U; d++) 
        {
             __NOP(); 
        }
    }
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
    {
        ret = 1U; /* SDA 仍被拉低：硬件级故障 */
    }
    else
    {
        ;/*空操作*/
    }
    /* SCL 保持高，SDA 低→高，补一个 STOP 收尾 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   /*拉低SDA*/

    for (d = 0U; d < 60U; d++) 
    {
         __NOP();
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);     /*拉高SDA*/
    for (d = 0U; d < 60U; d++) 
    {
         __NOP();
    }
    HWI2C1_Init();/*初始化i2c*/
    return ret;
}

/*=================EEPROM操作模块，后续可以移植到EEPROM.c================*/

/*写多个字节(EEPROM页写用),addr7为7位地址，buf为数据指针，n为数据长度*/
uint8_t HWI2C1_WriteBytes(uint8_t addr7, const uint8_t *buf, uint8_t n)
{
    uint16_t ret = 0U;
    if((buf == NULL) || (n == 0U))
    {
        ret = 1U;
    }
    else
    {
        ret = HAL_I2C_Master_Transmit(&s_hi2c1, HWI2C1_ADDR(addr7), (uint8_t*)buf, n, HWI2C_TIMOUT_MS);
        if(ret != HAL_OK)
        {
            ret = 1U;
        }
        else
        {
            ;/*空操作*/
        }
    }
    return ret;
}
/*读取多个字节(EEPROM页读用),addr7为7位地址，wbuf为写入数据指针，wn为写入数据长度，rbuf为读取数据指针，rn为读取字节数*/
uint8_t HWI2C1_WriteThenRead(uint8_t addr7, const uint8_t *wbuf, uint8_t wn,uint8_t *rbuf, uint8_t rn)
{
    uint16_t ret = 0U;
    uint16_t memaddr;
    uint16_t size;
    if((wbuf == NULL) || (rbuf == NULL) || (wn == 0U) || (rn == 0U))
    {
        ret = 1U;
    }
    else
    {
        /*如果写入数据长度大于等于2,则合并高8位和低8位,得到内存地址memaddr*/
        if(wn >= 2U)
        {
            memaddr = (uint16_t)(((uint16_t)wbuf[0] << 8) | wbuf[1]);/*合并高8位和低8位*/
            size = I2C_MEMADD_SIZE_16BIT; /*16位地址*/
        }
        else
        {
            memaddr = wbuf[0];/*直接使用低8位*/
            size = I2C_MEMADD_SIZE_8BIT; /*8位地址*/
        }
        ret = HAL_I2C_Mem_Read(&s_hi2c1, HWI2C1_ADDR(addr7), memaddr, size, rbuf, rn, HWI2C_TIMOUT_MS);/*读取n字节*/
        if(ret != HAL_OK)
        {
            ret = 1U;
        }
        else
        {
            ;/*空操作*/
        }
    }
    return ret;
}

/*有无ACK应答*/
uint8_t HWI2C1_Probe(uint8_t addr7)
{
    uint16_t ret = 0U;
    ret = HAL_I2C_IsDeviceReady(&s_hi2c1, HWI2C1_ADDR(addr7), 1U, HWI2C_TIMOUT_MS);
    if(ret != HAL_OK)
    {
        ret = 1U;
    }
    else
    {
        ;/*空操作*/
    }
    return ret;
   }
