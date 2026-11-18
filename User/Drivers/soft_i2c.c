#include "soft_i2c.h"

/*配置总线*/

const I2C_Bus I2C_Bus0 = {
    GPIOA, GPIO_PIN_5,      /* SCL：(PA5) */
    GPIOA, GPIO_PIN_6       /* SDA：(PA6) */
};

const I2C_Bus I2C_Bus1 = {
    GPIOB, GPIO_PIN_13,     /* SCL：(PB13) */
    GPIOB, GPIO_PIN_14      /* SDA：(PB14) */
};

const I2C_Bus I2C_BusEE = {
    GPIOB, GPIO_PIN_6,      /* SCL：(PB6) */
    GPIOB, GPIO_PIN_7       /* SDA：(PB7) */
};

#define SENSOR_HALF_US 2U
#define EEPROM_HALF_US 5U

/* 配置不同总线的时序快慢 */
static uint32_t half_us(const I2C_Bus *bus)
{
    return ((bus == &I2C_Bus0) || (bus == &I2C_Bus1)) ? SENSOR_HALF_US : EEPROM_HALF_US;
}
/*计时模块*/
static void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us; /*每一um包含多少时间周期*/
    uint32_t start = DWT->CYCCNT;                        /*获取当前周期数*/
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)     /*当前周期数是否超过延时周期数*/
    {
        __NOP(); /*空操作，等待延时周期数*/
    }
}

/* ==========================================================================
 * I2C 操作模块
 * ========================================================================== */

/*写入SCL引脚电平*/
static void scl(const I2C_Bus *bus, uint8_t level)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*写入SDA引脚电平*/
static void sda(const I2C_Bus *bus, uint8_t level)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*读取SDA引脚电平*/
static uint8_t sda_read(const I2C_Bus *bus)
{
    uint8_t ret = 0U;
    ret = (HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin) == GPIO_PIN_SET) ? 1U : 0U;
    return ret;
}

/*i2c起始信号*/
static void i2c_start(const I2C_Bus *bus)
{
    uint32_t t = half_us(bus);
    sda(bus, 1U);                   /* 空闲：两根线都是高 */
    if (t == SENSOR_HALF_US)
    {
        /* 重复 START 前也要保证低电平及 SDA 释放建立时间。 */
        delay_us(t);
    }
    else
    {
        ; /* no deal with */
    }
    scl(bus, 1U);
    delay_us(t);
    sda(bus, 0U);                   /* SCL 为高时 SDA 下降 = 起始条件 */
    delay_us(t);
    scl(bus, 0U);                   /* 拉低时钟，准备传第一个 bit */
    delay_us(t);
}

/*i2c停止信号*/
static void i2c_stop(const I2C_Bus *bus)
{
    uint32_t t = half_us(bus);
    sda(bus, 0U);                   /* 先拉低数据线 */
    delay_us(t);
    scl(bus, 1U);                   /* 再释放时钟 */
    delay_us(t);
    sda(bus, 1U);
    delay_us(t);
}

/* 发送一个字节，返回 0 表示从机应答(ACK)，1 表示无应答(NACK) */
static uint8_t i2c_write_byte(const I2C_Bus *bus, uint8_t byte)
{
    uint8_t i;
    uint8_t nack; /* 从机应答 */
    uint32_t t = half_us(bus); /* 时钟周期半长 */

    for (i = 0U; i < 8U; i++)
    {
        sda(bus, (byte & 0x80U) ? 1U : 0U);     /* 先摆好数据（高位在前） */
        byte = (uint8_t)(byte << 1U);           /* 左移一位，准备下一位 */
        delay_us(t);
        scl(bus, 1U);                           /* 时钟高电平期间从机采样 */
        delay_us(t);
        scl(bus, 0U);
        /* 传感器下一位的 setup 等待已保证低电平，不再重复等待。 */
        if (t == EEPROM_HALF_US)
        {
            delay_us(t);
        }
        else
        {
            ; /* no deal with */
        }
    }

    /* 等待从机应答 */
    sda(bus, 1U);                            /* 释放 SDA，让从机来拉低 */
    delay_us(t);
    scl(bus, 1U);
    delay_us(t);
    nack = sda_read(bus);                    /* 0 = 从机把线拉低了 = 应答 */

    /* 从机应答后，释放时钟 */
    scl(bus, 0U);
    if (t == EEPROM_HALF_US)
    {
        delay_us(t);
    }
    else
    {
        ; /* no deal with */
    }
    return nack;
}

/* 读一个字节。ack=1 回 ACK（还要继续读），ack=0 回 NACK（最后一个字节） */
static uint8_t i2c_read_byte(const I2C_Bus *bus, uint8_t ack)
{
    uint8_t i;
    uint8_t byte = 0U;
    uint32_t t = half_us(bus);

    sda(bus, 1U);                            /* 释放数据线，交给从机驱动 */
    for (i = 0U; i < 8U; i++)
    {
        delay_us(t);
        scl(bus, 1U);                        /* 时钟高电平期间读 */
        delay_us(t);
        byte = (uint8_t)((byte << 1U) | sda_read(bus));
        scl(bus, 0U);
    }

    sda(bus, (uint8_t)(ack ? 0U : 1U));      /* 拉低 = ACK，保持高 = NACK */
    delay_us(t);
    scl(bus, 1U);
    delay_us(t);
    scl(bus, 0U);
    if (t == EEPROM_HALF_US)
    {
        delay_us(t);
    }
    else
    {
        ; /* no deal with */
    }
    sda(bus, 1U);
    return byte;
}

/* ==========================================================================
 * 外部读写接口（寄存器）
 * ========================================================================== */
/* 读取多个字节，bus总线，addr7从机地址，reg寄存器地址，n字节数，buf缓存指针 */
uint8_t I2C_ReadRegs(const I2C_Bus *bus, uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t ret = 0U; /* 读取结果 */
    uint8_t i; /* 循环变量 */
    uint8_t addr_w = (uint8_t)(addr7 << 1);        /* 写地址 */
    uint8_t addr_r = (uint8_t)((addr7 << 1) | 1U); /* 读地址 */

    do
    {
        if (bus == NULL || buf == NULL || n == 0U)
        {
            ret = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }

        i2c_start(bus);    /* 发送起始信号 */

        /* 如果发送写地址失败 */
        if (i2c_write_byte(bus, addr_w) != 0U)
        {
            i2c_stop(bus);     /* 发送停止信号 */
            ret = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }
        /* 如果发送寄存器地址失败 */
        if (i2c_write_byte(bus, reg) != 0U)
        {
            i2c_stop(bus);
            ret = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }

        i2c_start(bus);                                 /* 重复起始，切到读方向 */
        /* 如果发送读地址失败 */
        if (i2c_write_byte(bus, addr_r) != 0U)
        {
            i2c_stop(bus);
            ret = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }
        /* 读取数据 */
        for (i = 0U; i < n; i++)
        {
            buf[i] = i2c_read_byte(bus, (uint8_t)(i < (uint8_t)(n - 1U)));
        }
        i2c_stop(bus);     /* 发送停止信号 */
        ret = 0U;
    } while (0);

    return ret;
}

/* 写入寄存器，bus总线，addr7从机地址，reg寄存器地址，value写入值 */
uint8_t I2C_WriteReg(const I2C_Bus *bus, uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t return_value = 0;
    uint8_t nack; /* NACK 标志位 */

    if (bus == NULL)
    {
        return_value = 1U;
    }
    else
    {
        i2c_start(bus);    /* 发送起始信号 */
        nack  = i2c_write_byte(bus, (uint8_t)(addr7 << 1)); /* 发送写地址 */
        nack |= i2c_write_byte(bus, reg);                   /* 发送寄存器地址 */
        nack |= i2c_write_byte(bus, value);                 /* 发送数据 */
        i2c_stop(bus);     /* 发送停止信号 */
        return_value = (nack != 0U) ? 1U : 0U;
    }

    return return_value;
}

/* 写入多个字节 */
uint8_t I2C_WriteBytes(const I2C_Bus *bus, uint8_t addr7, const uint8_t *buf, uint8_t n)
{
    uint8_t return_value = 0;
    uint8_t i;
    uint8_t nack;

    if (bus == NULL)
    {
        return_value = 1U;
    }
    else
    {
        if (buf == NULL && n != 0U)
        {
            return_value = 1U;
        }
        else
        {
            i2c_start(bus);    /* 发送起始信号 */
            nack = i2c_write_byte(bus, (uint8_t)(addr7 << 1)); /* 发送写地址 */
            /* 只有上一字节收到 ACK 才发送下一字节，退出循环后统一发 STOP。 */
            for (i = 0U; i < n && nack == 0U; i++)
            {
                nack = i2c_write_byte(bus, buf[i]); /* 发送数据 */
            }
            i2c_stop(bus);     /* 发送停止信号 */
            return_value = (nack != 0U) ? 1U : 0U;
        }
    }

    return return_value;
}

/* 写入多个字节，然后读取多个字节 */
uint8_t I2C_WriteThenRead(const I2C_Bus *bus, uint8_t addr7,const uint8_t *wbuf, uint8_t wn,uint8_t *rbuf, uint8_t rn)
{
    uint8_t return_value = 0;
    uint8_t exit_requested = 0U; /* 内层循环要求结束本次函数处理。 */
    uint8_t i;
    uint8_t addr_r = (uint8_t)((addr7 << 1) | 1U); /* 读地址 */

    /* 提前结束本次处理后，统一从函数末尾退出。 */
    do
    {
        if (bus == NULL)
        {
            return_value = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }
        /* 如果写缓冲区为空且写字节数为 0，或读缓冲区为空且读字节数为 0，直接返回失败*/
        if ((wbuf == 0 && wn != 0U) || rbuf == 0 || rn == 0U)
        {
            return_value = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }

        i2c_start(bus);    /* 发送起始信号 */
        if (i2c_write_byte(bus, (uint8_t)(addr7 << 1)) != 0U) /* 发送写地址 */
        {
            i2c_stop(bus);
            return_value = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }
        /* 发送写数据 */
        for (i = 0U; i < wn; i++)
        {
            if (i2c_write_byte(bus, wbuf[i]) != 0U)
            {
                i2c_stop(bus);
                return_value = 1U;
                exit_requested = 1U;
                break;
            }
            else
            {
                ; /* no deal with */
            }
        }
        if (exit_requested != 0U)
        {
            break;
        }
        else
        {
            ; /* no deal with */
        }

        i2c_start(bus);                                  /* 重复起始，切到读方向 */
        if (i2c_write_byte(bus, addr_r) != 0U) /* 发送读地址 */
        {
            i2c_stop(bus);
            return_value = 1U;
            break;
        }
        else
        {
            ; /* no deal with */
        }
        /* 读取数据 */
        for (i = 0U; i < rn; i++)
        {
            rbuf[i] = i2c_read_byte(bus, (uint8_t)(i < (uint8_t)(rn - 1U)));
        }
        i2c_stop(bus);
        return_value = 0U;
    } while (0);

    return return_value;
}

/* 重置 I2C 总线 */
uint8_t I2C_ResetBus(const I2C_Bus *bus)
{
    uint8_t return_value = 0;
    uint8_t i;  /* 循环变量 */
    uint32_t t; /* 时钟周期 */
    if (bus == NULL)
    {
        return_value = 1U;
    }
    else
    {
        t = half_us(bus);  /* 时钟周期 */
        i2c_start(bus);    /* 发送起始信号 */
        sda(bus, 1U);     /* 释放 SDA，让从机来拉低 */
        delay_us(t);       /* 等待从机拉低 SDA */

        /* 发送 9 个时钟周期，每个时钟周期拉低 SDA 1 个时钟周期 */
        for (i = 0U; i < 9U; i++)
        {
            scl(bus, 1U);  /*拉低 SDA */
            delay_us(t);
            scl(bus, 0U);  /* 拉高 SDA  */
            delay_us(t);
        }
        i2c_start(bus);    /* 发送起始信号 */
        i2c_stop(bus);     /* 发送停止信号 */

        /*==== 检查总线是否恢复 ====*/
        if (sda_read(bus) == 0U)
        {
            return_value = 1U;  /* SDA 仍被拉低 */
        }
        else if (HAL_GPIO_ReadPin(bus->scl_port, bus->scl_pin) != GPIO_PIN_SET)
        {
            return_value = 1U;  /* SCL 仍被拉低 */
        }
        else
        {
            return_value = 0U;  /* 总线空闲 */
        }
    }

    return return_value;
}

/* ==========================================================================
 * 初始化模块
 * ========================================================================== */

 /* 初始化 I2C 总线 */
void I2C_BusInit(const I2C_Bus *bus)
{
    GPIO_InitTypeDef gpio = {0}; /* GPIO 初始化结构体 */

    if (bus == NULL)
    {
        ; /* no deal with */
    }
    else
    {
        /*如果 SCL 或 SDA 连接到 GPIOA，使能 GPIOA 时钟*/
        if (bus->scl_port == GPIOA || bus->sda_port == GPIOA)
        {
            __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能 GPIOA 时钟 */
        }
        else
        {
            ; /* no deal with */
        }
        /*如果 SCL 或 SDA 连接到 GPIOB，使能 GPIOB 时钟*/
        if (bus->scl_port == GPIOB || bus->sda_port == GPIOB)
        {
            __HAL_RCC_GPIOB_CLK_ENABLE(); /* 使能 GPIOB 时钟 */
        }
        else
        {
            ; /* no deal with */
        }
        /*如果 SCL 或 SDA 连接到 GPIOC，使能 GPIOC 时钟*/
        if (bus->scl_port == GPIOC || bus->sda_port == GPIOC)
        {
            __HAL_RCC_GPIOC_CLK_ENABLE(); /* 使能 GPIOC 时钟 */
        }
        else
        {
            ; /* no deal with */
        }

        /* 先把输出锁存器置 1 再切成开漏，避免初始化瞬间把总线拉低 */
        HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET);

        gpio.Mode = GPIO_MODE_OUTPUT_OD;      /* 开漏，外部必须接上拉电阻 */
        gpio.Pull = GPIO_NOPULL;              /* 无上拉下拉 */
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;    /* 高速模式 */
        gpio.Pin = bus->scl_pin;              /* SCL 引脚 */
        HAL_GPIO_Init(bus->scl_port, &gpio);  /* 初始化 SCL 引脚 */
        gpio.Pin = bus->sda_pin;              /* SDA 引脚 */
        HAL_GPIO_Init(bus->sda_port, &gpio);  /* 初始化 SDA 引脚 */
    }

    return;

}

 /* 初始化 I2C 模块 */
void I2C_Init(void)
{
    static uint8_t dwt_ready = 0U; /* DWT 是否已初始化 */

    /* 初始化 I2C 3条总线 */
    I2C_BusInit(&I2C_Bus0);
    I2C_BusInit(&I2C_Bus1);
    I2C_BusInit(&I2C_BusEE);

    /* 如果 DWT 未初始化，启动周期计数 */
    if (dwt_ready == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 允许访问 DWT */
        DWT->CYCCNT = 0U;                                 /*  秒表清零 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* 启动周期计数 */
        dwt_ready = 1U;                                   /* 标记为已初始化 */
    }
    else
    {
        ; /* no deal with */
    }
}
