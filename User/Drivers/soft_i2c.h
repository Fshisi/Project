#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "main.h"

/*总线*/

typedef struct I2C_Bus {
    GPIO_TypeDef *scl_port;  //SCL引脚端口
    uint16_t      scl_pin;   //SCL引脚
    GPIO_TypeDef *sda_port;  //SDA引脚端口
    uint16_t      sda_pin;   //SDA引脚
} I2C_Bus;

extern const I2C_Bus I2C_Bus0;
extern const I2C_Bus I2C_Bus1;
extern const I2C_Bus I2C_BusEE;
/* ==========================================================================
 * 外部读写接口（寄存器）
 * ========================================================================== */
uint8_t I2C_ReadRegs(const I2C_Bus *bus, uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n);  /* 读取多个寄存器 */
uint8_t I2C_WriteReg(const I2C_Bus *bus, uint8_t addr7, uint8_t reg, uint8_t value);            /* 写入寄存器 */
uint8_t I2C_WriteBytes(const I2C_Bus *bus, uint8_t addr7, const uint8_t *buf, uint8_t n);        /* 写入多个字节 */
uint8_t I2C_WriteThenRead(const I2C_Bus *bus, uint8_t addr7,const uint8_t *wbuf, uint8_t wn,uint8_t *rbuf, uint8_t rn); /* 写入多个字节后读取多个字节 */
uint8_t I2C_ResetBus(const I2C_Bus *bus); /* 重置 I2C 总线 */

/* ==========================================================================
 * 初始化模块
 * ========================================================================== */
void I2C_Init(void); /* 初始化 I2C 模块 */

#endif
