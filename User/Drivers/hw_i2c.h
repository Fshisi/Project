#ifndef _HW_I2C_H
#define _HW_I2C_H

#include "main.h"

uint8_t HWI2C1_Init(void);
uint8_t HWI2C1_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t n); /*读取n个寄存器*/
uint8_t HWI2C1_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value); /*写入寄存器*/
uint8_t HWI2C1_RecoverBus(void); /*修复i2c死锁*/

/*=================EEPROM操作模块，后续可以移植到EEPROM.c=================*/
uint8_t HWI2C1_WriteBytes(uint8_t addr7, const uint8_t *buf, uint8_t n); /*写入n个字节*/
uint8_t HWI2C1_WriteThenRead(uint8_t addr7, const uint8_t *wbuf, uint8_t wn,uint8_t *rbuf, uint8_t rn); /*写入n个字节，然后读取n个字节*/
uint8_t HWI2C1_Probe(uint8_t addr7); /*探测地址addr7是否可访问*/
#endif
