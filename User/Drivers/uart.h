#ifndef UART_H
#define UART_H

#include "main.h"

/* 调试串口句柄。默认按 USART1 命名；若适配区改用 USART2/3，建议同步改名 */
extern UART_HandleTypeDef huart1;

void UART1_Init(void);              /* 串口初始化（波特率/引脚在 uart.c 适配区宏里改） */
void UART1_SendStr(const char *s);  /* 阻塞发送字符串 */

#endif
