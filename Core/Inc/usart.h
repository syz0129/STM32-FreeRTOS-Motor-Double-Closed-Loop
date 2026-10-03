#ifndef __USART_H
#define __USART_H

#include "main.h"
#include "app_config.h"

/* 串口硬件配置 */
#define USART_HANDLE      (&huart1)     // USART1,PA9/PA10
#define USART_LINE_LEN    32            // 单条指令最长32字节

extern UART_HandleTypeDef huart1;

void USART_Start(void);                 // 启动中断接收
void USART_TransmitStr(const char *str);
void UsartCmdTask(void *arg);           // 指令解析任务

#endif /* __USART_H */
