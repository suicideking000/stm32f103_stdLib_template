#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include "stdio.h"
#include "stdarg.h"
void Usart_Init(void);
void Usart_SendByte(uint8_t byte);
void Usart_SendString(uint8_t *array);
void Usart_SendNumber(uint32_t Number,  uint8_t base);
void Usart_Printf( char *format, ...);
void Usart_SendPacket(void);



uint8_t Usart_GetRxDataFlag(void);

#endif
