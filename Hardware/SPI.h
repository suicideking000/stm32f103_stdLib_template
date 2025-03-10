#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
#include "W25Q64.h"
void My_SPI_Init(void);
void My_SPI_W_SS(uint8_t BitValue);
void My_SPI_W_SCL(uint8_t Bitvalue);
void My_SPI_W_MOSI(uint8_t BitValue);
void My_SPI_Start(void);
void My_SPI_End(void);
uint8_t My_SPI_SwapByte(uint8_t ByteSend);


void My_SPI_W25Q64_Init(void);
void My_SPI_W25Q64_ReadID(uint8_t *MID , uint16_t *DID);
void My_SPI_W25Q64_WriteEnable(void);
void My_SPI_W25Q64_WaitBusy(void);
void My_SPI_W25Q64_PageProgram(uint32_t Address, uint8_t *Buffer, uint16_t Length);
void My_SPI_W25Q64_ReadData(uint32_t Address, uint8_t *Buffer, uint32_t Length);
void My_SPI_W25Q64_SectorErase(uint32_t Address);

#endif
