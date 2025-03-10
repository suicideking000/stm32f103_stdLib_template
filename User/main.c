#include "stm32f10x.h"  
#include "Delay.h"                // Device header
#include "Key.h"
#include "LED.h"
#include "OLED.h"
#include "CounterSensor.h"
#include "Timer.h"
#include "ADC.h"
#include "DMA.h"
#include "Usart.h"
#include "I2c.h"
#include "SPI.h"

extern uint8_t Usart_ReceiveData;
extern uint8_t Usart_ReceiveDataFlag;
extern uint8_t Usart_TXPACKET[];
extern uint8_t Usart_RXPACKET[];

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	OLED_Init();
	My_SPI_W25Q64_Init();


	uint8_t MID;
	uint16_t DID;
	uint8_t Buffer[]={0x01,0x02,0x03,0x04};
	uint8_t Buffer1[4]={0};

	My_SPI_W25Q64_ReadID(&MID, &DID);
	OLED_ShowHexNum(1, 1, MID, 2);
	OLED_ShowHexNum(1, 8, DID, 4);

	My_SPI_W25Q64_SectorErase(0x000000);
	My_SPI_W25Q64_PageProgram(0x000000, Buffer, 4);
	My_SPI_W25Q64_ReadData(0x000000, Buffer1, 4);

	OLED_ShowHexNum(2, 1, Buffer[0], 2);
	OLED_ShowHexNum(2, 4, Buffer[1], 2);
	OLED_ShowHexNum(2, 7, Buffer[2], 2);
	OLED_ShowHexNum(2, 10, Buffer[3], 2);

	OLED_ShowHexNum(3, 1, Buffer1[0], 2);
	OLED_ShowHexNum(3, 4, Buffer1[1], 2);
	OLED_ShowHexNum(3, 7, Buffer1[2], 2);
	OLED_ShowHexNum(3, 10, Buffer1[3], 2);
	while(1)
	{				

		
	}
	
}
