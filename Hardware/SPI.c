#include "SPI.h"


void My_SPI_W_SS(uint8_t BitValue)//写ss片选引脚
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction) BitValue);
}

void My_SPI_W_SCL(uint8_t Bitvalue)//写数据
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction) Bitvalue);
}

void My_SPI_W_MOSI(uint8_t BitValue)//写数据
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction) BitValue);
}

uint8_t My_SPI_R_MISO(void)//读数据
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
}

void My_SPI_Start(void)
{
    My_SPI_W_SS(0);    
}

void My_SPI_End(void)
{
    My_SPI_W_SS(1);
}


void My_SPI_Init()
{
    //开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //配置GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    My_SPI_W_SS(1);
    My_SPI_W_SCL(0);
}



uint8_t My_SPI_SwapByte(uint8_t ByteSend) //SPI模式0
{
	uint8_t i, ByteReceive = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到
	
	for (i = 0; i < 8; i ++)						//循环8次，依次交换每一位数据
	{
		My_SPI_W_MOSI(ByteSend & (0x80 >> i));		//使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
		My_SPI_W_SCL(1);								//拉高SCK，上升沿移出数据
		if (My_SPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}	//读取MISO数据，并存储到Byte变量
																//当MISO为1时，置变量指定位为1，当MISO为0时，不做处理，指定位为默认的初值0
		My_SPI_W_SCL(0);								//拉低SCK，下降沿移入数据
	}
	
	return ByteReceive;		
}

void My_SPI_W25Q64_Init(void)
{
    My_SPI_Init();
}  

void My_SPI_W25Q64_ReadID(uint8_t *MID , uint16_t *DID)
{
    My_SPI_Start();
    My_SPI_SwapByte(W25Q64_JEDEC_ID);
    *MID = My_SPI_SwapByte(0xFF);
    *DID = My_SPI_SwapByte(0xFF);
    *DID <<=8;
    *DID |= My_SPI_SwapByte(0xFF);
    My_SPI_End();
}

void My_SPI_W25Q64_WriteEnable(void)
{
    My_SPI_Start();
    My_SPI_SwapByte(W25Q64_WRITE_ENABLE);
    My_SPI_End();
}

void My_SPI_W25Q64_WaitBusy(void)
{
    uint32_t TimeOut =10000;
    My_SPI_Start();
    My_SPI_SwapByte(W25Q64_READ_STATUS_REGISTER_1);  
    while(My_SPI_SwapByte(0xFF) & 0x01)
    {
        TimeOut--;
        if(TimeOut == 0)
        {
            break;
        }
    }
    My_SPI_End();
}

void My_SPI_W25Q64_PageProgram(uint32_t Address, uint8_t *Buffer, uint16_t Length)
{
    My_SPI_W25Q64_WaitBusy();
    My_SPI_W25Q64_WriteEnable();
    My_SPI_Start();
    My_SPI_SwapByte(W25Q64_PAGE_PROGRAM);
    My_SPI_SwapByte((Address >> 16) & 0xFF);
    My_SPI_SwapByte((Address >> 8) & 0xFF);
    My_SPI_SwapByte(Address & 0xFF);
    for(uint16_t i = 0; i < Length; i++)
    {
        My_SPI_SwapByte(Buffer[i]);
    }
    My_SPI_End();
    My_SPI_W25Q64_WaitBusy();
}

void My_SPI_W25Q64_SectorErase(uint32_t Address)
{
    My_SPI_W25Q64_WaitBusy();
    My_SPI_Start();
    My_SPI_SwapByte(W25Q64_SECTOR_ERASE_4KB);
    My_SPI_SwapByte((Address >> 16) & 0xFF);
    My_SPI_SwapByte((Address >> 8) & 0xFF);
    My_SPI_SwapByte(Address & 0xFF);
    My_SPI_End();
}

void My_SPI_W25Q64_ReadData(uint32_t Address, uint8_t *Buffer, uint32_t Length)
{
    My_SPI_W25Q64_WaitBusy();
    My_SPI_Start();
    My_SPI_SwapByte(W25Q64_READ_DATA);
    My_SPI_SwapByte((Address >> 16) & 0xFF);
    My_SPI_SwapByte((Address >> 8) & 0xFF);
    My_SPI_SwapByte(Address & 0xFF);
    for(uint32_t i = 0; i < Length; i++)
    {
        Buffer[i] = My_SPI_SwapByte(0xFF);
    }
    My_SPI_End();
}


0 1 2 3 4             10 11 12 13 14    
2 3 4 8 9             2 3 4 8  9