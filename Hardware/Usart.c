#include"Usart.h"

uint8_t Usart_ReceiveData = 0;
uint8_t Usart_ReceiveDataFlag = 0;

uint8_t Usart_TXPACKET[4] = {0};
uint8_t Usart_RXPACKET[4] = {0};
uint8_t pRXPACKET = 0;

void Usart_Init(void)
{
    //开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    //初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //初始化USART
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    //开启USART中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    


    //初始化NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Usart_SendByte(uint8_t byte)
{
    USART_SendData(USART1, byte);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Usart_SendString(uint8_t *array)
{
   
    for(uint8_t i = 0;array[i]!='\0'; i++)
    {
        Usart_SendByte(array[i]);
    }
}

void Usart_SendNumber(uint32_t Number,  uint8_t base)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t temp[10];
    if(Number == 0)
    {
        Usart_SendByte('0');
    }
    else
    {
        while(Number)
        {
            temp[i] = Number % base;
            Number = Number / base;
            i++;
        }
        for(j = i; j > 0; j--)
        {
            if(temp[j - 1] < 10)
            {
                Usart_SendByte(temp[j - 1] + '0');
            }
            else
            {
                Usart_SendByte(temp[j - 1] - 10 + 'A');
            }
        }
    }
}

void Usart_SendPacket(void)
{
    Usart_SendByte(0xFF);
    for(uint8_t i = 0; i < 4; i++)
    {
        Usart_SendByte(Usart_TXPACKET[i]);
    }
    Usart_SendByte(0xFE);
}




























void Usart_Printf(char *fmt, ...)
{
    va_list ap;
    char string[256];
    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    Usart_SendString((uint8_t *)string);
    va_end(ap);
}


uint8_t Usart_GetRxDataFlag(void)
{
    if ( Usart_ReceiveDataFlag == 1)
    {
        Usart_ReceiveDataFlag = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

void USART1_IRQHandler(void)
{
    static uint8_t status = 0;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        Usart_ReceiveData  = USART_ReceiveData(USART1);
        
    }

    switch (status)
    {
    case 0:
        if(Usart_ReceiveData == 0xFF)
        {
            status = 1;
        }
        break;
    case 1:
        Usart_RXPACKET[pRXPACKET] = Usart_ReceiveData;
        pRXPACKET++;
        if(pRXPACKET >= 4)
        {
            status = 2;
            pRXPACKET = 0;
        }
        break;
    case 2:
        if(Usart_ReceiveData == 0xFE)
        {
            status = 0;
            Usart_ReceiveDataFlag = 1;
        }
        break;
    default:
        break;
    }
}
