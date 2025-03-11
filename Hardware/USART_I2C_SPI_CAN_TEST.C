#include"Usart.h"
#include"I2c.h"
#include"SPI.h"
#include<stdlib.h>
#include<string.h>



/*===============================================Usart+idle+dma中断+环形缓冲=========================================================*/
#define RX_MAXLEN 20
#define TX_MAXLEN 20



bool Usart_ReceiveDataFlag = 0;//环形缓冲区接收标志
bool usart_send_flag=0;
//USART为异步时钟,需要两边配置相同的波特率,数据位,停止位,校验位,硬件流控制
//USART的配置需要先开启时钟,然后配置USART的结构体,最后初始化USART
typedef struct Usart_cycleBuffer
{
    uint8_t head_cnt;
    uint8_t tail_cnt;
    uint8_t buffer[256];
}Usart_cycleBuffer_s;//串口环形缓冲结构体

Usart_cycleBuffer_s Usart_cycleBuffer;
uint8_t Usart_DMA_rxbuffer[RX_MAXLEN];//DMA接收缓冲区
uint8_t Usart_DMA_txbuffer[TX_MAXLEN];//DMA发送缓冲区

void __Usart_cycleBuffer_Init()
{
    Usart_cycleBuffer.head_cnt = 0;
    Usart_cycleBuffer.tail_cnt = 0;
    memset(Usart_cycleBuffer.buffer, 0, 256);
}
void __Usart_cycleBuffer_Write(uint8_t data)
{
    Usart_cycleBuffer.buffer[Usart_cycleBuffer.head_cnt] = data;
    Usart_cycleBuffer.head_cnt++;
    if(Usart_cycleBuffer.head_cnt == 256)
    {
        Usart_cycleBuffer.head_cnt = 0;
    }
}

uint8_t __Usart_cycleBuffer_Read()
{
    uint8_t data = Usart_cycleBuffer.buffer[Usart_cycleBuffer.tail_cnt];
    Usart_cycleBuffer.tail_cnt++;
    if(Usart_cycleBuffer.tail_cnt == 256)
    {
        Usart_cycleBuffer.tail_cnt = 0;
    }
    return data;
}

bool __Usart_cycleBuffer_IsEmpty()
{
    if(Usart_cycleBuffer.head_cnt == Usart_cycleBuffer.tail_cnt)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void __Usart_Init()
{
    //开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
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
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
    //初始化NVIC
    __USART_DMA_NVIC_Init();
    //DMA配置
    __DMA_Init();
    //开启USART中断
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//接收数据寄存器非空中断
    //开启DMA中断
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    //使能串口
    USART_Cmd(USART1, ENABLE);
}

void __DMA_Init()
{
    //开启时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //初始化DMA
    //公共属性
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据大小为字节
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据大小为字节
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址增
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//M2M为0,硬件触发


    //配置usart_rx dma
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Usart_DMA_rxbuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据传输的源
    DMA_InitStructure.DMA_BufferSize =RX_MAXLEN;//缓冲区大小,在接收到数据后会自动填充
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    //配置usart_tx dma
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Usart_DMA_txbuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//外设作为数据传输的目的
    DMA_InitStructure.DMA_BufferSize =0;//缓冲区大小,在接收到数据后会自动填充
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//普通模式
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel4, DISABLE);
}

//NVIC配置
void __USART_DMA_NVIC_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //关闭传输过半中断
    DMA_ITConfig(DMA1_Channel5, DMA_IT_HT, DISABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_HT, DISABLE);
    //开启传输完成中断
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}
//串口发送相关函数


//DMA 发送数据
void __USART_DMA_TX_DATA(uint32_t size)
{
    while(usart_send_flag);
    usart_send_flag=1;
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel4, size);
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1_Channel4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC4))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        usart_send_flag=0;
    }
}


/*usart中断函数*/
void USART1_IRQHandler(void)
{
    uint16_t temp;
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        temp = USART1->SR;
        temp = USART1->DR;//清除IDLE标志  
        Usart_ReceiveDataFlag = 1;
        DMA_Cmd(DMA1_Channel5, DISABLE);
        temp = DMA_GetCurrDataCounter(DMA1_Channel5);
        DMA_SetCurrDataCounter(DMA1_Channel5, RX_MAXLEN);
        DMA_Cmd(DMA1_Channel5, ENABLE);
        
        DMA_Cmd(DMA1_Channel4, DISABLE);
        temp = DMA_GetCurrDataCounter(DMA1_Channel4);
        DMA_SetCurrDataCounter(DMA1_Channel4, TX_MAXLEN- temp);
        DMA_Cmd(DMA1_Channel4, ENABLE);

        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        DMA_ClearFlag(DMA1_FLAG_TC4);
        USART_ClearFlag(USART1, USART_FLAG_TC);

        
    }
}


/*===============================================I2c软硬件实现=========================================================*/
#define I2C_DEVICE_WRITE_ADDRESS 0xD0
#define I2C_DEVICE_READ_ADDRESS 0xD1

//软件实现

void __I2C_SIMU_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
    Delay_us(10);
}

void __I2C_SIMU_W_SDA(uint8_t  BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
    Delay_us(10);
}

uint8_t __I2C_SIMU_R_SDA(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
}

void __I2C_SIMU_Init(void)
{
    //开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    //初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_11);
}


//I2C开始信号
//在SCL高电平时拉低SDA  先拉高二者,再给出起始信号
void __I2C_SIMU_Start(void)
{
    __I2C_SIMU_W_SDA(1);
    __I2C_SIMU_W_SCL(1);
    __I2C_SIMU_W_SDA(0);
    __I2C_SIMU_W_SCL(0);
}
//I2C停止信号
//在SCL高电平时拉低SDA  先拉低SDA,再拉高SCL,再拉高SDA
void __I2C_SIMU_Stop(void)
{
    __I2C_SIMU_W_SDA(0);
    __I2C_SIMU_W_SCL(1);
    __I2C_SIMU_W_SDA(1);
}
//发送1个byte
//从高位到低位,每次发送一个bit,并在SCL高电平时拉低SDA
void __I2C_SIMU_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        __I2C_SIMU_W_SDA((Byte & 0x80) >> 7);
        Byte <<= 1;
        __I2C_SIMU_W_SCL(1);
        __I2C_SIMU_W_SCL(0);
    }
}
//接收1个byte
//从高位到低位,每次接收一个bit,并在SCL高电平时读取SDA
void __I2C_SIMU_ReceiveByte(uint8_t *Byte)
{
    uint8_t i;
    *Byte = 0;
    for(i = 0; i < 8; i++)
    {
        *Byte <<= 1;
        __I2C_SIMU_W_SCL(1);
        //主机高电平读取
        if(__I2C_SIMU_R_SDA())
        {
            *Byte |= 0x01;
        }
        __I2C_SIMU_W_SCL(0);
    }
}
//发送应答
//在SCL高电平时拉低SDA
void __I2C_SIMU_SendAck(uint8_t Ack)
{
    __I2C_SIMU_W_SDA(Ack);
    __I2C_SIMU_W_SCL(1);
    __I2C_SIMU_W_SCL(0);
}
//接收应答
//在SCL高电平时读取SDA
void __I2C_SIMU_ReceiveAck(uint8_t *Ack)
{
    __I2C_SIMU_W_SDA(1);
    __I2C_SIMU_W_SCL(1);
    *Ack = __I2C_SIMU_R_SDA();
    __I2C_SIMU_W_SCL(0);
}

//指定地址写一个字节
//时序 : 发送起始信号→发送设备写地址→发送寄存器地址→发送数据→发送停止信号
void __I2C_SIMU_WriteReg(uint8_t Reg, uint8_t data)
{
    uint8_t ACK;
    __I2C_SIMU_Start();
    __I2C_SIMU_SendByte(I2C_DEVICE_WRITE_ADDRESS);
    __I2C_SIMU_ReceiveAck(&ACK);
    __I2C_SIMU_SendByte(Reg);
    __I2C_SIMU_ReceiveAck(&ACK);
    __I2C_SIMU_SendByte(data);
    __I2C_SIMU_ReceiveAck(&ACK);
    __I2C_SIMU_Stop();
}

//指定地址读一个字节
//时序: 发送起始信号→发送设备写地址→发送寄存器地址→发送设备读地址→读取数据→发送停止信号
void __I2C_SIMU_ReadReg(uint8_t Reg, uint8_t *data)
{
    uint8_t Ack;
    __I2C_SIMU_Start();
    __I2C_SIMU_SendByte(I2C_DEVICE_WRITE_ADDRESS);
    __I2C_SIMU_ReceiveAck(&Ack);
    __I2C_SIMU_SendByte(Reg);
    __I2C_SIMU_ReceiveAck(&Ack);
    __I2C_SIMU_Start();//重复起始条件
    __I2C_SIMU_SendByte(I2C_DEVICE_READ_ADDRESS);
    __I2C_SIMU_ReceiveAck(&Ack);
    __I2C_SIMU_ReceiveByte(data);
    __I2C_SIMU_SendAck(1);
    __I2C_SIMU_Stop();
}

//硬件实现

void __I2C_Hardware_Init(void)
{
    //开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    //初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //初始化I2C
    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
}

void __I2C_Hardware_WriteReg(uint8_t Reg, uint8_t data)
{
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));//EV5事件, 读SR1然后将地址写入DR将清除该事件
    I2C_Send7bitAddress(I2C2, I2C_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);//发送设备地址
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//EV6事件, 读SR1然后读SR2将清除该事件
    I2C_SendData(I2C2, Reg);//发送寄存器地址
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));//EV8事件, 读SR1将清除该事件
    I2C_SendData(I2C2, data);//发送数据
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//EV8_2事件
    I2C_GenerateSTOP(I2C2, ENABLE);
}

void __I2C_Hardware_ReadReg(uint8_t Reg, uint8_t *data)
{
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));//EV5事件
    I2C_Send7bitAddress(I2C2, I2C_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//EV6事件
    I2C_SendData(I2C2, Reg);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//EV8_2事件
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));//EV5事件
    I2C_Send7bitAddress(I2C2, I2C_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//EV6事件
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));//EV7_1事件
    *data = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
}


/*===============================================CAN=========================================================*/

/*数据帧 : 发送设备主动发送数据,接收设备被动接收数据
 * 1bit的SOF(帧起始)→发送显性0
 * ↓         ↓             ↓
 * 11bit的ID(报文)→发送ID                                     
 * ↓         ↓             ↓                                    
 * 1bit的RTR(远程传输请求)→发送RTR(数据帧为显性0,遥控帧为隐性1)  
 * ↓         ↓             ↓                                
 * 1bit的IDE(扩展帧)→发送IDE(标准帧为隐性0,扩展帧为显性1)       
 * ↓         ↓             ↓                                 
 * 1bit的r0(保留位)→发送r0                                    
 * ↓         ↓             ↓                                 
 * 4bit的DLC(数据长度)→发送DLC                                
 * ↓         ↓             ↓                                 
 * 0~64bit的DATA(数据)→发送DATA                               
 * ↓         ↓             ↓                                 
 * 15bit的CRC(循环冗余校验)→发送CRC                            
 * ↓         ↓             ↓                                 
 * 1bit的CRC界定符(隐性1)发送方释放总线                                           
 * ↓         ↓             ↓                                 
 * 1bit的ACK(确认)→发送ACK(如果接受方收到,接收方发送显性0,发送方接收隐性1)    
 * ↓         ↓             ↓
 * 1bit的ACK界定符(隐性1)接收方释放总线
 * ↓         ↓             ↓
 * 7bit的EOF(帧结束)→发送隐性1                                
 */

 /*遥控帧 : 接受设备主动请求数据,发送设备被动发送数据
 * 无数据段,RTP为隐性1,其余和数据帧相同
 */

 /*错误帧 : 设备默认处在被动错误状态,当错误发生时,设备主动发送错误帧6个隐性0,之后发送8bit的错误标识符(隐性1)*/

 /*过载帧 : 接收方处理过载时发送,和错误帧格式相同*/



 /*位填充规则:发送方发送五个相同电平后,自动追加一个相反电平的填充位,接收方检测到填充位时,会自动移除填充位,恢复原始数 */

 /*位同步
 *位时序: ss+pts+pbs1+pbs2,最小时间单位tq,ss为同步段,pts为传播时间段,pbs1为相位段1,pbs2为相位段2
 *采样点: pbs1和pbs2之间
 */

 //stm32内置bxCAN外设,支持CAN2.0A和CAN2.0B协议,可以自动发送CAN报文和按照过滤器自动接收指定CAN报文,程序无需关注总线电平细节
#define CAN_DEVICE_ID 0x1234;



void __CAN_Init()
{
    //启动时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //CAN_RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //CAN_TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //初始化CAN
    CAN_InitTypeDef CAN_InitStructure;
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM = DISABLE;//时间触发通信模式 每个FIFO有自己的时间寄存器
    CAN_InitStructure.CAN_ABOM = DISABLE;//自动总线关闭
    CAN_InitStructure.CAN_AWUM = DISABLE;//自动唤醒模式 

    CAN_InitStructure.CAN_NART = DISABLE;//不自动重传功能      
    CAN_InitStructure.CAN_RFLM = DISABLE;//报文锁定模式  
    CAN_InitStructure.CAN_TXFP = DISABLE;//发送FIFO优先级

    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;//CAN模式
    //波特率=1/正常的位时间
    //正常的位时间=1*tq+tbs1+tbs2
    //tq=(BRP+1)*APB1时钟周期
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//重新同步跳跃宽度
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;//时间段1
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;//时间段2
    CAN_InitStructure.CAN_Prescaler = 6;//BRP 分频系数(不用-1)
    CAN_Init(CAN1, &CAN_InitStructure);    
}

void __CAN_FILTER_Init()
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    //====32位列表模式=====
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    //第一组id
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    //第二组id
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    //====32位掩码模式====
    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;

    //32位id
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    //32位掩码
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    //16位列表模式
    //16位掩码模式
}

void __CAN_NVIC_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

}

//=================================发送接收函数===================================
void __CAN_CREATE_Message(CanTxMsg *TxMessage, uint32_t StdId, uint32_t ExtId, uint8_t IDE, uint8_t RTR, uint8_t DLC, uint8_t Data[])
{
    TxMessage->StdId = StdId;
    TxMessage->ExtId = ExtId;
    TxMessage->IDE = IDE;
    TxMessage->RTR = RTR;
    TxMessage->DLC = DLC;
    for(uint8_t i = 0; i < DLC; i++)
    {
        TxMessage->Data[i] = Data[i];
    }
}

void __CAN_Transmit(CanTxMsg *TxMessage)
{
    uint8_t transmit_mailbox = 0;
    uint32_t timeout = 0;
    
    transmit_mailbox = CAN_Transmit(CAN1, TxMessage);

    //判断发送是否成功
    while(CAN_TransmitStatus(CAN1, transmit_mailbox) == CAN_TxStatus_Failed)
    {
        timeout++;
        if(timeout > 0x1FFFFFF)
        {
            break;
        }
    }
}

uint8_t __CAN_ReceiveFlag(void)
{   
    if(CAN_MessagePending(CAN1, CAN_FIFO0)>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void __CAN_Receive(CanRxMsg *RxMessage)
{
    if(__CAN_ReceiveFlag())
    {
        CAN_Receive(CAN1, CAN_FIFO0, RxMessage);
    }
}
 
//处理接收到的数据
void __CAN_ReceiveMessage_process(CanRxMsg *RxMessage, uint8_t *id, uint8_t*len, uint8_t *data)
{
//是否扩展帧
    if(RxMessage->IDE == CAN_Id_Extended)
    {
        *id = RxMessage->ExtId;
    }
    else
    {
        *id = RxMessage->StdId;
    }
//是否数据帧
    if(RxMessage->RTR == CAN_RTR_Data)
    {
        *len = RxMessage->DLC;
        for(uint8_t i = 0; i < *len; i++)
        {
            data[i] = RxMessage->Data[i];
        }
    }
    else
    {
        *len = 0;
    }

}
//=================================发送接收函数===================================