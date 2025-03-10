#include"I2c.h"

#define MPU6050_WRITE_ADDRESS 0xD0
#define MPU6050_READ_ADDRESS 0xD1

void MyI2c_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
    Delay_us(10);
}
void MyI2c_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
    Delay_us(10);
}
uint8_t MyI2c_R_SDA(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
}
void MyI2c_Init(void)//软件模拟ic
{
    //开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
}

void MyI2c_Start(void)
{
    MyI2c_W_SDA(1);
    MyI2c_W_SCL(1);
    MyI2c_W_SDA(0);
    MyI2c_W_SCL(0);
}

void MyI2c_Stop(void)
{
    MyI2c_W_SDA(0);
    MyI2c_W_SCL(1);
    MyI2c_W_SDA(1);
}

void MyI2c_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        MyI2c_W_SDA((Byte & 0x80) >> 7);
        Byte <<= 1;
        MyI2c_W_SCL(1);
        MyI2c_W_SCL(0);
    }

}

void MyI2c_ReceiveByte(uint8_t *Byte)
{
    uint8_t i;
    *Byte = 0;
    MyI2c_W_SDA(1);
    for(i=0; i<8; i++)
    {
        *Byte <<= 1;
        MyI2c_W_SCL(1);
        if(MyI2c_R_SDA())
        {
            *Byte |= 0x01;
        }
        MyI2c_W_SCL(0);
    }

}

void MyI2c_SendAck(uint8_t Ack)
{
    MyI2c_W_SDA(Ack);
    MyI2c_W_SCL(1);
    MyI2c_W_SCL(0);
}

void MyI2c_ReceiveAck(uint8_t *Ack)
{
    MyI2c_W_SDA(1);
    MyI2c_W_SCL(1);
    *Ack = MyI2c_R_SDA();
    MyI2c_W_SCL(0);


}

void MPU6050_I2C_Init(void)
{
    MyI2c_Init();
    MPU6050_I2C_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    MPU6050_I2C_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_I2C_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_I2C_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_I2C_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_I2C_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

void MPU6050_I2C_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t ACK;
    MyI2c_Start();
    MyI2c_SendByte(MPU6050_WRITE_ADDRESS);
    MyI2c_ReceiveAck(&ACK);
    MyI2c_SendByte(reg);
    MyI2c_ReceiveAck(&ACK);
    MyI2c_SendByte(data);
    MyI2c_ReceiveAck(&ACK);
    MyI2c_Stop();
}

void MPU6050_I2C_ReadReg(uint8_t reg, uint8_t *data)
{
    uint8_t ACK;
    MyI2c_Start();
    MyI2c_SendByte(MPU6050_WRITE_ADDRESS);
    MyI2c_ReceiveAck(&ACK);
    MyI2c_SendByte(reg);
    MyI2c_ReceiveAck(&ACK);
    MyI2c_Start();
    MyI2c_SendByte(MPU6050_READ_ADDRESS);
    MyI2c_ReceiveAck(&ACK);
    MyI2c_ReceiveByte(data);
    MyI2c_SendAck(1);
    MyI2c_Stop();
}

void MPU6050_GetData(MPU6050_DATA* data)
{
    uint8_t temp[14];
    MPU6050_I2C_ReadReg(MPU6050_ACCEL_XOUT_H, &temp[0]);
    MPU6050_I2C_ReadReg(MPU6050_ACCEL_XOUT_L, &temp[1]);
    MPU6050_I2C_ReadReg(MPU6050_ACCEL_YOUT_H, &temp[2]);
    MPU6050_I2C_ReadReg(MPU6050_ACCEL_YOUT_L, &temp[3]);
    MPU6050_I2C_ReadReg(MPU6050_ACCEL_ZOUT_H, &temp[4]);
    MPU6050_I2C_ReadReg(MPU6050_ACCEL_ZOUT_L, &temp[5]);
    // MPU6050_I2C_ReadReg(MPU6050_TEMP_OUT_H, &temp[6]);
    // MPU6050_I2C_ReadReg(MPU6050_TEMP_OUT_L, &temp[7]);
    MPU6050_I2C_ReadReg(MPU6050_GYRO_XOUT_H, &temp[8]);
    MPU6050_I2C_ReadReg(MPU6050_GYRO_XOUT_L, &temp[9]);
    MPU6050_I2C_ReadReg(MPU6050_GYRO_YOUT_H, &temp[10]);
    MPU6050_I2C_ReadReg(MPU6050_GYRO_YOUT_L, &temp[11]);
    MPU6050_I2C_ReadReg(MPU6050_GYRO_ZOUT_H, &temp[12]);
    MPU6050_I2C_ReadReg(MPU6050_GYRO_ZOUT_L, &temp[13]);

    data->Accel_X = (int16_t)(temp[0]<<8 | temp[1]);
    data->Accel_Y = (int16_t)(temp[2]<<8 | temp[3]);
    data->Accel_Z = (int16_t)(temp[4]<<8 | temp[5]);
    data->Gyro_X = (int16_t)(temp[8]<<8 | temp[9]);
    data->Gyro_Y = (int16_t)(temp[10]<<8 | temp[11]);
    data->Gyro_Z = (int16_t)(temp[12]<<8 | temp[13]);
}

void MyI2c_Hardware_Init()
{
    //开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //初始化GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
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

void MPU6050_I2C_Hardware_WriteReg(uint8_t reg, uint8_t data)//指定地址写一个字节
{
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));//EV5事件
    I2C_Send7bitAddress(I2C2, MPU6050_WRITE_ADDRESS, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//EV6事件
    I2C_SendData(I2C2, reg);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));//EV8事件
    I2C_SendData(I2C2, data);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//EV8_2事件
    I2C_GenerateSTOP(I2C2, ENABLE);
}

void MPU6050_I2C_Hardware_ReadReg(uint8_t reg, uint8_t *data)//指定地址读一个字节
{
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));//EV5事件
    I2C_Send7bitAddress(I2C2, MPU6050_WRITE_ADDRESS, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//EV6事件
    I2C_SendData(I2C2, reg);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//EV8_2事件
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));//EV5事件
    I2C_Send7bitAddress(I2C2, MPU6050_READ_ADDRESS, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//EV6事件
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));//EV7_1事件
    *data = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
}

void MPU6050_I2C_Hardware_Init(void)
{
    MyI2c_Hardware_Init();
    MPU6050_I2C_Hardware_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    MPU6050_I2C_Hardware_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_I2C_Hardware_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_I2C_Hardware_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_I2C_Hardware_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_I2C_Hardware_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

void MPU6050_Hardware_GetData(MPU6050_DATA* data)
{
    uint8_t temp[14];
    MPU6050_I2C_Hardware_ReadReg(MPU6050_ACCEL_XOUT_H, &temp[0]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_ACCEL_XOUT_L, &temp[1]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_ACCEL_YOUT_H, &temp[2]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_ACCEL_YOUT_L, &temp[3]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_ACCEL_ZOUT_H, &temp[4]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_ACCEL_ZOUT_L, &temp[5]);
    // MPU6050_I2C_Hardware_ReadReg(MPU6050_TEMP_OUT_H, &temp[6]);
    // MPU6050_I2C_Hardware_ReadReg(MPU6050_TEMP_OUT_L, &temp[7]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_GYRO_XOUT_H, &temp[8]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_GYRO_XOUT_L, &temp[9]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_GYRO_YOUT_H, &temp[10]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_GYRO_YOUT_L, &temp[11]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_GYRO_ZOUT_H, &temp[12]);
    MPU6050_I2C_Hardware_ReadReg(MPU6050_GYRO_ZOUT_L, &temp[13]);

    data->Accel_X = (int16_t)(temp[0]<<8 | temp[1]);
    data->Accel_Y = (int16_t)(temp[2]<<8 | temp[3]);
    data->Accel_Z = (int16_t)(temp[4]<<8 | temp[5]);
    data->Gyro_X = (int16_t)(temp[8]<<8 | temp[9]);
    data->Gyro_Y = (int16_t)(temp[10]<<8 | temp[11]);
    data->Gyro_Z = (int16_t)(temp[12]<<8 | temp[13]);
}
