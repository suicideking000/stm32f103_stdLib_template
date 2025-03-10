#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "Delay.h"
#include "MPU6050_Reg.h"

typedef struct MPU6050_DATA
{
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
}MPU6050_DATA;



void MyI2c_W_SCL(uint8_t BitValue);
void MyI2c_W_SDA(uint8_t BitValue);
uint8_t MyI2c_R_SDA(void);
void MyI2c_Init(void);
void MyI2c_Start(void);
void MyI2c_Stop(void);
void MyI2c_SendByte(uint8_t Byte);
void MyI2c_ReceiveByte(uint8_t *Byte);
void MyI2c_SendAck(uint8_t Ack);
void MyI2c_ReceiveAck(uint8_t *Ack);


void MPU6050_I2C_Init(void);
void MPU6050_I2C_WriteReg(uint8_t reg, uint8_t data);
void MPU6050_I2C_ReadReg(uint8_t reg, uint8_t *data);
void MPU6050_GetData(MPU6050_DATA* data);


void MPU6050_I2C_Hardware_Init(void);
void MPU6050_I2C_Hardware_WriteReg(uint8_t reg, uint8_t data);
void MPU6050_I2C_Hardware_ReadReg(uint8_t reg, uint8_t *data);
void MPU6050_Hardware_GetData(MPU6050_DATA* data);


#endif
