#include "stm32f10x.h"
#include "mpu6050.h"
static void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd  (RCC_APB2Periph_GPIOB,ENABLE ); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

/***模式设置 使能I2C***/

static void I2C_Mode_Config(void)
{
    I2C_InitTypeDef I2C_InitStructure; 

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C ; 
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
    I2C_InitStructure.I2C_ClockSpeed = 50000; 

    I2C_Init(I2C1, &I2C_InitStructure);
      

    I2C_Cmd  (I2C1,ENABLE); 

    I2C_AcknowledgeConfig(I2C1, ENABLE);   
}

/***初始化***/

void I2C_MPU6050_Init(void)
{   
    I2C_GPIO_Config();
    I2C_Mode_Config();
} 



//第三步：写一个写数据函数和一个读数据函数

/** 读数据函数****/

void I2C_ByteWrite(uint8_t REG_Address,uint8_t REG_data)
{
  I2C_GenerateSTART(I2C1,ENABLE);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1,REG_Address);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_SendData(I2C1,REG_data);

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(I2C1,ENABLE);
}



/*** 读数据函数 **/

uint8_t I2C_ByteRead(uint8_t REG_Address)
{
    uint8_t REG_data;

    while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C1,ENABLE);

    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);

    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_Cmd(I2C1,ENABLE);

    I2C_SendData(I2C1,REG_Address);

    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C1,ENABLE);

    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Receiver);

    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_AcknowledgeConfig(I2C1,DISABLE);

    I2C_GenerateSTOP(I2C1,ENABLE);

    while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));

    REG_data=I2C_ReceiveData(I2C1);

    return REG_data;
}
