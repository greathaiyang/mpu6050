#ifndef	__I2C_H
#define	__I2C_H

#include "stm32f10x.h"



#define	IIC_GPIO_SDA	GPIOB	
#define	IIC_PIN_SDA		GPIO_Pin_7	

#define	IIC_GPIO_SCL	GPIOB
#define	IIC_PIN_SCL		GPIO_Pin_6





#define SET_SCL     	GPIO_SetBits(IIC_GPIO_SCL,IIC_PIN_SCL);
#define RESET_SCL     	GPIO_ResetBits(IIC_GPIO_SCL,IIC_PIN_SCL);


#define SET_SDA     	GPIO_SetBits(IIC_GPIO_SDA,IIC_PIN_SDA);
#define RESET_SDA     	GPIO_ResetBits(IIC_GPIO_SDA,IIC_PIN_SDA);






void  SDA_IN(void);
void  SDA_OUT(void) ;
void  SCL_OUT(void);
void IIC_Config(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_writebyte(uint8_t j) ;
uint8_t i2c_readbyte(void)  ;
void i2c_answer(void)  ;


void iic_send_ack(void);
void iic_send_nack(void);
void iic_wait_ack(void);
uint8_t iic_readByte(void);
void iic_sendByte(uint8_t data);
void iic_stop(void);
void iic_start(void);
#endif
