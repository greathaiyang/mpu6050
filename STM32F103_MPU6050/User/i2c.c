#include "delay.h"
#include "i2c.h"




void  SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_PIN_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_GPIO_SDA, &GPIO_InitStructure);
}	
void  SDA_OUT(void)       
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_PIN_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_GPIO_SDA, &GPIO_InitStructure);	
    
}
void  SCL_OUT(void)       
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_PIN_SCL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_GPIO_SCL, &GPIO_InitStructure);	
}



void IIC_Config(void)
{
    SDA_OUT();
    SCL_OUT();
    SET_SCL;
    SET_SDA;	
}

void i2c_start(void)
{
   SET_SDA;	
   delay_us(4);
   SET_SCL;
   delay_us(4);
   RESET_SDA;	
   delay_us(4);
   RESET_SCL;	
   delay_us(4);
}

void i2c_stop(void)
{
   RESET_SDA;	
   delay_us(4);
   SET_SCL;	
   delay_us(4);
   SET_SDA;	
   delay_us(4);
}

void i2c_writebyte(uint8_t j) 
{
	uint8_t i,temp;
   	temp=j;
   	for (i=0;i<8;i++)
   	{
	   RESET_SCL;
	   delay_us(4);
	   if(temp&0x80)
	   {
		   SET_SDA;
	   }
	   else 
	   {
		   RESET_SDA;
	   }
	   delay_us(4);  
	   SET_SCL;		
	   delay_us(4);
	   temp=temp<<1;
   	}
   	RESET_SCL;
   	delay_us(4);
   	SET_SDA;
   	delay_us(4);
}

uint8_t i2c_readbyte(void)  
{
    uint8_t i,j,k=0;
    RESET_SCL;
    delay_us(4);
    SET_SDA;
    SDA_IN();
    for (i=0;i<8;i++)
    {  
      delay_us(4);
      SET_SCL; 
      delay_us(4);
      if(GPIO_ReadInputDataBit(IIC_GPIO_SDA, IIC_PIN_SDA)==RESET) j=0;
      else j=1;
      k=(k<<1)|j;
      RESET_SCL;
    }
    SDA_OUT();
    delay_us(4);
    return(k);
}

void i2c_answer(void)        
{
   //unsigned char i=0;
   SET_SCL;
   SDA_IN();
   delay_us(4);
   while(GPIO_ReadInputDataBit(IIC_GPIO_SDA, IIC_PIN_SDA)!=RESET);//((sda==1)&&(i<255))
   	 // i++;	 
   RESET_SCL;
   SDA_OUT();
   delay_us(4);
}



void iic_start(void) //OK
{
    SDA_OUT();
    SET_SDA;
    SET_SCL;
    delay_us(4);

    RESET_SDA;
    delay_us(4);
    RESET_SCL;
}


void iic_stop(void)
{
    SDA_OUT();
    
    RESET_SCL;
    RESET_SDA;
    
    delay_us(4);
    SET_SCL;
    SET_SDA;
    delay_us(4); //结束 在等一段时间
}


void iic_sendByte(uint8_t data)
{
    SDA_OUT();

    for(uint8_t i=0;i<8;i++) //MSB
    { 
        if(data & 0x80)
        {
            SET_SDA;
        }
        else
        {
            RESET_SDA;
        }
        
        delay_us(4);
        SET_SCL;
        delay_us(4);
        RESET_SCL;
        delay_us(4);
        
        data <<= 1;
    }

}


uint8_t iic_readByte(void)
{
    uint8_t data=0;
    SDA_IN();

    for(uint8_t i=0;i<8;i++) //MSB
    { 
        data <<= 1;
        SET_SCL;
        delay_us(4);
        data |= GPIO_ReadInputDataBit(IIC_GPIO_SDA, IIC_PIN_SDA);
        RESET_SCL;
        delay_us(4);
    }    
    return data;
}


void iic_wait_ack(void)
{
    //发完数据过后等待ack,所以在发送数据的时候,SDA是不确定高低的
    //SET_SDA;    // 释放总线,让从机控制产生ACK
    //delay_us(5);
    
    SDA_IN();       //
    SET_SCL;
    delay_us(4);
    while(GPIO_ReadInputDataBit(IIC_GPIO_SDA, IIC_PIN_SDA) != 0);
    RESET_SCL;
    delay_us(4);
}


void iic_send_ack(void)
{
    SDA_OUT();
    RESET_SDA;
    delay_us(4);
    SET_SCL;
    delay_us(4);
    RESET_SCL;
    delay_us(4);
    SET_SDA;
}
