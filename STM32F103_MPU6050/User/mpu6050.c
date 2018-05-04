#include "mpu6050.h"
#include "i2c.h"
#include "delay.h"
#include "stdio.h"
#include "math.h"

extern void I2C_ByteWrite(uint8_t REG_Address,uint8_t REG_data);
extern uint8_t I2C_ByteRead(uint8_t REG_Address);

void MPU6050_WriteByte(uint8_t addr,uint8_t dat) 
{
  //  I2C_ByteWrite(addr,dat);
    iic_start();
    iic_sendByte(SlaveAddress); 
    iic_wait_ack();
    iic_sendByte(addr); 
    iic_wait_ack();
    iic_sendByte(dat); 
    iic_wait_ack();
    iic_stop();
}

uint8_t MPU6050_ReadByte(uint8_t addr) 
{
   // return I2C_ByteRead(addr);
    uint8_t RegData;
    iic_start();
    iic_sendByte(SlaveAddress); 
    iic_wait_ack();
    iic_sendByte(addr); 
    iic_wait_ack();
    iic_start();
    iic_sendByte(SlaveAddress+1);
    iic_wait_ack();
    RegData = iic_readByte();
    iic_stop();
    return RegData;
}

void MPU6050_Config(void) 
{      
    uint16_t id = 0;
    IIC_Config();
    printf("mpu6050 开始初始化\r\n");
    MPU6050_WriteByte(PWR_MGMT_1, 0x01 << 7);   //设备复位
    delay_ms(100);
    MPU6050_WriteByte(PWR_MGMT_1, 0x00);        //配置使用内部8MHz时钟 
    MPU6050_WriteByte(SMPLRT_DIV, 0x07);        // 1KHz/(1+7)的采样率  125Hz
    MPU6050_WriteByte(CONFIG, 0x04);            //DLPF_CFG 设置
    MPU6050_WriteByte(GYRO_CONFIG, 1 << 3);       // -500°/s ~ +500°/s
    MPU6050_WriteByte(ACCEL_CONFIG,1 << 3);      //-4g ~  +4g
    
    id = MPU6050_ReadByte(WHO_AM_I);
    while(id != 0x68);
    printf("mpu6050 初始化完成\r\n");
    MPU6050_WriteByte(PWR_MGMT_1,0X01);
}

// 角速度
// 250  º/s    131      LSB/(º/s)
// 500  º/s    65.5     LSB/(º/s)
// 1000 º/s    32.8     LSB/(º/s)
// 2000 º/s    16.4     LSB/(º/s)
// 加速度
// 2    g      16,384   LSB/g
// 4    g      8192     LSB/g 
// 8    g      4096     LSB/g
// 16   g      2048     LSB/g 

uint16_t MPU6050_GetData(uint8_t REG_Address) //0x3B
{  
    uint8_t  H,L; 
    H=MPU6050_ReadByte(REG_Address);  
    L=MPU6050_ReadByte(REG_Address+1);  
    return ((H<<8)+L);   //合成数据 
} 




//´«ËÍÊý¾Ý¸øÄäÃûËÄÖáÉÏÎ»»úÈí¼þ(V2.6°æ±¾)
//fun:¹¦ÄÜ×Ö. 0XA0~0XAF
//data:Êý¾Ý»º´æÇø,×î¶à28×Ö½Ú!!
//len:dataÇøÓÐÐ§Êý¾Ý¸öÊý
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//×î¶à28×Ö½ÚÊý¾Ý 
	send_buf[len+3]=0;	//Ð£ÑéÊýÖÃÁã
	send_buf[0]=0X88;	//Ö¡Í·
	send_buf[1]=fun;	//¹¦ÄÜ×Ö
	send_buf[2]=len;	//Êý¾Ý³¤¶È
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//¸´ÖÆÊý¾Ý
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//¼ÆËãÐ£ÑéºÍ	
	for(i=0;i<len+4;i++)
    {
          USART_SendData(USART1, send_buf[i]);
          /* Loop until the end of transmission */
          while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
          {}
    }
}
//·¢ËÍ¼ÓËÙ¶È´«¸ÐÆ÷Êý¾ÝºÍÍÓÂÝÒÇÊý¾Ý
//aacx,aacy,aacz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄ¼ÓËÙ¶ÈÖµ
//gyrox,gyroy,gyroz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄÍÓÂÝÒÇÖµ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz )
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
//	tbuf[12]=(Yaw>>8)&0XFF;
//	tbuf[13]=Yaw&0XFF;
//	tbuf[14]=(Pitch>>8)&0XFF;
//	tbuf[15]=Pitch&0XFF;
//	tbuf[16]=(Roll>>8)&0XFF;
//	tbuf[17]=Roll&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//×Ô¶¨ÒåÖ¡,0XA1
}	


//Í¨¹ý´®¿Ú1ÉÏ±¨½áËãºóµÄ×ËÌ¬Êý¾Ý¸øµçÄÔ
//aacx,aacy,aacz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄ¼ÓËÙ¶ÈÖµ
//gyrox,gyroy,gyroz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄÍÓÂÝÒÇÖµ
//roll:ºá¹ö½Ç.µ¥Î»0.01¶È¡£ -18000 -> 18000 ¶ÔÓ¦ -180.00  ->  180.00¶È
//pitch:¸©Ñö½Ç.µ¥Î» 0.01¶È¡£-9000 - 9000 ¶ÔÓ¦ -90.00 -> 90.00 ¶È
//yaw:º½Ïò½Ç.µ¥Î»Îª0.1¶È 0 -> 3600  ¶ÔÓ¦ 0 -> 360.0¶È
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//Çå0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//·É¿ØÏÔÊ¾Ö¡,0XAF
} 



// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return 3.1415926535f / 2;
	}
	if (v <= -1.0f)
	{
		return 3.1415926535f / 2;
	}
	return asin(v);
}






