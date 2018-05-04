#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

//**************************************** 
// 定义MPU6050内部地址  
//**************************************** 
#define SMPLRT_DIV      0x19 //陀螺仪采样率，典型值：0x07(125Hz) 
#define CONFIG          0x1A //低通滤波频率，典型值：0x06(5Hz) 
#define GYRO_CONFIG     0x1B //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)  
#define ACCEL_CONFIG    0x1C //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)  
#define ACCEL_XOUT_H    0x3B 
#define ACCEL_XOUT_L    0x3C 
#define ACCEL_YOUT_H    0x3D 
#define ACCEL_YOUT_L    0x3E 
#define ACCEL_ZOUT_H    0x3F 
#define ACCEL_ZOUT_L    0x40 
#define TEMP_OUT_H      0x41 
#define TEMP_OUT_L      0x42 
#define GYRO_XOUT_H     0x43 
#define GYRO_XOUT_L     0x44 
#define GYRO_YOUT_H     0x45 
#define GYRO_YOUT_L     0x46 
#define GYRO_ZOUT_H     0x47 
#define GYRO_ZOUT_L     0x48 
#define PWR_MGMT_1      0x6B //电源管理，典型值：0x00(正常启用) 
#define WHO_AM_I        0x75 //IIC地址寄存器(默认数值0x68，只读) 
#define SlaveAddress    0xD0 //IIC写入时的地址字节数据，+1为读取

#define GYRO_FS_250   0x00	//
#define GYRO_FS_500   0x08
#define GYRO_FS_1000  0x10
#define GYRO_FS_2000  0x18
#define ACCEL_FS_2    0x00
#define ACCEL_FS_4    0x08
#define ACCEL_FS_8    0x10
#define ACCEL_FS_16   0x18

void MPU6050_WriteByte(uint8_t addr,uint8_t dat) ;
uint8_t MPU6050_ReadByte(uint8_t addr) ;
void MPU6050_Config(void);
uint16_t MPU6050_GetData(uint8_t REG_Address);

void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz );
#endif
