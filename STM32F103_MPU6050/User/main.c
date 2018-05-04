/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "math.h"
#include "delay.h"
#include "mpu6050.h"
#include "oled.h"
//#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



extern float safe_asin(float v);
/* Private functions ---------------------------------------------------------*/

extern void I2C_MPU6050_Init(void);
int16_t bias[6] = {0};
void UpdateGyroBias(void)
{
    int32_t tmp[6] = {0};
	//扔掉前100次数据
//	for(uint8_t i=0;i<100;i++)
//	{
////		MPU6050_GetData(ACCEL_XOUT_H);
////		MPU6050_GetData(ACCEL_YOUT_H);
////		MPU6050_GetData(ACCEL_ZOUT_H);

////		MPU6050_GetData(GYRO_XOUT_H);
////		MPU6050_GetData(GYRO_YOUT_H);
////		MPU6050_GetData(GYRO_ZOUT_H);  		
//	}	
		
	
	for(uint8_t i=0;i<100;i++) //计算平均误差
	{
		tmp[0] += MPU6050_GetData(ACCEL_XOUT_H);
		tmp[1] += MPU6050_GetData(ACCEL_YOUT_H);
		tmp[2] += MPU6050_GetData(ACCEL_ZOUT_H);

		tmp[3] += MPU6050_GetData(GYRO_XOUT_H);
		tmp[4] += MPU6050_GetData(GYRO_YOUT_H);
		tmp[5] += MPU6050_GetData(GYRO_ZOUT_H);
        delay_ms(5);    
	}

    //bias[0] = tmp[0]/10;
    //bias[1] = tmp[1]/10;
    //bias[2] = tmp[2]/10;
    bias[3] = tmp[3]/100;
    bias[4] = tmp[4]/100;
    bias[5] = tmp[5]/100;
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//  /* Enable the USARTz Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}

int16_t a_x=0,a_y=0,a_z=0,a=0;
int16_t g_x=0,g_y=0,g_z=0;	
float roll=0;
float pitch=0;
float yaw=0;		
int main(void)
{
    float ax_rad,ay_rad,az_rad;
    float gx_rad,gy_rad,gz_rad;
    uint32_t lastcounter = 0;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1|
						   RCC_APB2Periph_GPIOA,ENABLE);
    
	USART_DeInit(USART1);
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	
	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
    NVIC_Configuration();
    delay_init();
//    OLED_Init();			 
//    OLED_Clear(); 
//    OLED_ShowString(6,3,"0.96' OLED TEST",16);    
#if 1    
    MPU6050_Config();
    delay_ms(100);
    UpdateGyroBias();
#endif    
    
    /* Infinite loop */
    while (1)
    {
 
        
#if 1       
        a_x = MPU6050_GetData(ACCEL_XOUT_H);
        a_y = MPU6050_GetData(ACCEL_YOUT_H);
        a_z = MPU6050_GetData(ACCEL_ZOUT_H); 
       
        //温度读取
        //temp = ((float) ( (int16_t)((rawIMU[6] << 8) | rawIMU[7]) ) + 12412.0) / 340.0;
        
        //将三轴加速度预处理为弧度制
        ax_rad = a_x/8192.0f;  		    //加速度量程 +-4g/S
        ay_rad = a_y/8192.0f;	 		//转换关系8192LSB/g
        az_rad = a_z/8192.0f;
        
       // printf("[a %f %f %f    ",a_x,a_y,a_z);
        
        //三轴陀螺仪原始数据
        g_x = MPU6050_GetData(GYRO_XOUT_H) - bias[3];
        g_y = MPU6050_GetData(GYRO_YOUT_H) - bias[4];
        g_z = MPU6050_GetData(GYRO_ZOUT_H) - bias[5]; 
        
       
        gx_rad = g_x/65.5f;
        gy_rad = g_y/65.5f;	//陀螺仪量程 +-500度/S
        gz_rad = g_z/65.5f;	//转换关系65.5LSB/度	
        // printf(" g %f %f %f ] \r\n",g_x,g_y,g_z);
            
        //三个方向加速度的和
        a = sqrt(ax_rad*ax_rad + ay_rad*ay_rad + az_rad*az_rad);//加速度的大小   
        //MadgwickAHRSupdateIMU(gx_rad*0.0174533f, gy_rad*0.0174533f, gz_rad*0.0174533f, ax_rad, ay_rad, az_rad);
        MahonyAHRSupdateIMU(gx_rad*0.0174533f, gy_rad*0.0174533f, gz_rad*0.0174533f, ax_rad, ay_rad, az_rad);
        
		roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f; //横滚 x
		pitch = safe_asin(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;//俯仰 y
		yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29578f ; //偏航 z

//        if(roll < 0)
//          roll = 360 + roll ;
//        if(pitch < 0)
//          pitch = 360 + pitch ;
//        if(yaw < 0)
//          yaw = 360 + yaw ;
//        if((get_SystickCounter() - lastcounter)>=300)
//        {
//            lastcounter = get_SystickCounter();
//            printf("[ %f\t %f\t %f\t ] \r\n",roll,pitch,yaw);
//        }
        mpu6050_send_data(a_x,a_y,a_z,g_x,g_y,g_z); //数据曲线
        usart1_report_imu(a_x,a_y,a_z,g_x,g_y,g_z,(int)(roll*100),(int)(pitch*100),(int)(yaw*10)); //飞行姿态
        delay_ms(5); 
#endif
  		
    }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
