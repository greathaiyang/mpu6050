#include "stm32f10x.h"
#include "delay.h"

static __IO uint32_t gSystickCount=0;

void delay_init(void)
{
    RCC_ClocksTypeDef RCC_ClocksStruct;
    
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    
    RCC_GetClocksFreq(&RCC_ClocksStruct);
    SysTick_Config(RCC_ClocksStruct.SYSCLK_Frequency / 1000);//1us 一次systick 
    
}


void delay_ms(__IO uint32_t msTime)
{ 
    uint32_t lgSystickCount = gSystickCount;
    while((gSystickCount - lgSystickCount) < msTime);
}

void delay_us(uint16_t usTime)
{    
   uint16_t i = 0;  
   while(usTime--)
   {
      i = 10;   //随便定义的一个值, 无意义, 纯粹是消耗CPU时间而已, 可能会编译器被优化  
      while(i--);    
   }
}


void SystickConut_Increase(void)
{
    gSystickCount++;
}


uint32_t get_SystickCounter(void)
{
    uint32_t tmp = gSystickCount;
    return tmp;
}
