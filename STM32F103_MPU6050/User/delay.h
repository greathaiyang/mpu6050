#ifndef	__DELAY_H
#define	__DELAY_H


#include "stm32f10x.h"


void delay_us(uint16_t usTime);
void delay_ms(__IO uint32_t msTime);
void delay_init(void);
void SystickConut_Increase(void);
uint32_t get_SystickCounter(void);
#endif
