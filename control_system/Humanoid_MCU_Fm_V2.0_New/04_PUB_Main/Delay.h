
#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

//void delay_init(u8 SYSCLK);
void SysTick_Init(void);
void delay_ms(u16 nms);
//void delay_us(u32 nus);

#endif
