#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"
#include "Global_Para.h"

#define GetSysTime_1MHzClk()	(*TIM4_CNT)

void TIM4_ProgramTest_Init(void);

#endif
