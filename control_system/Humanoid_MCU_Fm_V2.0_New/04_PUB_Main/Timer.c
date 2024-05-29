
#include "Timer.h"

void TIM4_ProgramTest_Init(void)
{
	//定时器配置
	//TIMER4 84MHz
	
	/* TIMER4 configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //使能TIM4时钟
	TIM_TimeBaseInitStructure.TIM_Period = 65535; 	// 
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;  // 1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);//初始化TIM4
	TIM_Cmd(TIM4, ENABLE); //使能定时器4
}
