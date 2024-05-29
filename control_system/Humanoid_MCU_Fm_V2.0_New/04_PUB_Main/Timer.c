
#include "Timer.h"

void TIM4_ProgramTest_Init(void)
{
	//��ʱ������
	//TIMER4 84MHz
	
	/* TIMER4 configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //ʹ��TIM4ʱ��
	TIM_TimeBaseInitStructure.TIM_Period = 65535; 	// 
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;  // 1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);//��ʼ��TIM4
	TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��4
}
