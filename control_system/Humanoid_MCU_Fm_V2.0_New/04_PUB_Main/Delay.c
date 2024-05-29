
#include "Delay.h"
#include "misc.h"

static uint16_t  fac_us=0;
static uint16_t fac_ms=0;

static volatile uint32_t sv_uiDelay = 0;
void SysTick_Init(void)
{	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	fac_us=SystemCoreClock/1000000;
	fac_ms=(uint16_t)fac_us*1000;
}

void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;
	SysTick->VAL =0x00;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL =0X00;	    
}

////仿原子延时，不进入systic中断
//void delay_us(u32 nus)
//{
// u32 temp;
// SysTick->LOAD = 168*nus;
// SysTick->VAL=0X00;//清空计数器
// SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
// do
// {
//  temp=SysTick->CTRL;//读取当前倒计数值
// }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
//     SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
//}

//void delay_ms(u16 nms)
//{
// u32 temp;
// SysTick->LOAD = 168000*nms;
// SysTick->VAL=0X00;//清空计数器
// SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
// do
// {
//  temp=SysTick->CTRL;//读取当前倒计数值
// }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
//    SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
//}

