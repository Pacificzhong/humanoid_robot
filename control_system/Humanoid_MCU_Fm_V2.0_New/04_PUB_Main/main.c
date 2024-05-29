/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Global_Para.h"
#include "UnitreeMotor.h"
#include "motor_rs485.h"
#include "motion_control.h"
#include "Delay.h"
#include "Plc.h"
#include "Timer.h"
/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
void BSP_Init(void);
void Motor_Init(void);
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;
uint16_t *TIM4_CNT = (uint16_t *)(0x40000000 + 0x0800 + 0x24);
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* SysTick end of count event each 10ms */
	
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
	SysTick_Init();
	
	// 外设初始化
	BSP_Init();
	// 电机初始化
	Motor_Init();
	
	while(1)
	{
		Comm_PC_MainLoopSchedule();
	}
}

void BSP_Init(void)
{
	MOTOR1_B1_RS485_Configuration();
	delay_ms(1);
	MOTOR2_B1_RS485_Configuration();
	delay_ms(1);
	MOTOR3_A1_RS485_Configuration();
	delay_ms(1);
	MOTOR4_A1_RS485_Configuration();
	delay_ms(1);
	PLC_Computer_USART_Configuration(); //跟上位机通信初始化
	delay_ms(1);
	TIM4_ProgramTest_Init();
	delay_ms(1);
}

void Motor_Init(void)
{
	A1_B1_Motor_Init(&unitree_a1b1_motors); //获取电机当前信息
	delay_ms(1);
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
