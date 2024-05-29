#include <stdio.h>
#include "wit_rs485.h"
#include "stm32f4xx.h"
#include "misc.h"
#include "Delay.h"

char s_cDataUpdate = 0;

void Poster_Wit_RS485_Configuration(unsigned int uiBaud);
void Uart_RS485_Send(unsigned char *p_data, unsigned int uiSize);
void Poster_Wit_RS485_Send(uint8_t *p_data, uint32_t uiSize);
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
void Delayms(uint16_t ucMs);

void Poster_Wit_Init(void)
{
	Poster_Wit_RS485_Configuration(9600);
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(Poster_Wit_RS485_Send);
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(Delayms);
}

inline void Poster_Wit_RS485_Configuration(unsigned int uiBaud)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
    //串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
    
    //485 control pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG, GPIO_Pin_11);

    /* 如果是通信频率不够，需要加入这句  */
    // USART_OverSampling8Cmd(MOTOR2_RS485_RCC_AxBxPeriph_UARTx, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
//	NVIC_Init(&NVIC_InitStructure);	
	
	USART_InitStructure.USART_BaudRate = uiBaud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_ClearFlag(USART2,USART_FLAG_TXE);
	USART_Cmd(USART2, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*void USART2_IRQHandler(void)
{
	unsigned char ucTemp = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		ucTemp = USART_ReceiveData(USART2);
		WitSerialDataIn(ucTemp);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}*/

inline void Uart_RS485_Send(unsigned char *p_data, unsigned int uiSize)
{	
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART_SendData(USART2, *p_data++);
		
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

inline void Poster_Wit_RS485_Send(uint8_t *p_data, uint32_t uiSize)
{
	GPIO_SetBits(GPIOG, GPIO_Pin_11);
	Uart_RS485_Send(p_data, uiSize);
	GPIO_ResetBits(GPIOG, GPIO_Pin_11);
	delay_ms(300);
}

inline void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

inline void Delayms(uint16_t ucMs)
{
	delay_ms(ucMs);
}
