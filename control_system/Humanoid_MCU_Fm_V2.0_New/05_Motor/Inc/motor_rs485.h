
#ifndef __MOTOR_RS485_H
#define __MOTOR_RS485_H

#include "motor_msg.h"
#include "unitreeMotor.h"
#include "stm32f4xx.h"
#include "Delay.h"
/*------------------------------------------A1B1������ݻ����С---------------------------------------------------*/
#define A1B1MOTOR_RECVBUF_SIZE				sizeof(ServoComdDataV3)		//�����������ݴ�С
#define SEND_TO_A1B1MOTOR_BUFF_SIZE 		sizeof(MasterComdDataV3)	//�����������ݴ�С
/*--------------------------------------------A1B1���485����-----------------------------------------------------*/
#define MOTOR1_B1_485_ID					0
#define MOTOR2_B1_485_ID					1
#define MOTOR3_A1_485_ID					2
#define MOTOR4_A1_485_ID					3
/*------------------------------------------A1B1�����뷢�ͻ�����---------------------------------------------------*/
extern uint8_t B1Motorbuffer[2][A1B1MOTOR_RECVBUF_SIZE];				//B1���ջ�����,��UnitreeMotor.c�ļ��ж���
extern uint8_t A1Motorbuffer[2][A1B1MOTOR_RECVBUF_SIZE];				//A1���ջ�����
extern uint8_t SendToB1Motor_Buff[2][SEND_TO_A1B1MOTOR_BUFF_SIZE];  	//B1���ͻ�����
extern uint8_t SendToA1Motor_Buff[2][SEND_TO_A1B1MOTOR_BUFF_SIZE];  	//A1���ͻ�����
/*------------------------------------------B1�������1��������----------------------------------------------------*/
#define MOTOR1_485_CONTROL_PORT				GPIOG
#define	MOTOR1_485_CONTROL_PIN				GPIO_Pin_11
#define MOTOR1_485_CONTROL_PORT_RCC			RCC_AHB1Periph_GPIOG
#define MOTOR1_485_START_READ()				GPIO_ResetBits(MOTOR1_485_CONTROL_PORT, MOTOR1_485_CONTROL_PIN)	 
#define MOTOR1_485_START_WRITE()			GPIO_SetBits(MOTOR1_485_CONTROL_PORT, MOTOR1_485_CONTROL_PIN)
#define MOTOR1_RS485_SEND_DMA_IRQHandler	DMA2_Stream7_IRQHandler
#define MOTOR1_RS485_RECV_USARTx_IRQHandler	USART1_IRQHandler

void MOTOR1_B1_RS485_Configuration(void);
void MOTOR1_RS485_SEND(void);
/*------------------------------------------B1�������2��������----------------------------------------------------*/
#define	MOTOR2_485_CONTROL_PORT				GPIOG
#define	MOTOR2_485_CONTROL_PIN				GPIO_Pin_12
#define MOTOR2_485_CONTROL_PORT_RCC			RCC_AHB1Periph_GPIOG
#define MOTOR2_485_START_READ()				GPIO_ResetBits(MOTOR2_485_CONTROL_PORT, MOTOR2_485_CONTROL_PIN)	
#define MOTOR2_485_START_WRITE()			GPIO_SetBits(MOTOR2_485_CONTROL_PORT, MOTOR2_485_CONTROL_PIN)
#define MOTOR2_RS485_SEND_DMA_IRQHandler	DMA2_Stream6_IRQHandler
#define MOTOR2_RS485_RECV_USARTx_IRQHandler	USART6_IRQHandler

void MOTOR2_B1_RS485_Configuration(void);
void MOTOR2_RS485_SEND(void);
/*------------------------------------------A1�������3��������----------------------------------------------------*/
#define	MOTOR3_485_CONTROL_PORT				GPIOG
#define	MOTOR3_485_CONTROL_PIN				GPIO_Pin_13
#define MOTOR3_485_CONTROL_PORT_RCC			RCC_AHB1Periph_GPIOG
#define MOTOR3_485_START_READ()				GPIO_ResetBits(MOTOR3_485_CONTROL_PORT, MOTOR3_485_CONTROL_PIN)	
#define MOTOR3_485_START_WRITE()			GPIO_SetBits(MOTOR3_485_CONTROL_PORT, MOTOR3_485_CONTROL_PIN)
#define MOTOR3_RS485_SEND_DMA_IRQHandler	DMA1_Stream6_IRQHandler
#define MOTOR3_RS485_RECV_USARTx_IRQHandler	USART2_IRQHandler

void MOTOR3_A1_RS485_Configuration(void);
void MOTOR3_RS485_SEND(void);
/*------------------------------------------A1�������4��������----------------------------------------------------*/
#define	MOTOR4_485_CONTROL_PORT				GPIOG
#define	MOTOR4_485_CONTROL_PIN				GPIO_Pin_14
#define MOTOR4_485_CONTROL_PORT_RCC			RCC_AHB1Periph_GPIOG
#define MOTOR4_485_START_READ()				GPIO_ResetBits(MOTOR4_485_CONTROL_PORT, MOTOR4_485_CONTROL_PIN)	
#define MOTOR4_485_START_WRITE()			GPIO_SetBits(MOTOR4_485_CONTROL_PORT, MOTOR4_485_CONTROL_PIN)
#define MOTOR4_RS485_SEND_DMA_IRQHandler	DMA1_Stream7_IRQHandler
#define MOTOR4_RS485_RECV_USARTx_IRQHandler	UART5_IRQHandler

void MOTOR4_A1_RS485_Configuration(void);
void MOTOR4_RS485_SEND(void);
/*----------------------------------------------------------------------------------------------------------------*/




#endif
