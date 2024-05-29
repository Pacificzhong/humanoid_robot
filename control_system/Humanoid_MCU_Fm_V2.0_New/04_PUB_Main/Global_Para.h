#ifndef __GLOBAL_MAIN_H
#define __GLOBAL_MAIN_H

#include "stm32f4xx.h"

#define FALSE			0
#define TRUE			1
/*---------------------------------------------任务参数-----------------------------------------------------*/
#define BOARD_NUM		1			
#define WIT_TEST_TASK	1
#define MOTOR_TEST_TASK	1
/*------------------------------------------A1B1电机减速比---------------------------------------------------*/
#define GEAR_RATIO_B1	8.66f
#define GEAR_RATIO_A1	9.10f
/*--------------------------------------------通信传输参数长度------------------------------------------------*/
#define	MOTOR_ENDINDEX					11
#define MOTOR_KNEE_RCVDATALEN			13
#define MOTOR_LINE_RCVDATALEN			16
#define	MOTOR_KNEE_SENDDATALEN			9
#define	MOTOR_LINE_SENDDATALEN			9
/*---------------------------------------------底层电机索引---------------------------------------------------*/
#define	LEFTLEG_B1MOTOR_X_INDEX			0
#define	LEFTLEG_B1MOTOR_Y_INDEX			1
#define LEFTLEG_B1MOTOR_Z_INDEX			2
#define	RIGHTLEG_B1MOTOR_X_INDEX		3 
#define RIGHTLEG_B1MOTOR_Y_INDEX		4
#define RIGHTLEG_B1MOTOR_Z_INDEX		5
#define	LEFTLEG_A1MOTOR_LEFT_INDEX		6
#define	LEFTLEG_A1MOTOR_RIGHT_INDEX		7
#define	RIGHTLEG_A1MOTOR_LEFT_INDEX		8
#define	RIGHTLEG_A1MOTOR_RIGHT_INDEX	9
#define	LEFTLEG_LINEMOTOR_INDEX			10
#define	RIGHTLEG_LINEMOTOR_INDEX		11
/*--------------------------------------------全局变量-------------------------------------------------------*/
extern volatile int cnt;
extern volatile int delay_time;
extern volatile int RunFlag;
extern volatile float pos_Kp;
extern volatile float pos_Ki;
extern volatile float pos_ref_ID0;
extern volatile float pos_ref_ID1;
extern volatile float pos_ref_ID2;
extern volatile float pos_ref_ID3;
extern volatile float pos_ref_ID4;
extern volatile float pos_ref_ID5;

extern uint16_t * TIM4_CNT;
/*----------------------------------------------END---------------------------------------------------------*/
#endif
