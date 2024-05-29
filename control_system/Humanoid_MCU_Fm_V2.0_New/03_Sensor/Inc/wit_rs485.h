#ifndef __WIT_RS485_H
#define __WIT_RS485_H

#include "wit_c_sdk.h"
#include "stm32f4xx.h"

/*------------------------------------------姿态传感器数据更新参数---------------------------------------------------*/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80


#define WitControlPinEN(pinstate)  \
        do{\
           pinstate?GPIO_SetBits(GPIOB, GPIO_Pin_11):GPIO_ResetBits(GPIOB, GPIO_Pin_11); \
        }while(0);

extern char s_cDataUpdate;

void Poster_Wit_Init(void);

#endif /* __WIT_RS485_H */
