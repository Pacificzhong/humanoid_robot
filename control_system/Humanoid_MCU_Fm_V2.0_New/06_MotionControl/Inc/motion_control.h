
#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "stm32f4xx.h"
#include "unitreeMotor.h"
#include "motor_rs485.h"
#include "Global_Para.h"

void A1_B1_Motor_Init(UnitreeA1B1 *knee_motor);
void B1_Motor_Union_Motion(uint8_t motor_num, float pos_ref);
void Motor_Jogging_Test(uint8_t board_num, uint8_t motor_num, float pos, float K_P, float K_W);

#endif
