#include "Global_Para.h"
#include "Plc.h"
//接收来自PC端的数据
const int16_t	*MotorReceivefromPc_Buffer[MOTOR_ENDINDEX + 1] = {
	(int16_t *)&RcvPCBuffer.LEFTLEG_B1MOTOR_X[0],
	(int16_t *)&RcvPCBuffer.LEFTLEG_B1MOTOR_Y[0],
	(int16_t *)&RcvPCBuffer.LEFTLEG_B1MOTOR_Z[0],
	
	(int16_t *)&RcvPCBuffer.LEFTLEG_LINEMOTOR[0],
	
	(int16_t *)&RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[0],
	(int16_t *)&RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[0],
	
	(int16_t *)&RcvPCBuffer.RIGHTLEG_B1MOTOR_X[0],
	(int16_t *)&RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[0],
	(int16_t *)&RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[0],
	
	(int16_t *)&RcvPCBuffer.RIGHTLEG_LINEMOTOR[0],
	
	(int16_t *)&RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[0],
	(int16_t *)&RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[0],
};
//发送给PC端的数据
const int16_t	*MotorSendtoPc_Buffer[MOTOR_ENDINDEX + 1] = {
	(int16_t *)&SendPCBuffer.LEFTLEG_B1MOTOR_X[0],
	(int16_t *)&SendPCBuffer.LEFTLEG_B1MOTOR_Y[0],
	(int16_t *)&SendPCBuffer.LEFTLEG_B1MOTOR_Z[0],
	
	(int16_t *)&SendPCBuffer.LEFTLEG_LINEMOTOR[0],
	
	(int16_t *)&SendPCBuffer.LEFTLEG_A1MOTOR_LEFT[0],
	(int16_t *)&SendPCBuffer.LEFTLEG_A1MOTOR_RIGHT[0],
	
	(int16_t *)&SendPCBuffer.RIGHTLEG_B1MOTOR_X[0],
	(int16_t *)&SendPCBuffer.RIGHTLEG_B1MOTOR_Y[0],
	(int16_t *)&SendPCBuffer.RIGHTLEG_B1MOTOR_Z[0],
	
	(int16_t *)&SendPCBuffer.RIGHTLEG_LINEMOTOR[0],
	
	(int16_t *)&SendPCBuffer.RIGHTLEG_A1MOTOR_LEFT[0],
	(int16_t *)&SendPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[0],
};
