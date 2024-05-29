
#include "Plc.h"

uint16_t Frame_PC_State = FRAME_NO_ARRIVE;
uint16_t Frame_PCData_State = FRAME_PROCESS_NO_RDY;
uint16_t gFrame3P5CntStartFlag = c3P5END;
uint16_t wFrame3P5Time = 9; //9us
uint8_t	RcvByteLen = 0;
uint8_t RcvDataLen = 0;
uint8_t DataLength = 0;
uint8_t PLC_Rcv_Buffer[MAX_RCV_BUFFER_SIZE] = {0};
uint8_t PLC_RcvData_Buffer[MAX_RCV_BUFFER_SIZE - 5] = {0};
MOTOR_RCVPC_DATA_ALLBUFFER RcvPCBuffer;
MOTOR_SENDPC_DATA_ALLBUFFER SendPCBuffer;

void PLC_Computer_USART_Configuration(void);
void Comm_PC_MainLoopSchedule(void);

void RcvIsr(void);
void RcvDataDeal(void);
void DataAllRcvJudge(void);
uint8_t COMMWrRdPara(const uint16_t group, const uint16_t DataLength, uint8_t *PLC_Rcv_Buffer, uint8_t CmdMode);

//放在主循环的函数
void Comm_PC_MainLoopSchedule(void)
{
	DataAllRcvJudge();
	if(Frame_PCData_State == FRAME_PROCESS_RDY) //接收来自PC端的数据已完成,开始解析协议
	{
		Frame_PCData_State = FRAME_PROCESS_NO_RDY;
		RcvDataDeal();
		memset(PLC_Rcv_Buffer, 0, sizeof(PLC_Rcv_Buffer));
		RcvByteLen = 0;
		RcvDataLen = 0;
	}
}

void DataAllRcvJudge(void)
{
//	if(RcvByteLen >= 17)
//	{
//		Frame_PCData_State = FRAME_PROCESS_RDY;
//		RcvByteLen = 0;
//	}
	static uint32_t gFrame3P5TimeCntNew;
	static uint32_t gFrame3P5TimeCntOld;
	uint16_t temp;
	
	if(gFrame3P5CntStartFlag == c3P5START)
    {
        gFrame3P5TimeCntNew = GetSysTime_1MHzClk();
        gFrame3P5TimeCntOld = gFrame3P5TimeCntNew;
        gFrame3P5CntStartFlag = c3P5END;
    }
	else if(Frame_PC_State == FRAME_NO_ARRIVE)
    {
        gFrame3P5TimeCntNew = GetSysTime_1MHzClk();
		temp = (uint16_t)(gFrame3P5TimeCntNew-gFrame3P5TimeCntOld);
        if((uint16_t)(gFrame3P5TimeCntNew-gFrame3P5TimeCntOld) > wFrame3P5Time)
        {
            Frame_PC_State = FRAME_ARRIVE;
        }
    }
	if((Frame_PC_State == FRAME_ARRIVE)&&(RcvByteLen != 0))
    {
		if(RcvByteLen >= 17)
		{
			//为有效帧
			Frame_PCData_State = FRAME_PROCESS_RDY;
		}
		else
		{
			memset(PLC_Rcv_Buffer, 0, sizeof(PLC_Rcv_Buffer));
		}
     }
}

//报文处理函数，放在MainLoop中
void RcvDataDeal(void)
{
	uint8_t cmd;
	uint16_t group;
	//uint8_t control_mode;
	uint8_t ret;
	//待加入CRC校验
	
	cmd = PLC_Rcv_Buffer[CMD]; //0~12，指示电机数量
	group = (uint16_t)PLC_Rcv_Buffer[GROUPH] << 8 | PLC_Rcv_Buffer[GROUPL];
	//control_mode = PLC_Rcv_Buffer[MODE];
	if(cmd >= 1)
	{
		DataLength = (uint16_t)PLC_Rcv_Buffer[DATALENH]<<8 | PLC_Rcv_Buffer[DATALENL];
	}
	if(DataLength >= 1)
	{
		ret = COMMWrRdPara(group, DataLength, PLC_Rcv_Buffer, WRITE_MODE);
	}
}

uint8_t COMMWrRdPara(const uint16_t group, const uint16_t DataLength, uint8_t *PLC_Rcv_Buffer, uint8_t CmdMode)
{
	uint16_t RcvDataCnt = 0;
	uint8_t MotorNumCnt = 0;
	switch(CmdMode)
	{
		case WRITE_MODE:
			for(int i = 0; i < DataLength;i++)
			{
				PLC_RcvData_Buffer[RcvDataCnt] = *(PLC_Rcv_Buffer + 6);
				RcvDataCnt++;
				PLC_Rcv_Buffer++;
			}
			if(group & 0x0001) //LEFTLEG_B1MOTOR_X
			{
				MotorNumCnt++;
				RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
				RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_W_INDEX] 		 = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
				RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
				RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
				RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
				RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
			}
			if(group & 0x0002)
			{
				RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_W_INDEX] 		 = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0004)
			{

				RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_W_INDEX] 		 = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0008)
			{
				RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0010)
			{
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0020)
			{
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0040)
			{
				RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_W_INDEX] 	    = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0080)
			{
				RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_W_INDEX] 	     = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0100)
			{
				RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_W_INDEX] 	     = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0200)
			{
				RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0400)
			{
				RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_W_INDEX] 	     = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
			if(group & 0x0800)
			{
				RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+1];
				RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+2]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+3];
				RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+4]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+5];
				RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+6]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+7];
				RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+8]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+9];
				RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+10]<<8 | PLC_RcvData_Buffer[MotorNumCnt*UNITREE_MOTOR_BYTE+11];
				MotorNumCnt++;
			}
//			switch(group)
//			{
//				case LEFTLEG_B1MOTOR_X_INDEX:
//					RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_W_INDEX] 		 = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_X[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case LEFTLEG_B1MOTOR_Y_INDEX:
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_W_INDEX] 		 = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Y[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case LEFTLEG_B1MOTOR_Z_INDEX:
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_W_INDEX] 		 = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.LEFTLEG_B1MOTOR_Z[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case RIGHTLEG_B1MOTOR_X_INDEX:
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_X[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case RIGHTLEG_B1MOTOR_Y_INDEX:
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Y[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case RIGHTLEG_B1MOTOR_Z_INDEX:
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.RIGHTLEG_B1MOTOR_Z[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case LEFTLEG_A1MOTOR_LEFT_INDEX:
//					RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_W_INDEX] 	    = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_LEFT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case LEFTLEG_A1MOTOR_RIGHT_INDEX:
//					RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_W_INDEX] 	     = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.LEFTLEG_A1MOTOR_RIGHT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case RIGHTLEG_A1MOTOR_LEFT_INDEX:
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_W_INDEX] 	     = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_LEFT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case RIGHTLEG_A1MOTOR_RIGHT_INDEX:
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.RIGHTLEG_A1MOTOR_RIGHT[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case LEFTLEG_LINEMOTOR_INDEX:
//					RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_W_INDEX] 	     = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.LEFTLEG_LINEMOTOR[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				case RIGHTLEG_LINEMOTOR_INDEX:
//					RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_TORQUE_INDEX]  = (uint16_t)PLC_RcvData_Buffer[0]<<8 | PLC_RcvData_Buffer[1];
//					RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_W_INDEX] 	  = (uint16_t)PLC_RcvData_Buffer[2]<<8 | PLC_RcvData_Buffer[3];
//					RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_POS_H16_INDEX] = (uint16_t)PLC_RcvData_Buffer[4]<<8 | PLC_RcvData_Buffer[5];
//					RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_POS_L16_INDEX] = (uint16_t)PLC_RcvData_Buffer[6]<<8 | PLC_RcvData_Buffer[7];
//					RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_KW_INDEX]      = (uint16_t)PLC_RcvData_Buffer[8]<<8 | PLC_RcvData_Buffer[9];
//					RcvPCBuffer.RIGHTLEG_LINEMOTOR[REF_KP_INDEX]      = (uint16_t)PLC_RcvData_Buffer[10]<<8 | PLC_RcvData_Buffer[11];
//					break;
//				default:
//						return 0;
//					break;
//			}
			break;
			return 1;
	}
}

/**
 * @brief 串口配置
 * @param[in] void
 */
void PLC_Computer_USART_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_Init(GPIOD, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_OverSampling8Cmd(USART3, ENABLE);
	USART_InitStructure.USART_BaudRate = 3000000; //通信波特率4M bit/s
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //接收不为空的中断
	USART_Cmd(USART3, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    //USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); //定长数据，采用DMA进行发送
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//	{ //RX
//		DMA_InitTypeDef dma;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//        DMA_DeInit(DMA1_Stream1);
//        while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)
//        {
//        };
//        dma.DMA_Channel = DMA_Channel_4; //USART3_RX Channel
//        dma.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
//        dma.DMA_Memory0BaseAddr = (uint32_t)PLC_Rcv_Buffer;
//        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//        dma.DMA_BufferSize = PC_RCV_BUFFER_SIZE;
//        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dma.DMA_Mode = DMA_Mode_Circular;
//        dma.DMA_Priority = DMA_Priority_Medium;
//        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//        DMA_Init(DMA1_Stream1, &dma);
//        DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
//		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
//        DMA_Cmd(DMA1_Stream1, ENABLE);
//	}
//	{ //  TX
//        DMA_InitTypeDef dma;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//        DMA_DeInit(DMA2_Stream7);
//        while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
//        {
//        };
//        dma.DMA_Channel = DMA_Channel_4;
//        dma.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
//        dma.DMA_Memory0BaseAddr = (uint32_t)SendToB1Motor_Buff[0];
//        dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//        dma.DMA_BufferSize = SEND_TO_A1B1MOTOR_BUFF_SIZE;
//        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dma.DMA_Mode = DMA_Mode_Normal;
//        dma.DMA_Priority = DMA_Priority_VeryHigh;
//        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//        DMA_Init(DMA2_Stream7, &dma);
//        DMA_Cmd(DMA2_Stream7, DISABLE);
//        DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
//    }
}


void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		RcvIsr();
	}
}

void RcvIsr(void)
{
	uint8_t temp = 0;
	if(Frame_PC_State == FRAME_ARRIVE)
	{
		RcvByteLen = 0;
		RcvDataLen = 0;
	}
	temp = USART_ReceiveData(USART3);
	PLC_Rcv_Buffer[RcvByteLen++] = temp;
	gFrame3P5CntStartFlag= c3P5START;
	Frame_PC_State = FRAME_NO_ARRIVE;
}
