
#include "unitreeMotor.h"
#include "Delay.h"
#include "motor_rs485.h"

uint8_t A1Motorbuffer[2][A1B1MOTOR_RECVBUF_SIZE] = {0};
uint8_t B1Motorbuffer[2][A1B1MOTOR_RECVBUF_SIZE] = {0};
uint8_t SendToA1Motor_Buff[2][SEND_TO_A1B1MOTOR_BUFF_SIZE] = {0};
uint8_t SendToB1Motor_Buff[2][SEND_TO_A1B1MOTOR_BUFF_SIZE] = {0};
unsigned short B1_Motor_ID[6] = {0, 1, 2, 0, 1, 2}; 
unsigned short A1_Motor_ID[4] = {0, 1, 0, 1};

UnitreeA1B1 unitree_a1b1_motors;

void modify_data(MOTOR_send* motor_s);
bool extract_data(MOTOR_recv* motor_r);

void MotorTorquePack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id, float T, float W, float Pos, float K_P, float K_W);
void MotorStopPack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id);


inline void modify_data(MOTOR_send* motor_s){
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T*256;
    motor_s->motor_send_data.Mdata.W = motor_s->W*128;
    motor_s->motor_send_data.Mdata.Pos = (int)((motor_s->Pos/6.2832)*16384.0);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    
    if(motor_s->motortype == A1){
        motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
    }
    else if(motor_s->motortype == B1){
        motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*512;       
    }
    
    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}

/* 数据接收函数 */
inline bool extract_data(MOTOR_recv* motor_r){
    if(motor_r->motor_recv_data.CRCdata.u32 !=
        crc32_core((uint32_t*)(&(motor_r->motor_recv_data)), 18)){
        motor_r->correct = false;
        return motor_r->correct;
    }else{
        motor_r->motor_id = motor_r->motor_recv_data.head.motorID;
        motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
        motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
        motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
        motor_r->T = ((float)motor_r->motor_recv_data.Mdata.T) / 256;
        motor_r->W = ((float)motor_r->motor_recv_data.Mdata.W) / 128;
        motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

        motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
        motor_r->Pos = 6.2832*((float)motor_r->motor_recv_data.Mdata.Pos) / 16384;
        
        /*
		motor_r->gyro[0] = ((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176;
        motor_r->gyro[1] = ((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176;
        motor_r->gyro[2] = ((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176;
        
        motor_r->acc[0] = ((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132;
        motor_r->acc[1] = ((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132;
        motor_r->acc[2] = ((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132;
		*/

        motor_r->correct = true;
        return motor_r->correct;
    }
}

/* 力矩控制 */
void MotorTorquePack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id, float T, float W, float Pos, float K_P, float K_W)
{
    motor_s->motortype = motorType;
    motor_s->id = id;
    motor_s->mode = 10; //FOC闭环控制
    motor_s->T = T;
    motor_s->W = W;
    motor_s->Pos = Pos;
    motor_s->K_P = K_P;
    motor_s->K_W = K_W;

    modify_data(motor_s);
}

/* 速度阻尼模式 */
void MotorSpeedPack(MOTOR_send* motor_s, enum MotorType motortype, unsigned short id, float K_W)
{
	motor_s->motortype = motortype;
	motor_s->id = id;
	motor_s->mode = 10;
	motor_s->W = 0.0;
	motor_s->T = 0;
	motor_s->Pos = 0.0;
	motor_s->K_W = K_W;
	motor_s->K_P = 0.0;
	
	modify_data(motor_s);
}

/* 零力矩模式 */
void MotorZeroTorquePack(MOTOR_send* motor_s, enum MotorType motortype, unsigned short id)
{
	motor_s->motortype = motortype;
	motor_s->id = id;
	motor_s->mode = 10;
	motor_s->W = 0.0;
	motor_s->T = 0.0;
	motor_s->Pos = 0.0;
	motor_s->K_W = 0.0;
	motor_s->K_P = 0.0;
	
	modify_data(motor_s);
}

/* 位置PD控制 */
void MotorPosPack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id, float Pos, float K_P, float K_W)
{
	float Gear_Ratio = 0.0f;
	motor_s->motortype = motorType;
	Gear_Ratio = (float)(motorType == A1?GEAR_RATIO_A1:GEAR_RATIO_B1);
	motor_s->id = id;
	motor_s->mode = 10;
	motor_s->T = 0;
	motor_s->W = 0;
	motor_s->Pos = Pos * Gear_Ratio; //单位rad. |Pos| < 823549
    motor_s->K_P = K_P; 			//0<K_P<16
    motor_s->K_W = K_W;				//0<K_W<32
	
	modify_data(motor_s);
}

/* 电机空闲设置 */
void MotorStopPack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id)
{
    motor_s->motortype = motorType;
    motor_s->id = id;
    motor_s->mode = 0; //电机空闲

    modify_data(motor_s);
}

//接收数据函数
void A1B1RS485Receive(uint8_t rs485_id)
{
	switch(rs485_id)
	{
		case 0: //B1
			switch(B1Motorbuffer[rs485_id][2])
			{
				case 0: //该条485总线上串联三个B1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.b1_motor_recv[0].motor_recv_data, B1Motorbuffer[rs485_id], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.b1_motor_recv[0]);
				break;
				case 1: //该条485总线上串联三个B1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.b1_motor_recv[1].motor_recv_data, B1Motorbuffer[rs485_id], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.b1_motor_recv[1]);
				break;
				case 2: //该条485总线上串联三个B1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.b1_motor_recv[2].motor_recv_data, B1Motorbuffer[rs485_id], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.b1_motor_recv[2]);
				break;
				default:
					break;
			}
		break;
		case 1: //B1
			switch(B1Motorbuffer[rs485_id][2])
			{
				case 0: //该条485总线上串联三个B1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.b1_motor_recv[3].motor_recv_data, B1Motorbuffer[rs485_id], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.b1_motor_recv[3]);
				break;
				case 1: //该条485总线上串联三个B1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.b1_motor_recv[4].motor_recv_data, B1Motorbuffer[rs485_id], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.b1_motor_recv[4]);
				break;
				case 2: //该条485总线上串联三个B1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.b1_motor_recv[5].motor_recv_data, B1Motorbuffer[rs485_id], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.b1_motor_recv[5]);
				break;
				default:
					break;
			}
		break;
		case 2: //A1
			switch(A1Motorbuffer[rs485_id-2][2])
			{
				case 0: //该条485总线上串联两个个A1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.a1_motor_recv[0].motor_recv_data, A1Motorbuffer[rs485_id-2], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.a1_motor_recv[0]);
				break;
				case 1: //该条485总线上串联两个A1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.a1_motor_recv[1].motor_recv_data, A1Motorbuffer[rs485_id-2], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.a1_motor_recv[1]);
				break;
				default:
					break;
			}
		break;
		case 3: //A1
			switch(A1Motorbuffer[rs485_id-2][2])
			{
				case 0: //该条485总线上串联两个个A1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.a1_motor_recv[2].motor_recv_data, A1Motorbuffer[rs485_id-2], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.a1_motor_recv[2]);
				break;
				case 1: //该条485总线上串联两个A1电机
					memcpy((uint8_t *)&unitree_a1b1_motors.a1_motor_recv[3].motor_recv_data, A1Motorbuffer[rs485_id-2], A1B1MOTOR_RECVBUF_SIZE);
					extract_data(&unitree_a1b1_motors.a1_motor_recv[3]);
				break;
				default:
					break;
			}
		break;
		default:
			break;
	}
}
//发送数据函数


