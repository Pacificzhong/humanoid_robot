
#include "motion_control.h"

void A1_B1_Motor_Init(UnitreeA1B1 *knee_motor);
void Motor_Jogging_Test(uint8_t board_num, uint8_t motor_num, float pos, float K_P, float K_W);

//A1�����B1��������У׼����
void A1_B1_Motor_Init(UnitreeA1B1 *knee_motor)
{
	//int8_t init_finish = FALSE;
	MOTOR_send	knee_send_msg[10];
	//int is_motor_still[10] = {0};		//�жϵ���Ƿ�ֹ�ĺ���
	for(uint8_t i = 0; i < 10;i++)		//���Ƶ����ֹ
	{
		if(i < 6)
		{
			MotorStopPack(&knee_send_msg[i], B1, B1_Motor_ID[i]);
			
		}
		else
		{
			MotorStopPack(&knee_send_msg[i], A1, A1_Motor_ID[i]);
		}
	}
	for(uint8_t i = 0; i < 30; i++) //����ֹͣ�źţ�ʹ�ܵ�����������ԣ�����ֻ����һ�δ�����ղ�����Ϣ����������20����ѣ�У׼��ʱԼ60ms
	{	
	    //���͸�A1�����B1�����ID = 0���ܹ�4������
		memcpy(&SendToB1Motor_Buff[0], &knee_send_msg[0].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR1_RS485_SEND();
		memcpy(&SendToB1Motor_Buff[1], &knee_send_msg[3].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR2_RS485_SEND();
		memcpy(&SendToA1Motor_Buff[0], &knee_send_msg[6].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR3_RS485_SEND();
		memcpy(&SendToA1Motor_Buff[1], &knee_send_msg[8].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR4_RS485_SEND();
		delay_ms(2);
		//���͸�A1�����B1�����ID=1���ܹ�4������
		memcpy(&SendToB1Motor_Buff[0], &knee_send_msg[1].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR1_RS485_SEND();
		memcpy(&SendToB1Motor_Buff[1], &knee_send_msg[4].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR2_RS485_SEND();
		memcpy(&SendToA1Motor_Buff[0], &knee_send_msg[7].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR3_RS485_SEND();
		memcpy(&SendToA1Motor_Buff[1], &knee_send_msg[9].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR4_RS485_SEND();
		delay_ms(2);
//		//���͸�B1�����ID=2���ܹ�2������
		memcpy(&SendToB1Motor_Buff[0], &knee_send_msg[2].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR1_RS485_SEND();
		memcpy(&SendToB1Motor_Buff[1], &knee_send_msg[5].motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
		MOTOR2_RS485_SEND();
		delay_ms(2);
	}
	for(int8_t i = 0; i < 10; i++)
	{
		if(i < 6)
		{
			knee_motor->init_b1_pos[i] = knee_motor->b1_motor_recv[i].Pos;
		}
		else
		{
			knee_motor->init_a1_pos[i] = knee_motor->a1_motor_recv[i].Pos;
		}
	}
}

void Keep_B1_Motor_Still(void) //��ID=0��1��B1�����ֹ
{
	MOTOR_send motor_s;
	MotorSpeedPack(&motor_s, B1, 0, 3.0); //����ģʽ
	memcpy(&SendToB1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR1_RS485_SEND();
	MotorSpeedPack(&motor_s, B1, 0, 3.0); //����ģʽ
	memcpy(&SendToB1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR2_RS485_SEND();
	
	delay_ms(10);
	
	MotorSpeedPack(&motor_s, B1, 1, 3.0); //����ģʽ
	memcpy(&SendToB1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR1_RS485_SEND();
	MotorSpeedPack(&motor_s, B1, 1, 3.0); //����ģʽ
	memcpy(&SendToB1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR2_RS485_SEND();
	delay_ms(10);
}

void Keep_A1_Motor_Flexible(void) //��A1�������������˳Ӧģʽ
{
	MOTOR_send motor_s;
	MotorZeroTorquePack(&motor_s, A1, 0);
	memcpy(&SendToA1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR3_RS485_SEND();
	MotorZeroTorquePack(&motor_s, A1, 0);
	memcpy(&SendToA1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR4_RS485_SEND();
	
	delay_ms(10);
	
	MotorZeroTorquePack(&motor_s, A1, 1);
	memcpy(&SendToA1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR3_RS485_SEND();
	MotorZeroTorquePack(&motor_s, A1, 1);
	memcpy(&SendToA1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
	MOTOR4_RS485_SEND();
	delay_ms(10);
}

//����㶯���ƣ��ú���������ǰ�ڲ��Թؽڵ��ת��
void Motor_Jogging_Test(uint8_t board_num, uint8_t motor_num, float pos, float K_P, float K_W)
{
	MOTOR_send motor_s;
	unsigned short id = 0;
	enum MotorType motor_type = B1;
	if(board_num == 1) // 6��B1��4��A1
	{
		if(motor_num > 10 & motor_num < 1)
			return;
		else if(motor_num <= 6 & motor_num > 0)
		{
			switch(motor_num)
			{
				case 1:
					id = 0;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToB1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR1_RS485_SEND();
				break;
				case 2:
					id = 1;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToB1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR1_RS485_SEND();
				break;
				case 3:
					id = 2;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToB1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR1_RS485_SEND();
				break;
				case 4:
					id = 0;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToB1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR2_RS485_SEND();
				break;
				case 5:
					id = 1;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToB1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR2_RS485_SEND();
				break;
				case 6:
					id = 2;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToB1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR2_RS485_SEND();
				break;
			}
		}
		else
		{
			switch(motor_num)
			{
				case 7:
					id = 0;
					motor_type = A1;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToA1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR3_RS485_SEND();
				break;
				case 8:
					id = 1;
					motor_type = A1;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToA1Motor_Buff[0], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR3_RS485_SEND();
				break;
				case 9:
					id = 0;
					motor_type = A1;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToA1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR4_RS485_SEND();
				break;
				case 10:
					id = 1;
					motor_type = A1;
					MotorPosPack(&motor_s, motor_type, id, pos, K_P, K_W);
					memcpy(&SendToA1Motor_Buff[1], &motor_s.motor_send_data, SEND_TO_A1B1MOTOR_BUFF_SIZE);
					MOTOR4_RS485_SEND();
				break;
			}
		}
	}
}

