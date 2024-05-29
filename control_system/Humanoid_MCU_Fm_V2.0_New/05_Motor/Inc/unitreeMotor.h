#ifndef UNITREEMOTOR_H
#define UNITREEMOTOR_H

#include "motor_msg.h" //电机通讯协议
#include "global_Para.h"
#include "stm32f4xx.h"
//#include "usart.h"
#include <stdbool.h>
#include "crc32.h"
/*------------------------------------------一些使用的函数---------------------------------------------------*/
			
/*-------------------------------------A1B1电机数据结构体-START----------------------------------------------*/
enum MotorType{
	A1, //4.8M baudrate, K_W X 1024
	B1 //6M	baudrate, K_W X 512
}; // 电机类型

enum MotorMode{
	BRAKE,
	FOC,
	CALIBRATE
};

typedef struct MOTOR_send
{
	// 定义 发送格式化数据
    MasterComdDataV3  motor_send_data;  //电机控制数据结构体，详见motor_msg.h
    enum MotorType motortype;		//电机类型：A1 or B1
	int hex_len;                   //发送的16进制命令数组长度, 34
    // long long send_time;            //发送该命令的时间, 微秒(us)
    // 待发送的各项数据
    unsigned short id;              //电机ID，0xBB代表全部电机
    unsigned short mode;            //0:空闲, 5:开环转动, 10:闭环FOC控制
    //实际给FOC的指令力矩为：
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                        //期望关节速度（电机本身的速度）(rad/s)
    float Pos;                      //期望关节位置（rad）
    float K_P;                      //关节刚度系数
    float K_W;                      //关节速度系数
    COMData32 Res;                  // 通讯 保留字节  用于实现别的一些通讯内容
} MOTOR_send;

typedef struct MOTOR_recv{
    // 定义 接收数据
    ServoComdDataV3 motor_recv_data;     //电机接收数据结构体，详见motor_msg.h
    enum MotorType motorType;
    int hex_len;                    //接收的16进制命令数组长度, 78
    // long long resv_time;            //接收该命令的时间, 微秒(us)
    bool correct;                   //接收数据是否完整（true完整，false不完整）
    //解读得出的电机数据
    unsigned char motor_id;         //电机ID
    unsigned char mode;             //0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;                       //温度
    unsigned char MError;           //错误码

    float T;                        // 当前实际电机输出力矩
    float W;                        // 当前实际电机速度（高速）
    float LW;                       // 当前实际电机速度（低速）
    int Acc;                      // 电机转子加速度
    float Pos;                      // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

    float gyro[3];                  // 电机驱动板6轴传感器数据
    float acc[3];
} MOTOR_recv;

typedef struct UnitreeA1B1{
	MOTOR_recv a1_motor_recv[4];
	MOTOR_recv b1_motor_recv[6];
	float init_b1_pos[6]; //六个B1电机
	float init_a1_pos[4]; //四个A1电机
} UnitreeA1B1;
/*-------------------------------------A1B1电机数据结构体-END-----------------------------------------------*/

/*-------------------------------------A1B1电机数据传输函数-------------------------------------------------*/
extern UnitreeA1B1 unitree_a1b1_motors;
extern unsigned short B1_Motor_ID[6];
extern unsigned short A1_Motor_ID[4];

void A1B1RS485Receive(uint8_t rs485_id);
void modify_data(MOTOR_send* motor_s);
bool extract_data(MOTOR_recv* motor_r);

void MotorTorquePack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id, float T, float W, float Pos, float K_P, float K_W);
void MotorStopPack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id);
void MotorPosPack(MOTOR_send* motor_s, enum MotorType motorType, unsigned short id, float Pos, float K_P, float K_W);
void MotorSpeedPack(MOTOR_send* motor_s, enum MotorType motortype, unsigned short id, float K_W);
void MotorZeroTorquePack(MOTOR_send* motor_s, enum MotorType motortype, unsigned short id);
#endif  // UNITREEMOTOR_H
