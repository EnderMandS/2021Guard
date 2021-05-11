#ifndef __CLASSIC_H__
#define __CLASSIC_H__

#include "main.h"
#include "pid.h"
#include <stdbool.h>

#define USE_SPRING		//非弹簧模式的功能已弃用，不能注释掉，留下只是为了保留代码
#define RAND_DIR

#define Classic_Slow		1500	//瞄准
#define Classic_Middle	4500	//巡检
#define Classic_Fast		8000	//闪避

#define Chassic_Spring_Slow		1500
#define Chassic_Spring_Middle	4500
#define Chassic_Spring_Fast		8000

#define Motor_Base 0x201
typedef enum
{
	Chassic_Left_3508_ID = 0x201,
	Chassic_Right_3508_ID = 0x202,
	Cartridge_2006_ID = 0x205,
}CAN_Message_ID;
typedef enum
{
	Chassic_L = Chassic_Left_3508_ID	-Motor_Base,
	Chassic_R = Chassic_Right_3508_ID	-Motor_Base,
	Cartridge = Cartridge_2006_ID			-Motor_Base,
}Motor_Data_ID;
enum
{
	Ready_Measure,
	Measuring,
	End_Measure
};

extern uint8_t Move_Allow;
extern int direction;
extern int eliminate_dithering_left;
extern int eliminate_dithering_right;
extern PID_TypeDef motor_pid[2];
extern int Classic_Move_Speed;
extern uint8_t Moto_ID[2];
extern uint8_t Aimming;
extern uint8_t Changing_Speed_Flag;
extern uint8_t Line;
extern int Last_Dir;
extern uint8_t Measuer_State;
extern uint32_t Rail_Len;

void Chassis_init(void);
float Slow_Change_Speed(int dir, uint16_t Speed);
void Spring(int dir,uint16_t Speed);
void Rand_Speed_Up_Init(void);
void Measuer_Rail_Len(void);

#endif
