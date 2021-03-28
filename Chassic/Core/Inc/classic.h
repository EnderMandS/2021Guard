#ifndef __CLASSIC_H__
#define __CLASSIC_H__

#include "main.h"
#include "pid.h"

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

extern uint8_t Move_Allow;
extern int direction;
extern int eliminate_dithering_left;
extern int eliminate_dithering_right;
extern PID_TypeDef motor_pid[2];
extern int Classic_Move_Speed;
extern uint8_t Moto_ID[2];

void Chassis_init(void);
float Slow_Change_Speed(int dir, uint16_t Speed);

#endif
