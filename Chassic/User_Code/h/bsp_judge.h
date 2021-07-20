#ifndef __BSP_JUDGE_H__
#define __BSP_JUDGE_H__

#include "main.h"
#include <stdbool.h>

typedef struct
{
	bool Supply_Health_Occupy_1;	//己方补给站 1号补血点占领状态
	bool Supply_Health_Occupy_2;	//己方补给站 2号补血点占领状态
	bool Supply_Health_Occupy_3;	//己方补给站 3号补血点占领状态
	bool Hit_Area_Occupy;	//打击点占领状
	bool Small_Buff_Active;	//小能量机关激活状态
	bool Big_Buff_Active;	//大能量机关激活状态
	bool R2_Occupy;	//己方R2环形高地占领状态
	bool R3_Occupy;	//己方R3环形高地占领状态
	bool R4_Occupy;	//己方R4环形高地占领状态
	bool Base_Shield_Existence;	//己方基地护盾状态
	bool Outpost_Alive;	//己方前哨战存活状态
}Field_Event;

extern bool Rand_Change_Flag;
extern Field_Event Field_Event_Data;
extern uint8_t Inspect_Position;
extern uint8_t Gimbal_Position;
extern float Target_Angle;

void Check_Being_Hit(void);
void Event_Decode(uint32_t Event);
void Receive_Robot_Interactive(void);
void Set_Game_Start(void);
void Set_Outpost_Alive(void);
void Set_Base_Shield_Existence(void);
float Random(float Min, float Max);
void Rand_Dir_Time(void);
void Robot_Command_Receive(void);

#endif
