#include "bsp_judge.h"
#include "judge.h"
#include "classic.h"
#include "shoot.h"
#include <stdio.h>
#include <stdlib.h>
#include "buzzer.h"
#include "bsp_can.h"
#include "can.h"

uint32_t Time_cnt=0;
uint8_t Being_Hit=0;
uint32_t SpeedUp_Time=0;
uint32_t Hit_Random_CD_cnt=0;		//Being hit random change direction CD
uint32_t Rand_cnt=800;
uint8_t Gimbal_Position=0;

float Random(float Min, float Max)
{
	if(Max<=Min)
		return 0;
	return ((rand()%100+1)/100.f)*(Max-Min)+Min;
}

void Rand_Speed_Up_Init(void)
{
	SpeedUp_Time = (uint32_t)((( (rand()%100+1.f)/100.f)+0.5f )*400);	//400Hz
}

#define Hit_Random_CD 400
void Check_Being_Hit(void)	//受到伤害加速
{
	if(Hit_Random_CD_cnt>0)
		--Hit_Random_CD_cnt;
	if(Hurt_Data_Update==true)
	{
		if(	PowerHeatData.chassis_power_buffer>30 &&	//缓冲能量判断
				Changing_Speed_Flag==0 &&		//边缘判断
				Measuer_State==End_Measure &&
				(Rail_Position>0.2f && Rail_Position<0.8f) &&	//rail position
				Classic_Move_Speed==Chassic_Spring_Middle &&	//被击打加速判断
				Hit_Random_CD_cnt==0 &&
				Being_Hit==0 &&
				rand()%2==0)	//50%概率 变向
		{
			Rand_cnt=(uint32_t)(Random(0.5,3)*400);
			Hit_Random_CD_cnt=Hit_Random_CD;
			direction=-direction;
			Last_Dir=direction;
			Buzzer_Short(1);
			Hurt_Data_Update=false;
			return;
		}
		if(Being_Hit==0)	//加速
		{
			Hit_Random_CD_cnt=Hit_Random_CD;
			Being_Hit=1;
			Rand_Speed_Up_Init();
			if(PowerHeatData.chassis_power_buffer>75)
			{
				if(Classic_Move_Speed==Chassic_Spring_Middle)
				{
					Classic_Move_Speed=Chassic_Spring_Fast;
					Buzzer_Short(1);
				}
			}
		}
	}
	if(Being_Hit==1)
	{
		++Time_cnt;
		if(Time_cnt>SpeedUp_Time)
		{
			Rand_Speed_Up_Init();
			Hurt_Data_Update=false;
			Being_Hit=Time_cnt=0;
			if(Classic_Move_Speed==Chassic_Spring_Fast)
				Classic_Move_Speed=Chassic_Spring_Middle;	//恢复巡检速度
		}
	}
}

bool Rand_Change_Flag=false;
void Rand_Dir_Change(void)
{
	if(	PowerHeatData.chassis_power_buffer>30 &&	//缓冲能量判断
			Changing_Speed_Flag==0 &&		//边缘判断
			Classic_Move_Speed==Chassic_Spring_Middle &&	//被击打加速判断
			rand()%4==0)	//25%概率
	{
		direction=-direction;
		Last_Dir=direction;
		Buzzer_Short(1);
	}
}

Field_Event Field_Event_Data={false,false,false,false,false,false,false,false,false,true,true};
void Event_Decode(uint32_t Event)
{
	uint32_t Num=0x00000001;
	
	if( (Event&Num)==1 )	//bit0
		Field_Event_Data.Supply_Health_Occupy_1=true;
	else
		Field_Event_Data.Supply_Health_Occupy_1=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit1
		Field_Event_Data.Supply_Health_Occupy_2=true;
	else
		Field_Event_Data.Supply_Health_Occupy_2=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit2
		Field_Event_Data.Supply_Health_Occupy_3=true;
	else
		Field_Event_Data.Supply_Health_Occupy_3=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit3
		Field_Event_Data.Hit_Area_Occupy=true;
	else
		Field_Event_Data.Hit_Area_Occupy=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit4
		Field_Event_Data.Small_Buff_Active=true;
	else
		Field_Event_Data.Small_Buff_Active=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit5
		Field_Event_Data.Big_Buff_Active=true;
	else
		Field_Event_Data.Big_Buff_Active=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit6
		Field_Event_Data.R2_Occupy=true;
	else
		Field_Event_Data.R2_Occupy=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit7
		Field_Event_Data.R3_Occupy=true;
	else
		Field_Event_Data.R3_Occupy=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit8
		Field_Event_Data.R4_Occupy=true;
	else
		Field_Event_Data.R4_Occupy=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit9
		Field_Event_Data.Base_Shield_Existence=true;
	else
		Field_Event_Data.Base_Shield_Existence=false;
	Event >>= 1;
	
	if( (Event&Num)==1 )	//bit10
		Field_Event_Data.Outpost_Alive=true;
	else
		Field_Event_Data.Outpost_Alive=false;
}

uint8_t Inspect_Position=0;	//receive data from other robot
														//send to gimbal where to inspect
														//clear to 0 when gimbal control inspect not busy
void Receive_Robot_Interactive(void)
{
	switch (Robot_Interactive.data_ID)
	{
		case 0x0222:
		{
			if( (is_red_or_blue()==BLUE && Robot_Interactive.Receive_ID==107) ||
					(is_red_or_blue()==RED  && Robot_Interactive.Receive_ID==7))
			{
				if(Robot_Interactive.Data[0]<=4 && Robot_Interactive.Data[0]>=1)
				{
					Inspect_Position=Robot_Interactive.Data[0];
					Buzzer_Short(1);
				}
			}
		}
		break;
		
		default:
			break;
	}
}


uint8_t Make_Position=0;
void Make_Receive_Robot(void)
{
	if(Inspect_Position==0 && Make_Position>0 && Make_Position<=4)
	{
		Inspect_Position=Make_Position;
		Make_Position=0;
		Buzzer_Short(1);
	}
}

bool Game_Start=false;
void Set_Game_Start(void)
{
	if(Game_Start)
		GameState.game_progress=4;
	else
		GameState.game_progress=0;
}

bool Set_Outpost=true;
void Set_Outpost_Alive(void)
{
	Field_Event_Data.Outpost_Alive=Set_Outpost;
}

bool Set_Base_Shield=true;
void Set_Base_Shield_Existence(void)
{
	Field_Event_Data.Base_Shield_Existence=Set_Base_Shield;
}

void Rand_Dir_Time(void)
{
	if(Rail_Position>0.2f && Rail_Position<0.8f)
	{
		--Rand_cnt;
		if(Rand_cnt==0 &&
			 Changing_Speed_Flag==0 &&
			 Classic_Move_Speed==Chassic_Spring_Middle &&
			 PowerHeatData.chassis_power_buffer>30)
		{
			direction=-direction;
			Last_Dir=direction;
			Buzzer_Short(1);
		}
		if(Rand_cnt==0)
			Rand_cnt=(uint32_t)(Random(0.5,3)*400);
	}
}
