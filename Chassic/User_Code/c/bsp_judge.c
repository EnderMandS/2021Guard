#include "bsp_judge.h"
#include "judge.h"
#include "classic.h"
#include "shoot.h"
#include <stdio.h>
#include <stdlib.h>
#include "buzzer.h"

bool No_Bullet=false;

uint32_t Time_cnt=0;
uint8_t Being_Hit=0;
uint32_t SpeedUp_Time=0;
uint32_t Rand_Speed_Up_Time=0;

void Rand_Speed_Up_Init(void)
{
	Rand_Speed_Up_Time=(rand()%3+3)*400;	//400Hz
}

void Check_Being_Hit(void)	//受到伤害加速
{
	#ifndef USE_SPRING
		if(PowerHeatData.chassis_power_buffer>190 && Changing_Speed_Flag==0 && Hurt_Data_Update==false)		//随机加速
		{
			--Rand_Speed_Up_Time;
			if(Rand_Speed_Up_Time==0)
				Hurt_Data_Update=true;
		}

		if(Hurt_Data_Update==true)
		{
			if(Being_Hit==0)
			{
				Being_Hit=1;
				SpeedUp_Time=(rand()%3+1)*200;
				if(PowerHeatData.chassis_power_buffer>75)
				{
					if(Classic_Move_Speed==Classic_Middle)
					{
						Classic_Move_Speed=Classic_Fast;
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
				if(Classic_Move_Speed==Classic_Fast)
					Classic_Move_Speed=Classic_Middle;	//恢复巡检速度
			}
		}
	#else
//		if(PowerHeatData.chassis_power_buffer>190 && Changing_Speed_Flag==0 && Hurt_Data_Update==false)		//随机加速
//		{
//			--Rand_Speed_Up_Time;
//			if(Rand_Speed_Up_Time==0)
//				Hurt_Data_Update=true;
//		}

		if(Hurt_Data_Update==true)
		{
			if(Being_Hit==0)
			{
				Being_Hit=1;
				SpeedUp_Time=(rand()%2+2)*200;
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
	#endif
}

bool Rand_Change_Flag=false;
void Rand_Dir_Change(void)
{
	if(	PowerHeatData.chassis_power_buffer>150 &&	//缓冲能量判断
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
			if( (is_red_or_blue()==BLUE && Robot_Interactive.Receive_ID==107) ||
					(is_red_or_blue()==RED  && Robot_Interactive.Receive_ID==7))
			{
				if(Inspect_Position==0 && Robot_Interactive.Data[0]<=4)
				{
					Inspect_Position=Robot_Interactive.Data[0];
					Buzzer_Short(1);
				}
			}
		break;
		
		default:
			break;
	}
}
