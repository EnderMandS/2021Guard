#include "bsp_judge.h"
#include "judge.h"
#include "classic.h"
#include "shoot.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "buzzer.h"
#include "bsp_can.h"
#include "can.h"

uint32_t Time_cnt=0;	//被击打加速计数
uint8_t Being_Hit=0;	//被击倒状态机变量
uint32_t SpeedUp_Time=0;	//加速时长
uint32_t Hit_Random_CD_cnt=0;		//Being hit random change direction CD
uint32_t Rand_cnt=800;	//随机变向时长变量
uint8_t Gimbal_Position=0;	//云台巡检位置

float Random(float Min, float Max)	//产生一个随机数,输入最大和最小范围
{
	if(Max<=Min)
		return 0;
	return ((rand()%100+1)/100.f)*(Max-Min)+Min;
}

void Rand_Speed_Up_Init(void)	//被击打加速时长初始化
{
	SpeedUp_Time = (uint32_t)((( (rand()%100+1.f)/100.f)+1.f )*400);	//400Hz
}

#define Hit_Random_CD 400	//被击打检测冷却时间 400Hz 1s
void Check_Being_Hit(void)	//受到伤害加速
{
	if(Hit_Random_CD_cnt>0)	//冷却时间自减
		--Hit_Random_CD_cnt;
	if(Hurt_Data_Update==true)	//裁判系统读回,伤害数据更新
	{
		if(	PowerHeatData.chassis_power_buffer>30 &&	//缓冲能量判断
				Changing_Speed_Flag==0 &&		//是否在变速判断
				Measuer_State==End_Measure &&	//Got rail len
				(Rail_Position>0.2f && Rail_Position<0.8f) &&	//rail position 边缘判断
				Classic_Move_Speed==Chassic_Spring_Middle &&	//是否在正常巡检状态
				Hit_Random_CD_cnt==0 &&	//不在冷却时间内
				Being_Hit==0 &&	//not at speed up state
				rand()%2==0)	//50%概率 变向
		{
			Rand_cnt=(uint32_t)(Random(0.5,2)*400);	//new a rand change direction time	//0.5  2.5
			Hit_Random_CD_cnt=Hit_Random_CD;	//进入冷却
			direction=-direction;	//速度反向
			Last_Dir=direction;
			Buzzer_Short(1);
			Hurt_Data_Update=false;	//clean flag
			return;	//已执行变向,退出函数
		}
		if(Being_Hit==0)	//加速
		{
			Hit_Random_CD_cnt=Hit_Random_CD;
			Being_Hit=1;	//状态机进入下一个状态
			if(PowerHeatData.chassis_power_buffer>75)	//enough power
			{
				if(Classic_Move_Speed==Chassic_Spring_Middle)	//in normal speed, not at aim and speed up 
				{
					Rand_Speed_Up_Init();	//rand speed up time
					Classic_Move_Speed=Chassic_Spring_Fast;
					Buzzer_Short(1);
				}
			}
		}
	}
	if(Being_Hit==1)
	{
		++Time_cnt;	//加速时间计数自增
		if(Time_cnt>SpeedUp_Time)	//加速完成
		{
			Hurt_Data_Update=false;	//加速完成清空标志位
			Being_Hit=Time_cnt=0;
			if(Classic_Move_Speed==Chassic_Spring_Fast)
				Classic_Move_Speed=Chassic_Spring_Middle;	//恢复巡检速度
		}
	}
}

Field_Event Field_Event_Data={false,false,false,false,false,false,false,false,false,true,true};	
void Event_Decode(uint32_t Event)	//场地事件更新,裁判系统读回
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
void Receive_Robot_Interactive(void)	//没用到
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

bool Game_Start=false;	//调试用,设置比赛开始状态
void Set_Game_Start(void)
{
	if(Game_Start)
		GameState.game_progress=4;
	else
		GameState.game_progress=0;
}

bool Set_Outpost=true;	//调试用,设置前哨站状态
void Set_Outpost_Alive(void)
{
	Field_Event_Data.Outpost_Alive=Set_Outpost;
}

bool Set_Base_Shield=true;	//调试用,设置基地护盾状态
void Set_Base_Shield_Existence(void)
{
	Field_Event_Data.Base_Shield_Existence=Set_Base_Shield;
}

void Rand_Dir_Time(void)	//随机变向
{
	if(Rail_Position>0.2f && Rail_Position<0.8f)	//不在轨道边缘
	{
		--Rand_cnt;	//变向倒计数
		if(Rand_cnt==0 &&	//time cnt complete
			 Changing_Speed_Flag==0 &&	//not changing speed
			 Classic_Move_Speed==Chassic_Spring_Middle &&	//in normal speed , not speed up and aim
			 PowerHeatData.chassis_power_buffer>30)	//enough power
		{
			direction=-direction;	//change speed direction
			Last_Dir=direction;
			Buzzer_Short(1);
		}
		if(Rand_cnt==0)
			Rand_cnt=(uint32_t)(Random(0.5,2)*400);	//new a rand number
	}
}

float Target_Angle=0;	//发给云台的巡检角度
#define Red_Guard_Position_X 5.42f	//小地图上红方哨兵位置
#define Red_Guard_Position_Y 8.44f
#define Blue_Guard_Position_X 22.71f	//蓝方哨兵位置
#define Blue_Guard_Position_Y 8.26f
void Robot_Command_Receive(void)	//在裁判系统接收函数中调用
{
	uint8_t Inspect_Position_temp=0;
	//Quadrant
	if(is_red_or_blue()==RED)	//红方根据坐标解算象限
	{
		if(Robot_Command.target_position_x>Red_Guard_Position_X && Robot_Command.target_position_y<=Red_Guard_Position_Y)
			Inspect_Position_temp=1;
		else if(Robot_Command.target_position_x>Red_Guard_Position_X && Robot_Command.target_position_y>Red_Guard_Position_Y)
			Inspect_Position_temp=2;
		else if(Robot_Command.target_position_x<=Red_Guard_Position_X && Robot_Command.target_position_y>Red_Guard_Position_Y)
			Inspect_Position_temp=3;
		else if(Robot_Command.target_position_x<=Red_Guard_Position_X && Robot_Command.target_position_y<=Red_Guard_Position_Y)
			Inspect_Position_temp=4;
	}
	else	//蓝方解算象限
	{
		if(Robot_Command.target_position_x<=Blue_Guard_Position_X && Robot_Command.target_position_y>Blue_Guard_Position_Y)
			Inspect_Position_temp=1;
		else if(Robot_Command.target_position_x<=Blue_Guard_Position_X && Robot_Command.target_position_y<=Blue_Guard_Position_Y)
			Inspect_Position_temp=2;
		else if(Robot_Command.target_position_x>Blue_Guard_Position_X && Robot_Command.target_position_y<=Blue_Guard_Position_Y)
			Inspect_Position_temp=3;
		else if(Robot_Command.target_position_x>Blue_Guard_Position_X && Robot_Command.target_position_y>Blue_Guard_Position_Y)
			Inspect_Position_temp=4;
	}
	
	//Angle
	if(is_red_or_blue()==RED)	//arctan dead area
	{
		if(Robot_Command.target_position_y==Red_Guard_Position_Y)
		{
			if(Robot_Command.target_position_x>Red_Guard_Position_X)
				Target_Angle=90;
			else
				Target_Angle=270;
			return;
		}
	}
	else
	{
		if(Robot_Command.target_position_y==Blue_Guard_Position_Y)
		{
			if(Robot_Command.target_position_x<Blue_Guard_Position_X)
				Target_Angle=90;
			else
				Target_Angle=270;
			return;
		}
	}
	
	float dif_x=0;
	float dif_y=0;
	if(is_red_or_blue()==RED)	//dx,dy
	{
		dif_x=Robot_Command.target_position_x-Red_Guard_Position_X;
		dif_y=Robot_Command.target_position_y-Red_Guard_Position_Y;
	}
	else
	{
		dif_x=Robot_Command.target_position_x-Blue_Guard_Position_X;
		dif_y=Robot_Command.target_position_y-Blue_Guard_Position_Y;
	}
	switch(Inspect_Position_temp)	//算出角度
	{
		case 1:
			Target_Angle=atan(-dif_x/dif_y)/(PI/2.f)*90;
		break;
		
		case 2:
			Target_Angle=atan(dif_y/dif_x)/(PI/2.f)*90+90;
		break;
		
		case 3:
			Target_Angle=atan(dif_x/-dif_y)/(PI/2.f)*90+180;
		break;
		
		case 4:
			Target_Angle=atan(dif_y/dif_x)/(PI/2.f)*90+270;
		break;
		
		default:
			Target_Angle=45.f;
		break;
	}
	Inspect_Position=Inspect_Position_temp;
}

uint32_t Rand_Hit_cnt=400;
void Rand_Hit_Creat(void)	//400Hz
{
	if(Rand_Hit_cnt!=0)
		
		--Rand_Hit_cnt;
	else
	{
		Rand_Hit_cnt=(uint32_t)(Random(2,4)*400);
		Hurt_Data_Update=true;
	}
}
