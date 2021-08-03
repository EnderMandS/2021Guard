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

uint32_t Time_cnt=0;	//��������ټ���
uint8_t Being_Hit=0;	//������״̬������
uint32_t SpeedUp_Time=0;	//����ʱ��
uint32_t Hit_Random_CD_cnt=0;		//Being hit random change direction CD
uint32_t Rand_cnt=800;	//�������ʱ������
uint8_t Gimbal_Position=0;	//��̨Ѳ��λ��

float Random(float Min, float Max)	//����һ�������,����������С��Χ
{
	if(Max<=Min)
		return 0;
	return ((rand()%100+1)/100.f)*(Max-Min)+Min;
}

void Rand_Speed_Up_Init(void)	//���������ʱ����ʼ��
{
	SpeedUp_Time = (uint32_t)((( (rand()%100+1.f)/100.f)+1.f )*400);	//400Hz
}

#define Hit_Random_CD 400	//����������ȴʱ�� 400Hz 1s
void Check_Being_Hit(void)	//�ܵ��˺�����
{
	if(Hit_Random_CD_cnt>0)	//��ȴʱ���Լ�
		--Hit_Random_CD_cnt;
	if(Hurt_Data_Update==true)	//����ϵͳ����,�˺����ݸ���
	{
		if(	PowerHeatData.chassis_power_buffer>30 &&	//���������ж�
				Changing_Speed_Flag==0 &&		//�Ƿ��ڱ����ж�
				Measuer_State==End_Measure &&	//Got rail len
				(Rail_Position>0.2f && Rail_Position<0.8f) &&	//rail position ��Ե�ж�
				Classic_Move_Speed==Chassic_Spring_Middle &&	//�Ƿ�������Ѳ��״̬
				Hit_Random_CD_cnt==0 &&	//������ȴʱ����
				Being_Hit==0 &&	//not at speed up state
				rand()%2==0)	//50%���� ����
		{
			Rand_cnt=(uint32_t)(Random(0.5,2)*400);	//new a rand change direction time	//0.5  2.5
			Hit_Random_CD_cnt=Hit_Random_CD;	//������ȴ
			direction=-direction;	//�ٶȷ���
			Last_Dir=direction;
			Buzzer_Short(1);
			Hurt_Data_Update=false;	//clean flag
			return;	//��ִ�б���,�˳�����
		}
		if(Being_Hit==0)	//����
		{
			Hit_Random_CD_cnt=Hit_Random_CD;
			Being_Hit=1;	//״̬��������һ��״̬
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
		++Time_cnt;	//����ʱ���������
		if(Time_cnt>SpeedUp_Time)	//�������
		{
			Hurt_Data_Update=false;	//���������ձ�־λ
			Being_Hit=Time_cnt=0;
			if(Classic_Move_Speed==Chassic_Spring_Fast)
				Classic_Move_Speed=Chassic_Spring_Middle;	//�ָ�Ѳ���ٶ�
		}
	}
}

Field_Event Field_Event_Data={false,false,false,false,false,false,false,false,false,true,true};	
void Event_Decode(uint32_t Event)	//�����¼�����,����ϵͳ����
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
void Receive_Robot_Interactive(void)	//û�õ�
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

bool Game_Start=false;	//������,���ñ�����ʼ״̬
void Set_Game_Start(void)
{
	if(Game_Start)
		GameState.game_progress=4;
	else
		GameState.game_progress=0;
}

bool Set_Outpost=true;	//������,����ǰ��վ״̬
void Set_Outpost_Alive(void)
{
	Field_Event_Data.Outpost_Alive=Set_Outpost;
}

bool Set_Base_Shield=true;	//������,���û��ػ���״̬
void Set_Base_Shield_Existence(void)
{
	Field_Event_Data.Base_Shield_Existence=Set_Base_Shield;
}

void Rand_Dir_Time(void)	//�������
{
	if(Rail_Position>0.2f && Rail_Position<0.8f)	//���ڹ����Ե
	{
		--Rand_cnt;	//���򵹼���
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

float Target_Angle=0;	//������̨��Ѳ��Ƕ�
#define Red_Guard_Position_X 5.42f	//С��ͼ�Ϻ췽�ڱ�λ��
#define Red_Guard_Position_Y 8.44f
#define Blue_Guard_Position_X 22.71f	//�����ڱ�λ��
#define Blue_Guard_Position_Y 8.26f
void Robot_Command_Receive(void)	//�ڲ���ϵͳ���պ����е���
{
	uint8_t Inspect_Position_temp=0;
	//Quadrant
	if(is_red_or_blue()==RED)	//�췽���������������
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
	else	//������������
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
	switch(Inspect_Position_temp)	//����Ƕ�
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
