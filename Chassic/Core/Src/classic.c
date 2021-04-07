#include "classic.h"
#include "pid.h"
#include "math.h"
#include "bsp_can.h"

int Buff_Time=200;

uint16_t Time_Cnt=0;
int Last_Dir=0;

float set_spd[2];
PID_TypeDef motor_pid[2];
uint8_t Moto_ID[2]={Chassic_L,Chassic_R};
uint8_t Move_Allow=0;
int direction=1;
uint8_t Line=1;
int eliminate_dithering_left = 0;
int eliminate_dithering_right = 0;
int Classic_Move_Speed=Classic_Slow;
uint8_t Aimming=0;
uint8_t Changing_Speed_Flag=0;	//改变速度方向时置1，由变速函数置0		光电开关和闪避检测置1

void Chassis_init(void)
{
	for(int j=0;j<2;j++)
	{
		pid_init(&motor_pid[j]);
		motor_pid[j].f_param_init(&motor_pid[j],PID_Speed,5000,2500,10,0,6000,0,1.5,0.1,0);  //0.004
/*				   PID_ID id,uint16_t maxOutput,uint16_t integralLimit,float deadband,uint16_t controlPeriod,int16_t max_err,     
			int16_t  target,
			 float kp,
			 float ki,
			 float kd);*/
	}
    set_spd[0] = 0;
    set_spd[1] = 0;
}

float Slow_Change_Speed(int dir, uint16_t Speed)
{
	switch(Speed)
	{
		case Classic_Slow:
			Buff_Time=400;
		break;
		
		case Classic_Middle:
			Buff_Time=200;
		break;
		
		case Classic_Fast:
			Buff_Time=150;
		break;
		
		default:
			Buff_Time=150;
		break;
	}
	if(dir!=Last_Dir)
	{
		++Time_Cnt;
		if(Time_Cnt<Buff_Time)
		{
			if(Last_Dir==0)
				return dir*sin( (Time_Cnt*1.0)/Buff_Time*PI*0.5 )*Speed;
			else
				return Last_Dir*cos( (Time_Cnt*1.0)/Buff_Time*PI )*Speed;
		}
		else
		{
			Last_Dir=dir;
			Changing_Speed_Flag=Time_Cnt=0;
		}
	}
	return dir*Speed;
}

void Spring(int dir,uint16_t Speed)
{
	if(Speed==Chassic_Spring_Slow)	//瞄准到不反弹
	{
		motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(dir,Speed);
		for (uint8_t i=0; i<2; i++)
		{
			motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
			Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
		}
		return;
	}
	else if(dir!=Last_Dir)
//	if(dir!=Last_Dir)
	{
		if(Last_Dir==1)
		{
			if(gear_motor_data[Moto_ID[0]].speed_rpm>0)
			{
				for (uint8_t i=0; i<2; i++)
				{
//					motor_pid[i].target=0;
//					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
			else
				Last_Dir=dir;
			return;
		}
		else if(Last_Dir==-1)
		{
			if(gear_motor_data[Moto_ID[0]].speed_rpm<0)
			{
				for(uint8_t i=0; i<2; i++)
				{
//					motor_pid[i].target=0;
//					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
			else
				Last_Dir=dir;
			return;
		}
	}
	else
	{
		for (uint8_t i=0; i<2; i++)
		{
			motor_pid[i].target=dir*Speed;
			motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
			Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
		}
	}
}
