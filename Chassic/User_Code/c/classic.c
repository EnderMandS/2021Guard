#include "classic.h"
#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "bsp_can.h"
#include "stm32f4xx_it.h"

int Buff_Time=200;

uint16_t Time_Cnt=0;
int Last_Dir=1;

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
//	switch(Speed)
//	{
//		case Classic_Slow:
//			Buff_Time=400;
//		break;
//		
//		case Classic_Middle:
//			Buff_Time=200;
//		break;
//		
//		case Classic_Fast:
//			Buff_Time=150;
//		break;
//		
//		default:
//			Buff_Time=150;
//		break;
//	}
	Buff_Time=100;
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

#define Wait_Cnt 400		//检测到电机反转之后等待时间 实际时间=Wait_Cnt/400Hz
bool Dir_Change_Wait=false;
int16_t Max_Speed=0;
uint32_t Dir_Change_Wait_Cnt=0;
void Spring(int dir,uint16_t Speed)
{
	if(dir!=Last_Dir)	//速度改变
	{
		if(Last_Dir==1)
		{
			if(gear_motor_data[Moto_ID[0]].speed_rpm>0)	//电机给0,等待弹簧反弹
			{
				for (uint8_t i=0; i<2; i++)
				{
					motor_pid[i].target=dir*Speed;
					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
			else
			{
				Dir_Change_Wait=true;
				Last_Dir=dir;
			}
			return;
		}
		else if(Last_Dir==-1)
		{
			if(gear_motor_data[Moto_ID[0]].speed_rpm<0)	//电机给0,等待弹簧反弹
			{
				for(uint8_t i=0; i<2; i++)
				{
					motor_pid[i].target=dir*Speed;
					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
			else
			{
				Dir_Change_Wait=true;
				Last_Dir=dir;
			}
			return;
		}
	}
	else
	{
		if(Dir_Change_Wait==true)	//等待弹簧弹性势能释放
		{
			++Dir_Change_Wait_Cnt;
			if(Dir_Change_Wait_Cnt>=Wait_Cnt)	//最多等待1s, 超时继续移动
			{
				Dir_Change_Wait_Cnt=0;
				Dir_Change_Wait=false;
			}
			if( abs(gear_motor_data[Moto_ID[0]].speed_rpm) > abs(Max_Speed) )
				Max_Speed=gear_motor_data[Moto_ID[0]].speed_rpm;
			else if( abs(gear_motor_data[Moto_ID[0]].speed_rpm) < abs(Max_Speed)-200 )	//弹性势能完全释放,速度开始下降,电机给电
			{
				Dir_Change_Wait_Cnt=0;
				Dir_Change_Wait=false;
			}
		}
		else	//正常移动PID计算
		{
			for (uint8_t i=0; i<2; i++)
			{
				motor_pid[i].target=dir*Speed;
				motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
				Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
			}
		}
	}
}

#define Sample_Times 6	//采样次数,实际使用5次数据,第一次数据不是在轨道边缘开始采样舍弃
uint8_t Measuer_State=Ready_Measure;	//采样状态
uint32_t Rail_Len=0;	//轨道长度
uint32_t Rail_Len_Buff[Sample_Times]={0};	//轨道长度缓冲区
uint16_t Rail_Len_Buff_cnt=0;	//采样次数计数值
void Measuer_Rail_Len(void)
{
	if(Measuer_State==End_Measure)	//完成测量,退出函数
		return;
	if(direction!=Last_Dir)	//速度反向,已经走完轨道一次
	{
		if(Measuer_State==Measuring)	//测量中
		{
			Rail_Len_Buff[Rail_Len_Buff_cnt]=abs(gear_motor_data[Moto_ID[0]].round_cnt);	//写入缓冲区
			++Rail_Len_Buff_cnt;
			if(Rail_Len_Buff_cnt!=Sample_Times)	//没有采样完成
			{
				Measuer_State=Ready_Measure;	//准备下次计数
			}
			else
			{
				uint32_t sum=0;	//采样完成,多次取平均
				for(uint8_t i=1; i<Sample_Times; ++i)	//第一次数据舍弃
					sum += Rail_Len_Buff[i];
				Rail_Len = sum/(Sample_Times-1);
				Measuer_State=End_Measure;	//置测量完成标志位
			}
		}
	}
	else	//没有触发弹簧,正常移动
	{
		if(Measuer_State==Ready_Measure)	//准备下一次测量,圈数清零
		{
			gear_motor_data[Moto_ID[0]].round_cnt=0;
			gear_motor_data[Moto_ID[1]].round_cnt=0;
			Measuer_State=Measuring;	//置测量中标志位
		}
	}
}
void Go_To_Middle(uint16_t Speed)	//回到轨道中间	0.4-0.6
{
	int dir=0;
	if(Rail_Position<0.4f)	//判断位置,确定移动方向
		dir=-1;
	else if(Rail_Position>0.6f)
		dir=1;
	for (uint8_t i=0; i<2; i++)	//pid计算
	{
		motor_pid[i].target=dir*Speed;
		motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
	}
	if(dir!=0)	//方向为0,不给输出
	{
		for (uint8_t i=0; i<2; i++)
			Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
	}
	else
	{
		for (uint8_t i=0; i<2; i++)
			Motor_Output[ Moto_ID[i] ]=0;
	}
}
