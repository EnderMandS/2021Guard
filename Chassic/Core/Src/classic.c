#include "classic.h"
#include "pid.h"
#include "math.h"

uint16_t Time_Cnt=0;
int Last_Dir=0;

float set_spd[2];
PID_TypeDef motor_pid[2];
uint8_t Moto_ID[2]={Chassic_L,Chassic_R};
uint8_t Move_Allow=0;
int direction=1;
int eliminate_dithering_left = 0;
int eliminate_dithering_right = 0;
int Classic_Move_Speed=Classic_Slow;
uint8_t Aimming=0;
int Chassic_Speed_Buf=0;

void Chassis_init(void)
{
	for(int j=0;j<2;j++)
	{
		pid_init(&motor_pid[j]);
		motor_pid[j].f_param_init(&motor_pid[j],PID_Speed,16384,5000,10,0,6000,0,1.5,0.1,0);  //0.004
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
			Time_Cnt=0;
		}
	}
	return dir*Speed;
}
