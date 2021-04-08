#include "bsp_judge.h"
#include "judge.h"
#include "classic.h"
#include "shoot.h"
#include <stdio.h>
#include <stdlib.h>

int Cartridge_Speed_Offset=0;
bool No_Bullet=false;

uint32_t Time_cnt=0;
uint8_t Being_Hit=0;
uint32_t SpeedUp_Time=0;

void Check_Being_Hit(void)	//受到伤害加速	200Hz
{
	#ifndef USE_SPRING
	if(Hurt_Data_Update==true)
	{
		if(Being_Hit==0)
		{
			Being_Hit=1;
//			SpeedUp_Time=(rand()%3+1)*200;	//1-3s 200Hz
			SpeedUp_Time=3*200;
			if(PowerHeatData.chassis_power_buffer>75)
			{
				if(Classic_Move_Speed==Classic_Middle)
					Classic_Move_Speed=Classic_Fast;
			}
		}
	}
	if(Being_Hit==1)
	{
		++Time_cnt;
		if(Time_cnt>SpeedUp_Time)
		{
			Hurt_Data_Update=false;
			Being_Hit=Time_cnt=0;
			if(Classic_Move_Speed==Classic_Fast)
				Classic_Move_Speed=Classic_Middle;	//恢复巡检速度
		}
	}
	#else
	if(Hurt_Data_Update==true)
	{
		if(Being_Hit==0)
		{
			Being_Hit=1;
//			SpeedUp_Time=(rand()%3+1)*200;	//1-3s 200Hz
			SpeedUp_Time=3*200;		//3秒
			if(PowerHeatData.chassis_power_buffer>75)
			{
				if(Classic_Move_Speed==Chassic_Spring_Middle)
					Classic_Move_Speed=Chassic_Spring_Fast;
			}
		}
	}
	if(Being_Hit==1)
	{
		++Time_cnt;
		if(Time_cnt>SpeedUp_Time)
		{
			Hurt_Data_Update=false;
			Being_Hit=Time_cnt=0;
			if(Classic_Move_Speed==Chassic_Spring_Fast)
				Classic_Move_Speed=Chassic_Spring_Middle;	//恢复巡检速度
		}
	}
	#endif
}

void Power_Heat_Cheak(void)
{
	if(Power_Heat_Data_Updata==true)
	{
		Power_Heat_Data_Updata=false;
		if(PowerHeatData.shooter_id1_17mm_cooling_heat>=320)	//热量上限320J
		Cartridge_Speed_Offset-=500;
	}
}

void Empty_Bullet(void)	//in 1Hz interrupt 判断是否有弹射出，连续10秒没有发射加速运动
{
	static int cnt=10;
	
	if(GameState.game_progress==4)	//4对战中
	{
		if(Shoot_Update==true)
		{
			Shoot_Update=false;
			cnt=10;
			No_Bullet=false;
		}
		else if(Cartridge_wheel.target!=0)
		{
			if(cnt==0)
			{
				No_Bullet=true;
				Classic_Move_Speed=Classic_Fast;
			}
			else
				--cnt;
		}
	}
}
