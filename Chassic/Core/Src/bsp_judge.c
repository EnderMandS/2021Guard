#include "bsp_judge.h"
#include "judge.h"
#include "classic.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

int Chassic_Speed_Offset=0;
int Cartridge_Speed_Offset=0;

void Check_Being_Hit(void)	//受到伤害加速
{
	static uint32_t Time_cnt=0;
	static uint8_t Being_Hit=0;
	static uint32_t SpeedUp_Time=0;
	
	if(Hurt_Data_Update==true)
	{
		Hurt_Data_Update=false;
		if(Being_Hit==0)
		{
			Being_Hit=1;
			SpeedUp_Time=(rand()%3+1)*200;	//1-3s 200Hz
			if(Classic_Move_Speed==Classic_Middle)
				Classic_Move_Speed=Classic_Fast;
		}
	}
	if(Being_Hit==1)
	{
		++Time_cnt;
		if(Time_cnt>SpeedUp_Time)
		{
			Being_Hit=Time_cnt=0;
			if(Classic_Move_Speed==Classic_Fast)
				Classic_Move_Speed=Classic_Middle;	//恢复巡检速度
		}
	}
}

void Power_Heat_Cheak(void)
{
	if(Power_Heat_Data_Updata==true)
	{
		Power_Heat_Data_Updata=false;
		if(PowerHeatData.chassis_power>=30)	//底盘功率上限30w
			Chassic_Speed_Offset-=500;
		if(PowerHeatData.shooter_id1_17mm_cooling_heat>=320)	//热量上限320J
		Cartridge_Speed_Offset-=500;
	}
}
