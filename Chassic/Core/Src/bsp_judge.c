#include "bsp_judge.h"
#include "judge.h"
#include "classic.h"
#include <stdbool.h>

int Chassic_Speed_Offset=0;
int Cartridge_Speed_Offset=0;

void Check_Being_Hit(void)	//200Hz 2秒内受到三次装甲伤害动作 冷却3秒
{
	static uint16_t Time_cnt=0;
	static uint16_t Hit_cnt=0;
	static uint8_t Change_Speed=0;
	
	++Time_cnt;
	if(Hurt_Data_Update==true)
	{
		Hurt_Data_Update=false;
		++Hit_cnt;
	}
	if(Time_cnt>400 && Change_Speed==0)
	{
		if(Hit_cnt>=3)
			Change_Speed=1;
		Time_cnt=Hit_cnt=0;
	}
	if(Change_Speed==1)
	{
		if(HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_SET && Changing_Speed_Flag==0)	//没碰到墙
		{
			direction=-direction;
			Changing_Speed_Flag=1;
		}
		Change_Speed=2;
	}
	if(Change_Speed==2 && Time_cnt>600)
	{
		Change_Speed=Time_cnt=Hit_cnt=0;
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
