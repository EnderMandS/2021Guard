#include "bsp_judge.h"
#include "judge.h"
#include <stdbool.h>

void Check_Being_Hit(void)	//200Hz 1秒内受到三次装甲伤害动作
{
	static uint16_t Time_cnt=0;
	static uint8_t Hit_cnt=0;
	++Time_cnt;
	if(Hurt_Data_Update==true)
	{
		Hurt_Data_Update=false;
		++Hit_cnt;
	}
	if(Time_cnt>200)
	{
		if(Hit_cnt>=3)
			;
		else
			Time_cnt=Hit_cnt=0;
	}
}
