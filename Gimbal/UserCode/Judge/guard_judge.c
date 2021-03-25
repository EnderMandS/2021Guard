#include "guard_judge.h"
#include "judge.h"

uint8_t Shoot_Heat_Allow=0;

extern int Classic_Move_Speed;
extern int Firc_Speed;

void Classic_Speed_Check(void)
{
	if(PowerHeatData.chassis_power > GameRobotStat.max_chassis_power)
	{
		if(Classic_Move_Speed>10)
			Classic_Move_Speed -= 10;
	}
}
void Shoot_Speed_Check(void)
{
	if(ShootData.bullet_speed > GameRobotStat.shooter_heat0_speed_limit)
	{
		if(Firc_Speed<-10)
			Firc_Speed += 10;
	}
}
void Shoot_Heat_Limit(void)
{
	if( (PowerHeatData.shooter_heat0+10) < GameRobotStat.shooter_heat0_cooling_limit)
	{
		Shoot_Heat_Allow=1;
	}
	else
	{
		Shoot_Heat_Allow=0;
	}
}
