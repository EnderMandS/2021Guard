#include "guard_judge.h"
#include "judge.h"

uint8_t Shoot_Heat_Allow=0;

extern int Firc_Speed;

void Shoot_Speed_Check(void)
{
	if(ShootData.bullet_speed > GameRobotStat.shooter_heat0_speed_limit)
	{
		if(Firc_Speed<-10)
			Firc_Speed += 10;
	}
}
