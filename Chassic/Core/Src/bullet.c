#include "bullet.h"
#include "bsp_can.h"
#include "judge.h"
#include "shoot.h"
#include "classic.h"

#define Heat_17mm 10
#define Heat_42mm 100

uint8_t Switch_State[2]={0};	// [0]Up  [1]Down

void Updata_Switch_State(void)
{
	switch( HAL_GPIO_ReadPin(Shoot_Switch_Up_Pin, Shoot_Switch_Up_Port) )
	{
		case GPIO_PIN_RESET:
			Switch_State[0]=0;
		break;
		
		case GPIO_PIN_SET:
			Switch_State[0]=1;
		break;
	}
}
uint8_t Enough_Heat(void)
{
	int Heat_Rest=GameRobotStat.shooter_id1_17mm_cooling_limit-PowerHeatData.shooter_id1_17mm_cooling_heat;
	if(Heat_Rest>50)
		return 3;
	if(Heat_Rest>30)
		return 2;
	if(Heat_Rest>10)
		return 1;
	return 0;
}
