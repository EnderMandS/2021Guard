#include "buzzer.h"
#include "tim.h"

uint16_t PSC=7;		//0-1000	这两个参数改变控制音调
uint16_t PWM=10000;	//10000-20000

uint32_t Buzzer_cnt=0;	//循环次数计数
uint32_t Buzzer_On_Time=0;	//响时长
uint32_t Buzzer_Off_Time=0;	//停时长
bool Buzzer_Busy=false;	//蜂鸣器忙标志
bool Buzzer_Working=false;

void Buzzer_Init(void)	//蜂鸣器初始化
{
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	Buzzer_ON();	//上电响一声
	HAL_Delay(500);
	Buzzer_OFF();
}
void Buzzer_ON(void)	//蜂鸣器响
{
	__HAL_TIM_PRESCALER(&htim4, PSC);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, PWM);
}
void Buzzer_OFF(void)	//蜂鸣器停
{
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
void Buzzer_ms(uint16_t cnt, uint16_t On_Time, uint16_t Off_Time)	//蜂鸣器控制  cnt:循环次数	On_Time:响时长	Off_Time:停时长	单位ms
{
	if(Buzzer_Busy==false)	//非忙状态时响应
	{
		Buzzer_cnt=cnt;
		Buzzer_On_Time=On_Time;
		Buzzer_Off_Time=Off_Time;
	}
}
void Buzzer_Short(uint8_t cnt)	//短响
{
	Buzzer_cnt=cnt;
	Buzzer_Off_Time=Buzzer_On_Time=50;
}
void Buzzer_Middle(uint8_t cnt)	//中等
{
	Buzzer_cnt=cnt;
	Buzzer_Off_Time=Buzzer_On_Time=150;
}
void Buzzer_Long(uint8_t cnt)	//长鸣
{
	Buzzer_cnt=cnt;
	Buzzer_Off_Time=Buzzer_On_Time=500;
}
