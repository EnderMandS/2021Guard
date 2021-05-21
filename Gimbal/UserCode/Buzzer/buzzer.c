#include "buzzer.h"
#include "tim.h"

uint16_t PSC=5;		//0-1000
uint16_t PWM=10000;	//10000-20000

uint32_t Buzzer_cnt=0;
uint32_t Buzzer_On_Time=0;
uint32_t Buzzer_Off_Time=0;
bool Buzzer_Busy=false;
bool Buzzer_Working=false;

void Buzzer_Init(void)
{
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	Buzzer_ON();
	HAL_Delay(1000);
	Buzzer_OFF();
}
void Buzzer_ON(void)
{
	__HAL_TIM_PRESCALER(&htim4, PSC);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, PWM);
}
void Buzzer_OFF(void)
{
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
void Buzzer_ms(uint16_t cnt, uint16_t On_Time, uint16_t Off_Time)
{
	if(Buzzer_Busy==false)
	{
		Buzzer_cnt=cnt;
		Buzzer_On_Time=On_Time;
		Buzzer_Off_Time=Off_Time;
	}
}
