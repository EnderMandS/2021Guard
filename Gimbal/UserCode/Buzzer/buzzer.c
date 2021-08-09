/*
 * @Author: M
 * @Date: 2021-05-17 18:20:08
 * @LastEditTime: 2021-08-09 15:29:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMe:\RM\Guard\NewGuard\Gimbal\UserCode\Buzzer\buzzer.c
 */
#include "buzzer.h"
#include "tim.h"

uint16_t PSC=5;		//0-1000
uint16_t PWM=10000;	//10000-20000

uint32_t Buzzer_cnt=0;
uint32_t Buzzer_On_Time=0;
uint32_t Buzzer_Off_Time=0;
bool Buzzer_Busy=false;
bool Buzzer_Working=false;

/**
 * @description: 蜂鸣器初始化
 * @param {*}
 * @return {*}
 */
void Buzzer_Init(void)
{
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	Buzzer_ON();
	HAL_Delay(500);
	Buzzer_OFF();
}
/**
 * @description: 蜂鸣器响
 * @param {*}
 * @return {*}
 */
void Buzzer_ON(void)
{
	__HAL_TIM_PRESCALER(&htim4, PSC);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, PWM);
}
/**
 * @description: 蜂鸣器停
 * @param {*}
 * @return {*}
 */
void Buzzer_OFF(void)
{
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
/**
 * @description: 蜂鸣器响控制
 * @param {uint16_t} cnt 次数
 * @param {uint16_t} On_Time 响时长
 * @param {uint16_t} Off_Time 暂歇时长
 * @return {*}
 */
void Buzzer_ms(uint16_t cnt, uint16_t On_Time, uint16_t Off_Time)
{
	if(Buzzer_Busy==false)
	{
		Buzzer_cnt=cnt;
		Buzzer_On_Time=On_Time;
		Buzzer_Off_Time=Off_Time;
	}
}
/**
 * @description: 蜂鸣器短鸣 50ms
 * @param {uint8_t} cnt 次数
 * @return {*}
 */
void Buzzer_Short(uint8_t cnt)
{
	Buzzer_cnt=cnt;
	Buzzer_Off_Time=Buzzer_On_Time=50;
}
/**
 * @description: 蜂鸣器中鸣 150ms
 * @param {uint8_t} cnt 次数
 * @return {*}
 */
void Buzzer_Middle(uint8_t cnt)
{
	Buzzer_cnt=cnt;
	Buzzer_Off_Time=Buzzer_On_Time=150;
}
/**
 * @description: 蜂鸣器长鸣 500ms
 * @param {uint8_t} cnt 次数
 * @return {*}
 */
void Buzzer_Long(uint8_t cnt)
{
	Buzzer_cnt=cnt;
	Buzzer_Off_Time=Buzzer_On_Time=500;
}
