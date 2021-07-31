/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "bsp_can.h"
#include "classic.h"
#include "shoot.h"
#include "judge.h"
#include "bsp_usart.h"
#include "bullet.h"
#include "bsp_judge.h"
#include <stdlib.h>
#include <stdbool.h>
#include "buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define touch_Left (-1)
#define touch_Right 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t Time_Tick=0;	//计时，一秒内没有收到云台数据停止动作
uint8_t Motor_Power_Up=0;	//判断电机上电，1上电完成
uint8_t Shoot_Ultra_Mode=1;	//剩余热量多，高射速消耗热量，1有效
int Heat_Rest=0;	//热量剩余,中断更新
float Rail_Position=0.f;	//轨道位置,中断更新
bool Hit_Gimbal=false;	//云台撞柱
uint32_t Ka_Dan_cnt=0;	//卡弹时间计数
bool Ka_Dan=false;	//卡单标志位
uint32_t Ka_Dan_Times=0;	//卡弹次数
bool Ka_Dan_Stop=false;	//卡弹次数过多拨弹停转
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	static uint32_t TIM1_cnt=0;	//can分频用
	++TIM1_cnt;
	if(TIM1_cnt==0xFFFFFFFF)
		TIM1_cnt=0;
	
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	if(Motor_Power_Up==0)	//电机没上电,蜂鸣器响
	{
		if(	gear_motor_data[Chassic_L].real_current!=0 &&
				gear_motor_data[Chassic_R].real_current!=0 &&
				gear_motor_data[Cartridge].real_current!=0)
		Motor_Power_Up=1;
		else
			Buzzer_ms(1,50,2000);
	}

	++Time_Tick;	//Time_Tick在CAN接收服务函数中置零
	if(Time_Tick>200)	//没有云台数据
	{
		#ifdef Test_Mode	//测试模式	没数据停止
			Move_Allow=0;
		#else
			Move_Allow=1;	//比赛	没数据保持运动
		#endif
		Shoot_State=0;
		Switch_State[0]=Switch_State[1]=1;	//微动开关状态,没用到
	}
	
	if(Changing_Speed_Flag==0)	//速度反向时不检测光电开关	close check
	{
		if(HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin) == GPIO_PIN_RESET)	//左边检测到墙壁，开始反转
		{
			eliminate_dithering_right = 0;	//清空另一边的消抖计数
			eliminate_dithering_left++;
			if (eliminate_dithering_left >= 20) //消抖
			{
				if(Classic_Move_Speed==Chassic_Spring_Fast)
					Classic_Move_Speed=Chassic_Spring_Middle;	//恢复巡检速度
				#ifdef USE_SPRING
					direction = touch_Left;
				#else
					direction = Last_Dir = touch_Left;
				#endif
				Changing_Speed_Flag=1;	//方向改变，标志位置1
				Buzzer_Short(1);
			}
		}
		else if(HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_RESET)
		{
			eliminate_dithering_right++;	
			eliminate_dithering_left = 0;	//清空另一边的消抖计数
			if (eliminate_dithering_right >= 20)
			{
				if(Classic_Move_Speed==Chassic_Spring_Fast)
					Classic_Move_Speed=Chassic_Spring_Middle;	//恢复巡检速度
				#ifdef USE_SPRING
					direction = touch_Right;
				#else
					direction = Last_Dir = touch_Right;
				#endif
				Hit_Gimbal=true;
				Changing_Speed_Flag=1;	//方向改变，标志位置1
				Buzzer_Short(1);	//蜂鸣器
			}
		}
	}
	else if(Changing_Speed_Flag==1)	//Leave check
	{
		if(direction==touch_Left)	//远离柱子,光电开关不触发,圈数计数清零
		{
			if(HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin) == GPIO_PIN_SET)
			{
				Changing_Speed_Flag=0;
				gear_motor_data[Moto_ID[0]].round_cnt=0;	//离开柱子,圈数清零
			}
		}
		else if(direction==touch_Right)
		{
			if(HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_SET)
			{
				Changing_Speed_Flag=0;
				Hit_Gimbal=false;
				gear_motor_data[Moto_ID[0]].round_cnt=0;	//离开柱子,圈数清零
			}
		}
	}

	if(Measuer_State==End_Measure)	//轨道测量结束,更新在轨位置
		Rail_Position=(abs(gear_motor_data[Moto_ID[0]].round_cnt)*1.f)/(Rail_Len*1.f);
	
	switch (Move_Allow)
	{
		case 1:	//正常移动
		{
			#ifdef USE_SPRING
				Check_Being_Hit();	//被击打改变速度or toggle direction
			#endif
				if(Measuer_State==End_Measure)	//已获得轨道长度
				{
					if(Aim==false)	//没有瞄准到目标：随机变向
					{
						Rand_Dir_Time();
					}
				}
				Spring(direction,Classic_Move_Speed);	//处理弹簧反弹和底盘PID计算
			break;
		}
		
		case 2:	//获取轨道长度
		{
			if(Measuer_State!=End_Measure)	//没有测量完成,正常移动
			{
				Spring(direction,Classic_Move_Speed);
				Measuer_Rail_Len();
			}
			break;
		}
		
		case 3:		//回到轨道中间 有bug 这个case暂时没有使用
			if(Measuer_State==End_Measure)
				Go_To_Middle(Chassic_Spring_Slow);
			else
			{
				motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,0);	//缓慢减速
				for(uint8_t i=0; i<2; ++i)
				{
					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
		break;
		
		default:	//停止动作
		{
			for(uint8_t i=0; i<2; ++i)
			{
				motor_pid[i].target=0;
				motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
				Motor_Output[ Moto_ID[i] ]=0;
			}
			break;
		}
	}
	
	Heat_Rest=320-PowerHeatData.shooter_id1_17mm_cooling_heat;	//剩余热量
	if(Heat_Rest>200)	//剩余热量多，高射频消耗热量
		Shoot_Ultra_Mode=1;
	else if(Heat_Rest<100)
		Shoot_Ultra_Mode=0;
	
	if(Shoot_State!=0)
	{
		if(Ka_Dan==false)
		{
			switch(Shoot_State)	//射击模式
			{
				case 1:		//Single	单发位置环 没有使用 保留代码
					if(Heat_Rest>20)
					{
						Shoot_State=0;
						Cartridge_angle=(Cartridge_angle+45)%360;
						Cartridge_wheel.output=gear_moto_position_pid_calc(&Cartridge_Position_Pid[OUT],&Cartridge_Position_Pid[IN],Cartridge_angle,gear_motor_data[Cartridge].real_total_angle,gear_motor_data[Cartridge].speed_rpm);
					}
					else
					{
						Cartridge_wheel_PID_Calc(0);
						Motor_Output[Cartridge]=0;
					}
				break;
				
				case 2:		//Fast
					if(Shoot_Ultra_Mode==1)	//剩余热量多,高射频
						Cartridge_wheel_PID_Calc(Cartridge_Ultra);
					else if(Heat_Rest>50)	//较少,低射频
						Cartridge_wheel_PID_Calc(Cartridge_Fast);
					else
						Cartridge_wheel_PID_Calc(Cartridge_Slow);
					Motor_Output[Cartridge]=Cartridge_wheel.output;
				break;
				
				case 3:		//Slow
					if(Heat_Rest>40)
					{
						Cartridge_wheel_PID_Calc(Cartridge_Slow);
						Motor_Output[Cartridge]=Cartridge_wheel.output;
					}
					else
					{
						Cartridge_wheel_PID_Calc(0);
						Motor_Output[Cartridge]=0;
					}
				break;
				
				default:
					Cartridge_wheel_PID_Calc(0);
					Motor_Output[Cartridge]=0;
				break;
			}
		}
		
		if(Motor_Output[Cartridge]>9500 && Ka_Dan==false)	//拨弹盘堵转检测
		{
			++Ka_Dan_cnt;	//计数值自加
			if(Ka_Dan_cnt>400)	//1s 400hz
			{
				++Ka_Dan_Times;
				Ka_Dan=true;	//卡单标志位
				if(Ka_Dan_Times%5!=0)
				{
					Ka_Dan_cnt=400;	//反转时间
				}
				else
				{
					Ka_Dan_Stop=true;
					Ka_Dan_cnt=2000;	//停转时间
				}
				Buzzer_Short(1);
			}
		}
		else if(Ka_Dan==true)	//卡单反转
		{
			--Ka_Dan_cnt;
			if(Ka_Dan_cnt!=0)
			{
				if(Ka_Dan_Stop==false)
				{
					Cartridge_wheel_PID_Calc(-Cartridge_Fast);
					Motor_Output[Cartridge]=Cartridge_wheel.output;
				}
				else
				{
					Cartridge_wheel_PID_Calc(0);
					Motor_Output[Cartridge]=0;
				}
			}
			else
				Ka_Dan_Stop=Ka_Dan=false;	//清空标志位
		}
		else
			Ka_Dan_cnt=0;
	}
	else
	{
		Cartridge_wheel_PID_Calc(0);
		Motor_Output[Cartridge]=0;
	}
	
	if(TIM1_cnt%2==0)		//2 freq div
	{
		CAN_Motor_Ctrl(&hcan2,Motor_Output);
	}
	else
	{
		uint8_t Data[8]={0};
		Data[0]=is_red_or_blue();		//红蓝方 视觉需求
		Data[1]=Hit_Gimbal;		//云台规避
		
		#ifdef Test_Mode
			Set_Game_Start();	//测试的时候手动设置裁判系统状态
		#endif
		if(GameState.game_progress==4)	//比赛开始
			Data[2]=true;
		else
		{
			Data[2]=false;	//比赛未开始,前哨站和基地护盾默认存在
			Field_Event_Data.Outpost_Alive=true;
			Field_Event_Data.Base_Shield_Existence=true;
		}
		
		#ifdef Test_Mode
			Set_Outpost_Alive();
		#endif
		Data[3]=Field_Event_Data.Outpost_Alive;	//前哨站存活状态
		
		#ifdef Test_Mode
			Set_Base_Shield_Existence();
		#endif
		Data[4]=Field_Event_Data.Base_Shield_Existence;	//基地护盾

		#warning	//wait for test
		#ifndef Test_Mode
			if(BulletRemaining.bullet_remaining_num_17mm==0 && GameState.game_progress==4)	//500发		Shootable
				Data[5]=false;	//剩余发弹量为0,向云台发送数据,具体处理在云台
			else
		#endif
			Data[5]=true;
		
		CAN_Send_Gimbal(&hcan1,Data,6);	//发送数据给云台

		if(Inspect_Position!=0)	//裁判系统读回,云台手的控制指令
		{
			/*	0 Normal
				 *	1-4 Quadrant
				 *	5 Front
				 *	6 Back
				*/
			uint8_t Data_Gimbal2[8]={0};
			Data_Gimbal2[0]=Inspect_Position;
			memcpy(&Data_Gimbal2[1],&Target_Angle,sizeof(float));	//云台手标记相对于哨兵的角度
			if(CAN_Send_Gimbal2(&hcan1,Data_Gimbal2,1+sizeof(float))==HAL_OK)	//send success clean flag
				Inspect_Position=0;
		}
	}
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	static uint32_t TIM6_cnt=0;
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	if(Buzzer_Busy==false && Buzzer_cnt!=0 && Buzzer_On_Time!=0)	//蜂鸣器
		Buzzer_Busy=true;	//调用蜂鸣器发声函数会置Buzzer_cnt和Buzzer_On_Time的值
	if(Buzzer_Busy==true)
	{
		++TIM6_cnt;	//蜂鸣器忙状态计数自增
		if(TIM6_cnt<Buzzer_On_Time)	//在Buzzer_On_Time内蜂鸣器响
		{
			if(Buzzer_Working==false)
			{
				Buzzer_ON();
				Buzzer_Working=true;
			}
		}
		else if(TIM6_cnt<Buzzer_On_Time+Buzzer_Off_Time)	//在Buzzer_Off_Time内蜂鸣器不响
		{
			if(Buzzer_Working==true)
			{
				Buzzer_OFF();
				Buzzer_Working=false;
			}
		}
		else
		{
			TIM6_cnt=0;
			--Buzzer_cnt;	//蜂鸣器循环次数自减
			if(Buzzer_cnt==0)
				Buzzer_Busy=false;
		}
	}
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupts.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
	Dma_UsartIdleHanlder(&huart6, JUDGEMENT_BUF_LEN);
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
