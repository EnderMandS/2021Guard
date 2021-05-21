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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define touch_Left -1
#define touch_Right 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int32_t Fric_Speed=-0;
uint32_t Time_Tick=0;	//计时，一秒内没有收到云台数据停止动作
uint8_t Motor_Power_Up=0;	//判断电机上电，1上电完成
uint8_t Shoot_Ultra_Mode=0;	//剩余热量多，高射速消耗热量，1有效
int Heat_Rest=0;
float Rail_Position=0.f;
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
	static uint32_t TIM1_cnt=0;
	++TIM1_cnt;
	if(TIM1_cnt==0xFFFFFFFF)
		TIM1_cnt=0;
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	if(Motor_Power_Up==0)
	{
		if(	gear_motor_data[Chassic_L].real_current!=0 &&
				gear_motor_data[Chassic_R].real_current!=0 &&
				gear_motor_data[Cartridge].real_current!=0)
			Motor_Power_Up=1;
	}
	else
	{
//		Updata_Switch_State();
		++Time_Tick;	//Time_Tick在CAN接收服务函数中置零
		if(Time_Tick>200)	//没有云台数据，停止动作
		{
			Move_Allow=Shoot_State=0;
			Switch_State[0]=Switch_State[1]=1;
		}
		
		if(Changing_Speed_Flag==0)	//速度反向时不检测光电开关
		{
			if(HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin) == GPIO_PIN_RESET)	//左边检测到墙壁，开始反转
			{
				eliminate_dithering_right = 0;
				eliminate_dithering_left++;
				if (eliminate_dithering_left == 20) //消抖
				{
					direction = touch_Left;
					Changing_Speed_Flag=1;	//方向改变，标志位置1
				}
			}
			else if(HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_RESET)
			{
				eliminate_dithering_right++;
				eliminate_dithering_left = 0;
				if (eliminate_dithering_right == 20)
				{
					direction = touch_Right;
					Changing_Speed_Flag=1;	//方向改变，标志位置1
				}
			}
		}
		#ifdef USE_SPRING
			else if(Changing_Speed_Flag==1)
			{
				if(direction==touch_Left)
				{
					if(HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin) == GPIO_PIN_SET)
					{
							Changing_Speed_Flag=0;
					}
				}
				else if(direction==touch_Right)
				{
					if(HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_SET)
					{
							Changing_Speed_Flag=0;
					}
				}
			}
		#endif
		
		Check_Being_Hit();	//被击打改变速度
		
		if(Measuer_State==End_Measure)	//已获得轨道长度
		{
			Rail_Position=(abs(gear_motor_data[Moto_ID[0]].round_cnt)*1.f)/(Rail_Len*1.f);	//轨道位置0-1
			if(Aim==false)	//没有瞄准到目标：随机变向
			{
				float range=0.025f;	//相对于变向点的变向范围
				float point_1=0.33f;	//变向点1
				float point_2=0.66f;	//变向点2
				if(	(Rail_Position>point_1-range && Rail_Position<point_1+range)	||
						(Rail_Position>point_2-range && Rail_Position<point_2+range)	)	//是否在变向点
				{
					if(Rand_Change_Flag==false)	//是否已执行变向函数
					{
						Rand_Change_Flag=true;
						Rand_Dir_Change();
					}
				}
				else
					Rand_Change_Flag=false;
			}
		}
		
		switch (Move_Allow)
		{
			case 1:	//正常移动
			{
				#ifndef USE_SPRING
					motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,Classic_Move_Speed);
					for (uint8_t i=0; i<2; i++)
					{
						motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm); //根据设定值进行PID计算。
						Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
					}
				#else
					Spring(direction,Classic_Move_Speed);
				#endif
				break;
			}
			
			case 2:	//获取轨道长度
			{
				if(Measuer_State!=End_Measure)
				{
					#ifndef USE_SPRING
						motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,Classic_Move_Speed);
						for (uint8_t i=0; i<2; i++)
						{
							motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm); //根据设定值进行PID计算。
							Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
						}
					#else
						Spring(direction,Classic_Move_Speed);
					#endif
					Measuer_Rail_Len();
				}
				else	//测量完之后回到轨道中间停下
				{
					Go_To_Middle(Chassic_Spring_Slow);
				}
				break;
			}
			
			case 3:		//回到轨道中间
				if(Measuer_State==End_Measure)
					Go_To_Middle(Chassic_Spring_Slow);
				else
				{
					motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,0);
					for(uint8_t i=0; i<2; ++i)
					{
						motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
						Motor_Output[ Moto_ID[i] ]=0;
					}
				}
			break;
			
			default:	//停止动作
			{
				motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,0);
				for(uint8_t i=0; i<2; ++i)
				{
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
		
		switch(Shoot_State)	//射击模式
		{
			case 1:		//Single
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
				if(Shoot_Ultra_Mode==1)
					Cartridge_wheel_PID_Calc(Cartridge_Ultra);
				else if(Heat_Rest>50)
					Cartridge_wheel_PID_Calc(Cartridge_Fast);
				else
					Cartridge_wheel_PID_Calc(Cartridge_Slow);
				Motor_Output[Cartridge]=Cartridge_wheel.output;
			break;
			
			case 3:		//Slow
				if(Heat_Rest>20)
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
			
			default:	//Load
//				if(Switch_State[1]==0)
//				{
//					Cartridge_wheel_PID_Calc(1000);
//					Motor_Output[Cartridge]=Cartridge_wheel.output;
//				}
//				else
//				{
//					Cartridge_wheel_PID_Calc(0);
//					Motor_Output[Cartridge]=0;
//				}
				Cartridge_wheel_PID_Calc(0);
				Motor_Output[Cartridge]=0;
			break;
		}
		
		
		if(TIM1_cnt%2==0)		//2 freq div
		{
			CAN_Motor_Ctrl(&hcan2,Motor_Output);
			
			uint8_t Data[8]={0};
			Data[0]=is_red_or_blue();		//红蓝方 视觉需求
			Data[1]=direction;		//移动方向
			Data[2]=Last_Dir;		//云台规避
			
			if(GameState.game_progress==4)	//比赛开始
				Data[3]=true;
			else
				Data[3]=false;
			
			Data[4]=Field_Event_Data.Outpost_Alive;	//前哨站存活状态
			
			Data[5]=Field_Event_Data.Base_Shield_Existence;	//基地护盾
			
			if(BulletRemaining.bullet_remaining_num_17mm==0)	//500发
				Data[6]=false;
			else
				Data[6]=true;
			
			Data[7]=Inspect_Position;	//receive data from other robot, send to gimbal where to inspect
			
			CAN_Send_Gimbal(&hcan1,Data,8);
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
