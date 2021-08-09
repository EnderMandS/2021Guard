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
#include "usartinfo.h"
#include "bsp_usart.h"
#include "pid.h"
#include "caninfo.h"
#include "Remote_control.h"
#include "gimbal.h"
#include "can.h"
#include "shoot.h"
#include "classic.h"
#include "guard_judge.h"
#include "buzzer.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define shoot_speed 1	//Speed for 1   position for 0

float yaw_nowangle=301.5f;
float pit_nowangle=98.f;
int control_mode = 0; //Contorl Mode: 0 for protect, 1 for remote, 2 for auto

uint8_t TIM1_Div=0;	//分频计数

/*
0x200	1,2,3,4
0x1FF	5,6,7,8		1,2,3,4
0x2FF						5,6,7
*/

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int Firc_Speed=6200;		//6100
/*
	3000 10m/s
	
	5000 21.6m/s
	6000 26.2m/s
*/

uint8_t Motor_Power_Up=0;	//电机上电标志位
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
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
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

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
	
	// 400Hz

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	
	//Gyro Online Cheack, clear in usart interrupt
	++Gryo_Update_cnt;
	if(Gryo_Update_cnt>=400*3)	//3 seconds without gyro data, switch to motor control
		Pitch_USE_Gyro=false;
	
	if(Motor_Power_Up==0)	//Wait for motor & gryo power up
	{
		if(	gear_motor_data[Gimbal_Y].real_current!=0 &&
			gear_motor_data[Gimbal_P].real_current!=0 &&
			gear_motor_data[Fric_1].real_current!=0   &&
			gear_motor_data[Fric_2].real_current!=0	  &&
			Hi229_Update!=0	)
				Motor_Power_Up=1;
		else
			Buzzer_ms(1,50,2000);
	}

	yaw_nowangle = Yaw_Motor_Angle_Change();	//获得yaw角度,0-360,有同步带,需要角度转换
	if(Pitch_USE_Gyro==true)
		pit_nowangle = eular[0];
	else
		pit_nowangle = gear_motor_data[Gimbal_P].angle*Motor_Ecd_to_Ang - Zero_Offset_Cal();	//motor angle - zero offset
	
	switch(remote_control.switch_right)	//left switch
	{
		case 1:	//Auto
		{
			Motor_Output_State[Gimbal_Y]=Motor_Output_State[Gimbal_P]=1;	//使能云台can输出
			Gimbal_Sotf_Start();	//云台缓起
			Gimbal_Automatic_control();	//自动控制
			switch(view_shoot_mode)	//Flick	DD:None, EE:Slow, FF:Fast
			{
				case 0xEE:	//Fast
					Shoot_Ctrl=2;
					break;
				
				case 0xFF:	//Slow
					
					Shoot_Ctrl=3;
					break;
				
				default:		//None
					Shoot_Ctrl=0;
					break;
			}
			
			#define Auto_Ctrl
			#ifdef Auto_Ctrl	//According to match state, chooce different function.  Auto priority higher than remote
				if(remote_control.switch_left==2 && Game_Start==true)	//Mtach start open fric
					remote_control.switch_left=3;
				if(remote_control.switch_left==3 && Outpost_Alive==false)	//Outpost not alive, open chassis
					remote_control.switch_left=1;
				if((remote_control.switch_left==3||remote_control.switch_left==1) && Game_Start==false)	//Match end, back to railway middle
					remote_control.switch_left=2;
			#else		//Match start open friction,chassis.	Remote priority higher than auto
				if(remote_control.switch_left==2 && Game_Start==true)
					remote_control.switch_left=1;
			#endif
			
			switch(remote_control.switch_left)	//自动控制模式下左拨弹状态
			{
				case 1:	//Chassis + Friction
					Chassic_State=1;	//底盘移动
					if(Shootable==false)	//底盘裁判系统读回
						Shoot_Speed_Pid_Calc(0);
					else
					{
						Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=1;
						Shoot_Speed_Pid_Calc(Firc_Speed);
					}
					break;
				
				case 3:	//Friction
					Chassic_State=0;	//底盘停止
					if(Shootable==false)
						Shoot_Speed_Pid_Calc(0);
					else
					{
						Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=1;
						Shoot_Speed_Pid_Calc(Firc_Speed);
					}
					break;
				
				default:
					Chassic_State=0;
					Shoot_Speed_Pid_Calc(0);
					Shoot_Ctrl=0;	//None friction forbid flick
					break;
			}
		}
		break;
		
		case 3:	//Remote
		{
			Motor_Output_State[Gimbal_Y]=Motor_Output_State[Gimbal_P]=1;	//使能云台can输出
			Gimbal_Sotf_Start();	//缓起
			Gimbal_Remote_Control();	//遥控
			if(sotf_start==0)		//wait for gimbal soft start
			{
				switch(remote_control.switch_left)	//遥控模式下左拨杆
				{
					case 1:	//chassis 只有底盘
					{
						Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=0;
						Shoot_Speed_Pid_Calc(0);
						Shoot_Ctrl=0;
						Chassic_State=1;
					}
					break;
					
					case 3:	//friction + flick	摩擦轮+拨弹
					{
						Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=1;
						Shoot_Speed_Pid_Calc(Firc_Speed);
						Shoot_Ctrl=2;	//Fast高射速
						Chassic_State=0;
					}
					break;
					
					case 2:	//None
					{
						Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=0;
						Shoot_Ctrl=0;
						Chassic_State=0;
						Shoot_Speed_Pid_Calc(0);	//friction
					}
					break;
				}
			}
		}
		break;
		
		case 2:	//None
		{
			read_allow = 0;	//清空缓起标志位
			control_allow = 0;
			sotf_start = 1;
			Shoot_Speed_Pid_Calc(0);	//friction
			Motor_Output[Fric_1]=Motor_Output[Fric_2]=0;
			switch(remote_control.switch_left)
			{
				case 1:	//轨长采样
					Motor_Output_State[Gimbal_Y]=1;
					if( Gimbal_Keep_Middle()==true )
						Chassic_State=2;	//Get railway lenth
				break;
				
				default:
					Chassic_State=0;
					Shoot_Ctrl=0;
				break;
			}
		}
		break;
		
		default:
			break;
	}

	//使能为PID输出,失能为0
	if(Motor_Output_State[Gimbal_Y]==1)
		Motor_Output[Gimbal_Y]=yaw;
	else
		Motor_Output[Gimbal_Y]=0;
	
	if(Motor_Output_State[Gimbal_P]==1)
		Motor_Output[Gimbal_P]=pitch;
	else
		Motor_Output[Gimbal_P]=0;
	
	if(Motor_Output_State[Fric_1]==1)
		Motor_Output[Fric_1]=Fric_wheel[0].output;
	else
		Motor_Output[Fric_1]=0;
	
	if(Motor_Output_State[Fric_2]==1)
		Motor_Output[Fric_2]=Fric_wheel[1].output;
	else
		Motor_Output[Fric_2]=0;
	
	uint8_t Switch_State=0;	//Micro switch state	没用到
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET)
		Switch_State=0;
	else
		Switch_State=1;
	
	++TIM1_Div;	//计数自增
	if(TIM1_Div>=2)	//底盘控制
	{
		TIM1_Div=0;
		if(Shootable==false && Outpost_Alive==false)	//Read for judgement, unablde to shoot, 500 bullte limit
		{
			Chassic_State=1;
			Shoot_Ctrl=0;
			Aimming=false;
		}
		uint8_t Chassic_Data[5]={Chassic_State,Shoot_Ctrl,Switch_State,Aimming,Inspect_Position};
		Chassic_Ctrl(Chassic_Data,5);	//发给底盘
	}
	else if(TIM1_Div==1)	//电机控制
	{
		CAN_Motor_Ctrl(&hcan1,Motor_Output);
		memset(Motor_Output_State,0,sizeof(Motor_Output_State));	//clean
		Chassic_State=Shoot_Ctrl=0;
	}
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	Dma_UsartIdleHanlder(&huart1, VIEW_BUF_LEN);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	Dma_UsartIdleHanlder(&huart3, 36);
  /* USER CODE END USART3_IRQn 1 */
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
	if(Buzzer_Busy==false && Buzzer_cnt!=0 && Buzzer_On_Time!=0)	//蜂鸣器忙判断
		Buzzer_Busy=true;
	if(Buzzer_Busy==true)
	{
		++TIM6_cnt;	//计数自增
		if(TIM6_cnt<Buzzer_On_Time)	//在On_Time内响
		{
			if(Buzzer_Working==false)
			{
				Buzzer_ON();
				Buzzer_Working=true;
			}
		}
		else if(TIM6_cnt<Buzzer_On_Time+Buzzer_Off_Time)	//off_time停止
		{
			if(Buzzer_Working==true)
			{
				Buzzer_OFF();
				Buzzer_Working=false;
			}
		}
		else
		{	//on和off都计数完成，计数自减
			TIM6_cnt=0;
			--Buzzer_cnt;
			if(Buzzer_cnt==0)	//自减到零蜂鸣器非忙
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
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
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
	Dma_UsartIdleHanlder(&huart6, GROY_DATA_BUF_LEN);
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
