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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
//速度环为1，位置环为0
#define shoot_speed 1


float yaw_nowangle;
float pit_nowangle;
int control_mode = 0; //控制模式0为保护模式，1为手控模式，2为自瞄模式
int Cartridge_output;
int set_spd_to_Classis_wheel;

//#define Classic_Move
#define touch_Left -1
#define touch_Right 1
int direction=1;
int eliminate_dithering_left = 0;
int eliminate_dithering_right = 0;

uint16_t Change_Dir_Cnt=0;

/*
0x200	1,2,3,4
0x1FF	5,6,7,8		1,2,3,4
0x2FF						5,6,7
*/

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define Classic_Move_Speed 700
//#define Firc_Speed -1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int Classic_Move_Speed=700;
int Firc_Speed=-1000;
uint16_t Cartridge_angle=0;
uint32_t Cartridge_TIM_cnt=0;
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

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	yaw_nowangle = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang;
	pit_nowangle = gear_motor_data[Gimbal_P].angle * Motor_Ecd_to_Ang;
	
	switch(remote_control.switch_right)	//右拨杆
	{
		case 1:	//自瞄
		{
			Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=Motor_Output_State[Gimbal_Y]=Motor_Output_State[Gimbal_P]=1;
			if(remote_control.switch_left==1)
				Chassic_State=1;
			Gimbal_Automatic_control();
			switch(view_shoot_mode)	//拨弹	DD:不响应 EE:低速发射 FF:高速发射	
			{
				case 0xEE:	//低速
					Shoot_Ctrl=2;
					break;
				
				case 0xFF:	//高速
					Shoot_Ctrl=3;
					break;
				
				default:		//不发射
					Shoot_Ctrl=0;
					break;
			}
			Shoot_Speed_Pid_Calc(Firc_Speed);	//摩擦轮
		}
		break;
		 
		case 3:	//遥控
		{
			Motor_Output_State[Gimbal_Y]=Motor_Output_State[Gimbal_P]=1;
			Gimbal_Sotf_Start();
			Gimbal_Remote_Control();
			if(sotf_start==0)		//等待云台缓起完成
			{
				switch(remote_control.switch_left)	//左拨杆
				{
					case 1:	//摩擦轮+拨弹+底盘
					{
						Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=1;
						Shoot_Speed_Pid_Calc(Firc_Speed);	//摩擦轮
						Shoot_Ctrl=3;
						Chassic_State=1;
					}
					break;
					
					case 3:	//摩擦轮+定时位置环拨弹
					{
						Motor_Output_State[Chassic_L]=Motor_Output_State[Chassic_R]=Motor_Output_State[Fric_1]=Motor_Output_State[Fric_2]=1;
						Shoot_Speed_Pid_Calc(Firc_Speed);	//摩擦轮
						Shoot_Ctrl=0;
						++Cartridge_TIM_cnt;
						if(Cartridge_TIM_cnt>400)
						{
							Cartridge_TIM_cnt=0;
							Shoot_Ctrl=1;
						}
					}
					break;
					
					case 2:	//无
					{
						Shoot_Ctrl=0;
						Chassic_State=0;
						Shoot_Speed_Pid_Calc(0);	//摩擦轮
					}
					break;
				}
			}
		}
		break;
		
		case 2:	//无控制
		{
			read_allow = 0;
			remote_control_allow = 0;
			sotf_start = 1;
			Shoot_Speed_Pid_Calc(0);	//摩擦轮
			Cartridge_wheel_PID_Calc(0);	//拨弹
			Shoot_Ctrl=0;
			Chassic_State=0;
		}
		break;
		
		default:
			break;
	}

	#ifdef Classic_Move
		uint8_t Moto_ID[2]={Chassic_L,Chassic_R};
		if(Motor_Output_State[Chassic_L]==1 && Motor_Output_State[Chassic_R]==1)
		{
			if(Classic_First_Start)
			{
				Classic_First_Start=0;
				if(direction==0)
					direction=1;
			}
			motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,Classic_Move_Speed);
			for (uint8_t i=0; i<2; i++)
			{
				motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm); //根据设定值进行PID计算。
				Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
			}
		}
		else
		{
			Classic_First_Start=1;
			direction=0;
			motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,0);
			for(uint8_t i=0; i<2; ++i)
			{
				motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
				Motor_Output[ Moto_ID[i] ]=0;
			}
		}
	#endif
	
	if(Motor_Output_State[Gimbal_Y]==1)
		Motor_Output[Gimbal_Y]=yaw;
	
	if(Motor_Output_State[Gimbal_P]==1)
		Motor_Output[Gimbal_P]=pitch;
	
	if(Motor_Output_State[Fric_1]==1)
		Motor_Output[Fric_1]=Fric_wheel[0].output;
	
	if(Motor_Output_State[Fric_2]==1)
		Motor_Output[Fric_2]=Fric_wheel[1].output;
	
	uint8_t Switch_State=0;	//发送微动开关状态
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_RESET)
		Switch_State=0;
	else
		Switch_State=1;
	
	uint8_t Chassic_Data[3]={Chassic_State,Shoot_Ctrl,Switch_State};
	Chassic_Ctrl(Chassic_Data,3);
	CAN_Motor_Ctrl(&hcan1,Motor_Output);
	for(uint8_t i=0; i<12; ++i)
		Motor_Output[i]=Motor_Output_State[i]=0;
	
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
