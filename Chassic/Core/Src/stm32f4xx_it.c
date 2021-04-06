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
		
//		Power_Heat_Cheak();	//功率&热量超出检测
		
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
						--eliminate_dithering_left;
						if(eliminate_dithering_left==0)
							Changing_Speed_Flag=0;
					}
				}
				else if(direction==touch_Right)
				{
					if(HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_SET)
					{
						--eliminate_dithering_right;
						if(eliminate_dithering_right==0)
							Changing_Speed_Flag=0;
					}
				}
			}
		#endif
		
		Check_Being_Hit();	//被击打改变速度方向检测
		
		if(Move_Allow==1)
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
		}
		else
		{
			motor_pid[0].target=motor_pid[1].target=Slow_Change_Speed(direction,0);
			for(uint8_t i=0; i<2; ++i)
			{
				motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
				Motor_Output[ Moto_ID[i] ]=0;
			}
		}
		
		Heat_Rest=320-PowerHeatData.shooter_id1_17mm_cooling_heat;	//剩余热量
		if(Heat_Rest>200)	//剩余热量多，高射频消耗热量
			Shoot_Ultra_Mode=1;
		else if(Heat_Rest<100)
			Shoot_Ultra_Mode=0;
		
//		Shoot_State=0;
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
					Cartridge_wheel_PID_Calc(Cartridge_Ultra+Cartridge_Speed_Offset);
				else if(Heat_Rest>50)
					Cartridge_wheel_PID_Calc(Cartridge_Fast+Cartridge_Speed_Offset);
				else
					Cartridge_wheel_PID_Calc(Cartridge_Slow+Cartridge_Speed_Offset);
				Motor_Output[Cartridge]=Cartridge_wheel.output;
			break;
			
			case 3:		//Slow
				if(Heat_Rest>20)
				{
					Cartridge_wheel_PID_Calc(Cartridge_Slow+Cartridge_Speed_Offset);
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
		
		CAN_Motor_Ctrl(&hcan2,Motor_Output);
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
	uint8_t Data[8]={0};
	memcpy(Data,&Fric_Speed,4);
	Data[4]=is_red_or_blue();
	CAN_Send_Gimbal(&hcan1,Data,5);
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
