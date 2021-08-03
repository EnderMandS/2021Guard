/*
 * @Descripttion: 视觉串口及裁判系统数据处理已经修改的串口发送函数
 * @version: V1.0
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-10-26 18:04:11
 */
#include "usartinfo.h"
#include "usart.h"
#include "string.h"
#include "Remote_control.h"
#include "judge.h"
#include "Gimbal.h"
#include "buzzer.h"
#include "crc.h"

uint8_t view_send_state;	//视觉串口接收角度标志位
uint8_t View_Buf[VIEW_BUF_LEN];		//视觉串口接收缓冲区
uint8_t rx_view_buf[VIEW_BUF_LEN];	//视觉串口接收区
uint8_t Groy_Data_Buf[GROY_DATA_BUF_LEN];	//陀螺仪缓冲区
uint8_t UART_Buffer[36];	//遥控器串口缓冲区
float eular[3]; //欧拉角 eular[0]==Pitch eular[1]==Roll eular[2]==Yaw 
uint8_t rxbuf[200]={0};
uint8_t Hi229_Update=0;	//陀螺仪数据接收更新标志位
uint8_t view_shoot_mode;	//视觉串口发射标志位
float2uchar pitchangle;
float2uchar yawangle;
float2uchar send_pitch;
float2uchar send_yaw;
uint8_t extern_view_send_state = 0;	//发送角度给NUC标志位
uint8_t Aimming=0;		//判断自瞄或巡检
#define Slow_Time 15	//20ms	检测到目标后云台减速时间	
#define Shot_State_Stay_Time 10	//20ms	拨弹盘状态保持时间	识别可能会丢几帧,拨弹不需要停
uint16_t Shot_Stay_Time=0;	//拨弹状态保持计数变量
uint16_t Slow_Inspect_Speed=Slow_Time;	//云台减速时间变量
uint32_t Gryo_Update_cnt=0;	//陀螺仪更新标志位

/**
  * @brief  串口空闲中断DMA接收回调函数
  * @param  串口通道地址 UART_HandleTypeDef *
  * @retval None
  */

void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		Callback_RC_Handle(&remote_control,UART_Buffer);
	}
	else if (huart == &huart1)
	{
		/*	[0] Head 0xAF
		 *	[1] Control command
		 *	[2] Shoot Mode
		 *	[3]-[6] pitch
		 *	[7]-[10] yaw
		 *	[11] Task Mode
		 *	[12] 风车标志位
		 *	[13] 保留
		 *	[14]-[15] CRC16
		 *	50Hz 20ms
		*/
		memcpy(rx_view_buf, View_Buf, 100); //数据长度rx_view_buf
		
		if (rx_view_buf[0] == 0xAF && Verify_CRC16_Check_Sum(rx_view_buf,16)==true)	//标识符
		{
			switch (rx_view_buf[1]) //命令位
			{
				case 0xAA:
					view_send_state = 1;
					Aimming = 0;	//没有瞄准到
					if(Slow_Inspect_Speed==0)	//清空计数,快速巡检
						Gimbal_Inspect_setSpeed(Gimbal_Inspect_SPEED_FAST);
					else
						--Slow_Inspect_Speed;
					break;
					
				case 0xDD:
					view_send_state = 1;
					Aimming = 0;
					Slow_Inspect_Speed=Slow_Time;	//减速巡检
					Gimbal_Inspect_setSpeed(Gimbal_Inspect_SPEED_SLOW);
					break;
				
				case 0xBB:
					view_send_state = 2;
					extern_view_send_state = 1;	//发送角度数据
					break;
				
				case 0xCC:
					view_send_state = 3;
					Aimming = 1;	//瞄准到
					Slow_Inspect_Speed=Slow_Time;
					if(Shot_Stay_Time!=0)	//持续Shot_State_Stay_Time*20ms 50Hz
						--Shot_Stay_Time;
					
					if(rx_view_buf[2]==0xEE || rx_view_buf[2]==0xFF)	//拨弹保持时间
						Shot_Stay_Time=Shot_State_Stay_Time;
					if(Shot_Stay_Time!=0)
						view_shoot_mode=0xEE;	//high
					else
						view_shoot_mode=0xDD;	//none
					
					break;
				
				default:
					break;
			}

			if(remote_control.switch_right==1 && view_send_state==3 )	//接收角度
			{
				memcpy(pitchangle.c, rx_view_buf + 3, 4);
				memcpy(yawangle.c, rx_view_buf + 7, 4);
				Gimbal_Automatic_target(pitchangle.f, yawangle.f);
			}
		}
	}
	else if (huart == &huart6)					/*串口陀螺仪Hi229*/
	{
		memcpy(&rxbuf,Groy_Data_Buf,200);	//数据长度
		if(rxbuf[0]==0x5A&&rxbuf[1]==0xA5&&rxbuf[6]==0xB0&&rxbuf[13]==0xD0)
		{
			Hi229_Update=1;
			Gryo_Update_cnt=0;
			eular[0] = -((float)(int16_t)(rxbuf[14] + (rxbuf[15]<<8)))/100;	//pitch
			eular[1] = ((float)(int16_t)(rxbuf[16] + (rxbuf[17]<<8)))/100;	//roll
			eular[2] = (((float)(int16_t)(rxbuf[18] + (rxbuf[19]<<8)))/10);	//Yaw轴
		}
	}
}

/**
 * @brief: 串口1发送函数
 * @param {*}
 * @retval: 
 * @attention: 由于HAL库的接收中断会锁住串口，不使用HAL串口发送，本函数是直接访问寄存器
 */
void Uart1_TransmissionT_Data(uint8_t *p_data, uint32_t size)
{

	while (size) 
	{
		WRITE_REG(huart1.Instance->DR, *p_data); //将字节写入DR寄存器
		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != SET) 
		{
		}

		size--;
		p_data++;
	}
}
/**
 * @brief: 串口6发送函数
 * @param {*}
 * @retval: 
 * @attention: 
 */
void Uart6_TransmissionT_Data(uint8_t *p_data, uint32_t size)
{

	while (size)
	{
		WRITE_REG(huart6.Instance->DR, *p_data);
		while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE) != SET)
		{
		}

		size--;
		p_data++;
	}
}
