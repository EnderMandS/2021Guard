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
//#include "gimbal.h"
int view_send_state;
uint8_t Judgement_Buf[JUDGEMENT_BUF_LEN];
uint8_t View_Buf[VIEW_BUF_LEN];
uint8_t rx_view_buf[24];
uint8_t rx_judge_buf[200];
uint8_t UART_Buffer[36];
uint8_t view_shoot_mode ;
float2uchar pitchangle;
float2uchar yawangle;
float2uchar send_pitch;
float2uchar send_yaw;
frame_judge save_frame_id_message[10] = {0};
int extern_view_send_state = 0;


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
		memcpy(&rx_view_buf, View_Buf, 100); //数据长度rx_view_buf
		if (rx_view_buf[0] == 0xAF && rx_view_buf[11] == 0xFA)	//标识符
		{
			switch(rx_view_buf[1])	//命令位 
			{
				case 0xAA:
					view_send_state = 1;
					break;
				
				case 0xBB:
					view_send_state = 2;
					extern_view_send_state = 1;
					break;
				
				case 0xCC:
					view_send_state = 3;
					view_shoot_mode = rx_view_buf[2];
					break;
				
				default:
					break;
			}
			
//			if (rx_view_buf[1] == 0xAA)//命令位 
//			{
//				view_send_state = 1;
//			}
//			else if (rx_view_buf[1] == 0xBB)
//			{
//				view_send_state = 2;
//				extern_view_send_state = 1;
//			}
//			else if (rx_view_buf[1] == 0xCC)
//			{
//				view_send_state = 3;
//				view_shoot_mode = rx_view_buf[2];
//			}
//				for(int i = 0;i<13;i++)
//				{
//					sum += rx_view_buf[i];
//				}
//				if(rx_view_buf[13] != sum)
//				{
//					return;
//				}
			memcpy(pitchangle.c, rx_view_buf + 3, 4);
			memcpy(yawangle.c, rx_view_buf + 7, 4);
			if (remote_control.switch_right==1 && view_send_state==3 )
			{
					pitch_angle = pitchangle.f;   
					yaw_angle = yawangle.f;   
			}
		}
	}
	//裁判系统串口
	else if (huart == &huart6)
	{
		memcpy(&rx_judge_buf, Judgement_Buf, 200); //数据长度
		Judge_Read_Data(Judgement_Buf);
	}
}


/**
 * @brief: 串口7发送函数
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
 * @brief: 串口8发送函数
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
