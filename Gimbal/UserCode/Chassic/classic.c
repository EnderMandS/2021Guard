/*
 * @Author: M
 * @Date: 2021-04-11 09:56:15
 * @LastEditTime: 2021-08-09 15:23:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMe:\RM\Guard\NewGuard\Gimbal\UserCode\Chassic\classic.c
 */
#include "classic.h"
# include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
#include <string.h>
#include <math.h>
#include "can.h"
#include "caninfo.h"

uint8_t Chassic_State=0;

/**
 * @description: 云台给底盘发送数据 StdID:0x101
 * @param {uint8_t} *Data	数据指针
 * @param {uint8_t} Len	长度
 * @return {*}
 */
void Chassic_Ctrl(uint8_t *Data, uint8_t Len)
{
	CAN_TxHeaderTypeDef can_tx_message;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = Len;
	uint8_t can_send_data[8]={0};
	uint32_t send_mail_box;
	can_tx_message.StdId=0x101;
	memcpy(can_send_data,Data,Len);
	if(HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box)==HAL_ERROR)
		++Can_Error;
}
