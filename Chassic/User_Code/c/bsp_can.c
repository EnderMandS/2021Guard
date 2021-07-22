#include "bsp_can.h"
#include "can.h"
#include "classic.h"
#include "shoot.h"
#include "bullet.h"
#include "judge.h"
#include "bsp_judge.h"
#include <string.h>

#define ABS(x) ((x > 0) ? x : -x)	//绝对值

gear_moto_measure_t gear_motor_data[12];	//电机数据结构体数组
int16_t Motor_Output[12]={0};	//电机输出暂存数组
extern uint32_t Time_Tick;	//定时器中自增,can接收函数清空,用于判断云台有没有掉线

bool Aim=false;	//云台接收的数据,是否瞄准到了目标

void CAN_Filter_Init(void)	//can过滤器初始化
{
  CAN_FilterTypeDef CAN_Filter_STM;
	CAN_Filter_STM.FilterActivation=ENABLE;
	CAN_Filter_STM.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_Filter_STM.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_Filter_STM.FilterIdHigh = 0x0000;
  CAN_Filter_STM.FilterIdLow = 0x0000;
  CAN_Filter_STM.FilterMaskIdHigh = 0x0000;
  CAN_Filter_STM.FilterMaskIdLow = 0x0000;
  CAN_Filter_STM.FilterBank = 0;
  CAN_Filter_STM.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_STM);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CAN_Filter_STM.SlaveStartFilterBank = 14;
  CAN_Filter_STM.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_STM);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void get_gear_motor_measure(gear_moto_measure_t *ptr, uint8_t rxd[])	//can接收数据写入电机结构体,并计算圈数
{
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(rxd[0] << 8 | rxd[1]);
    ptr->speed_rpm = (int16_t)(rxd[2] << 8 | rxd[3]);
    ptr->real_current = (uint16_t)(rxd[4] << 8 | rxd[5]);
    ptr->temperate = rxd[6];
    if (ptr->angle - ptr->last_angle > 4096)
    {
        ptr->round_cnt--;
    }
    else if (ptr->angle - ptr->last_angle < -4096)
    {
        ptr->round_cnt++;
    }
		int res1, res2, delta;
		if(ptr->angle < ptr->last_angle)
		{			//可能的情况
			res1 = ptr->angle + 8192 - ptr->last_angle;	//正转，delta=+
			res2 = ptr->angle - ptr->last_angle;				//反转	delta=-
		}
		else
		{	//angle > last
			res1 = ptr->angle - 8192 - ptr->last_angle ;//反转	delta -
			res2 = ptr->angle - ptr->last_angle;				//正转	delta +
		}
	//不管正反转，肯定是转的角度小的那个是真的
		if(ABS(res1)<ABS(res2))
			delta = res1;
		else
			delta = res2;

	ptr->total_angle += delta;
	(ptr)->real_total_angle = ((((ptr)->total_angle)%(36*8192))*360)/(8192*36);	
}
void Gimbal_Receive(uint8_t Receive_Data[8])	//can云台接收数据转换函数
{
	Time_Tick=0;	//清空计数,表示从云台到了数据

	Move_Allow=Receive_Data[0];	//移动状态	0:禁止	1:正常	2:获取轨道长度
	
	switch( Receive_Data[1] )	//Shoot control
	{
		case 0x00:	//Stop
		case 0x01:	//Single
		case 0x02:	//Fast
		case 0x03:	//Slow
			Shoot_State=Receive_Data[1];
			break;
		
		default:
			Shoot_State=0;
			break;
	}
	
	Switch_State[1]=Receive_Data[2];	//微动开关状态,没用到,保留代码
	
	switch(Receive_Data[3])	//瞄准状态 0:Inspect  1:Aim
	{
		case 0:	//没有瞄到,正常巡检
			Aim=false;
			if(Classic_Move_Speed!=Chassic_Spring_Fast)
				Classic_Move_Speed=Chassic_Spring_Middle;	//设置底盘速度为Middle
		break;
			
		case 1:
			if((is_red_or_blue()==BLUE && GameRobotHP.blue_7_robot_HP>250) ||	//瞄准到了,血量足够则减速,提高瞄准精确度
					(is_red_or_blue()==RED && GameRobotHP.red_7_robot_HP>250))	//rest HP low, don't slow down
			{
				Aim=true;
				Classic_Move_Speed=Chassic_Spring_Slow;	//设置底盘速度为Slow
			}
			else	//血量不足不减速
			{
				Aim=false;
				if(Classic_Move_Speed!=Chassic_Spring_Fast)
					Classic_Move_Speed=Chassic_Spring_Middle;
			}
		break;
		
		default:
			break;
	}
	
	Gimbal_Position=Receive_Data[4];	//云台巡检状态,0为正常,1-4为象限,5前,6后,在1-4象限为云台手控制,5-6为装甲板击打更改的巡检方向
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)	//can接收回调函数
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8]={0};
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if (hcan == &hcan2)
	{
		switch (rx_header.StdId)
		{
			case 0x201:
			case 0x202:
			case 0x203:
			case 0x204:
			case 0x205:
			case 0x206:
			case 0x207:
			case 0x208:
			case 0x209:
			case 0x20A:
			case 0x20B:
					get_gear_motor_measure(&gear_motor_data[rx_header.StdId-Motor_Base], rx_data);	//can接收数据写入电机结构体
				break;
			
		default:
				break;
		}
	}
	else if (hcan == &hcan1)
	{
		if(rx_header.StdId==0x101)
			Gimbal_Receive(rx_data);	//云台接收数据
	}
}
void CAN_Motor_Ctrl(CAN_HandleTypeDef *hcan, int16_t Motor_Data[12])	//can发送函数
{
	CAN_TxHeaderTypeDef can_tx_message;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	uint16_t Std_ID[3]={0x200,0x1FF,0x2FF};	
	
	for(uint8_t i=0; i<1; ++i)	
	{
		can_tx_message.StdId = Std_ID[i];
		can_send_data[0] = Motor_Data[4*i] >> 8;
		can_send_data[1] = Motor_Data[4*i];
		can_send_data[2] = Motor_Data[4*i+1] >> 8;
		can_send_data[3] = Motor_Data[4*i+1];
		can_send_data[4] = Motor_Data[4*i+2] >> 8;
		can_send_data[5] = Motor_Data[4*i+2];
		can_send_data[6] = Motor_Data[4*i+3] >> 8;
		can_send_data[7] = Motor_Data[4*i+3];
		HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_send_data, &send_mail_box);
	}
}
void CAN_Send_Gimbal(CAN_HandleTypeDef *hcan, uint8_t Data[], uint8_t Len)	//发送给云台 ID:1AA
{
	CAN_TxHeaderTypeDef can_tx_message;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = Len;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_message.StdId = 0x1AA;
	memcpy(can_send_data,Data,Len);
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}
HAL_StatusTypeDef CAN_Send_Gimbal2(CAN_HandleTypeDef *hcan, uint8_t Data[], uint8_t Len)	//发送给云台 ID:1BB
{
	CAN_TxHeaderTypeDef can_tx_message;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = Len;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_message.StdId = 0x1BB;
	memcpy(can_send_data,Data,Len);
	return HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}
