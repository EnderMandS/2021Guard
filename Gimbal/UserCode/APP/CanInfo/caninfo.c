#include "caninfo.h"
#include "main.h"
#include "can.h"
#include <string.h>
#include "Gimbal.h"
#include "buzzer.h"

gear_moto_measure_t gear_motor_data[12];
int16_t Motor_Output[12]={0};
uint8_t Motor_Output_State[12]={0};
uint8_t Wait_For_Motor_Cnt[12]={0};
uint8_t Wait_For_Motor_State=1;
bool Game_Start=false;
bool Base_Shield=true;
bool Outpost_Alive=true;
extern int Firc_Speed;
extern uint8_t color;
uint32_t Can_Error=0;
bool Shootable=true;
uint8_t Inspect_Position=0;
bool Hit_Gimbal=false;

/**
 * @brief: 数据处理,将接收到数据传入指针并解算
 * @param {void} 
 * @retval: 
 * @attention: 
 */
void get_motor_measure(gear_moto_measure_t *ptr, uint8_t rxd[])
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
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
 * @brief: 减速电机数据处理,将接收到数据传入指针并解算
 * @param {void} 
 * @retval: 
 * @attention: 
 */
void get_gear_motor_measure(gear_moto_measure_t *ptr, uint8_t rxd[])
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
/**
 * @brief: HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
 * @param {void} 
 * @retval: 
 * @attention: 
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if (hcan == &hcan1)
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
					get_gear_motor_measure(&gear_motor_data[rx_header.StdId-Motor_Base], rx_data);
				break;
			
			case 0x1AA:
				color=rx_data[0];
				Hit_Gimbal=rx_data[1];
				Game_Start=rx_data[2];
				Outpost_Alive=rx_data[3];
				Base_Shield=rx_data[4];
				Shootable=rx_data[5];

				Limit_Yaw=Outpost_Alive;	//Outpost not alive, don't limit yaw
			break;
					
			case 0x1BB:
				/*	0 Normal
				 *	1-4 Quadrant
				 *	5 Front
				 *	6 Back
				*/
				Inspect_Position=rx_data[0];
				switch(Inspect_Position)
				{
					case 1:
					case 2:
					case 3:
					case 4:
						Position_Inspect_cnt=Position_Inspect_Time;
					break;
					
					case 5:
					case 6:
						Position_Inspect_cnt=Front_Back_Time;
					break;
					
					default:
						Inspect_Position=Position_Inspect_cnt=0;
					break;
				}
				memcpy(&Inspect_Angle,&rx_data[1],sizeof(float));
				Angle_Stay=Angle_Stay_Time;
				Buzzer_Short(1);
			break;
			
			default:
				break;
		}
	}
}

/**
 * @brief: CAN发送函数
 * @param {void}} 
 * @retval: StdID 0x200为ID1~4（0x201~0x204）；StdID 0x1FF为ID5~8(0x205~0x208)；StdID为 0x2FF为ID9~12(0x209~0x20C)
 * @attention: 
 */
void CAN_Send_CMD(CAN_HandleTypeDef *hcan, int StdID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	CAN_TxHeaderTypeDef can_tx_message;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_message.StdId = StdID;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (motor1 >> 8);
	can_send_data[1] = motor1;
	can_send_data[2] = (motor2 >> 8);
	can_send_data[3] = motor2;
	can_send_data[4] = (motor3 >> 8);
	can_send_data[5] = motor3;
	can_send_data[6] = (motor4 >> 8);
	can_send_data[7] = motor4;
	if(HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_send_data, &send_mail_box)==HAL_ERROR)
		++Can_Error;
}
void CAN_Motor_Ctrl(CAN_HandleTypeDef *hcan, int16_t Motor_Data[12])
{
	CAN_TxHeaderTypeDef can_tx_message;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	uint16_t Std_ID[3]={0x200,0x1FF,0x2FF};
	
	for(uint8_t i=0; i<3; ++i)	//没用到，改成2
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
		if(HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_send_data, &send_mail_box)==HAL_ERROR)
			++Can_Error;
	}
}
