#include "classic.h"
# include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
#include <string.h>
#include <math.h>
#include "can.h"

uint8_t Chassic_State=0;
uint8_t Classic_Ctrl[8]={0};
float set_spd[2];
PID_TypeDef motor_pid[2];
float vx_set,vy_set,wz_set;
uint8_t Classic_First_Start=1;	//∑¿÷πµ◊≈ÃÕ£÷π∫Û÷ÿ∆Ù≥È¥§

void Chassis_init(void)
{
	for(int j=0;j<2;j++)
	{
		pid_init(&motor_pid[j]);
		motor_pid[j].f_param_init(&motor_pid[j],PID_Speed,16384,5000,10,0,6000,0,1.5,0.1,0);  //0.004
/*				   PID_ID id,uint16_t maxOutput,uint16_t integralLimit,float deadband,uint16_t controlPeriod,int16_t max_err,     
			int16_t  target,
			 float kp,
			 float ki,
			 float kd);*/
	}
    set_spd[0] = 0;
    set_spd[1] = 0;
}
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
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}
