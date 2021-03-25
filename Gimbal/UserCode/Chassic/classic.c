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
float Slow_Change_Speed(int dir, uint16_t Speed)
{
	static uint16_t Time_Cnt=0;
	static int Last_Dir=0;
	float Back=0;
	if(Last_Dir!=dir)
	{
		++Time_Cnt;
//		if(Time_Cnt<=500)
//			Back = (500.0-(Time_Cnt*1.0)) /500.0 * (Speed*1.0) * (Last_Dir*1.0);
//		else if(Time_Cnt>500 && Time_Cnt<=1000)
//			Back = ((Time_Cnt*1.0)-500.0) /500.0 * (Speed*1.0) * (dir*1.0);
//		else if(Time_Cnt>1000)
//		{
//			Time_Cnt=0;
//			Last_Dir=dir;
//		}
		if(Time_Cnt<=1000)
		{
			Back = cos((Time_Cnt*1.0)/1000.0*PI) * (Speed*1.0);
		}
		else
		{
			Time_Cnt=0;
			Last_Dir=dir;
		}
	}
	else
		Back=dir*Speed;
	
	return Back;
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

