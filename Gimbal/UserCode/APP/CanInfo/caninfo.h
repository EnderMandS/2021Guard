/*
 * @Descripttion: Can数据处理
 * @version: V1。0
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-10-26 14:34:04
 */
#ifndef _CANINFO_H
#define _CANINFO_H

#include "main.h"
#include <stdbool.h>

#define Motor_Base 0x201
// 各电机ID分配
typedef enum
{
	Gimbal_Yaw_6020_ID = 0x206,
	Gimbal_Pitch_6020_ID = 0x208,
	Fric_wheel_1=0x201,
	Fric_wheel_2=0x202
}CAN_Message_ID;

//motor_data分配
typedef enum
{
    Gimbal_Y  = Gimbal_Yaw_6020_ID		-Motor_Base,
    Gimbal_P  = Gimbal_Pitch_6020_ID	-Motor_Base,
		Fric_1		= Fric_wheel_1					-Motor_Base,
		Fric_2		= Fric_wheel_2					-Motor_Base
}Motor_Data_ID;

//减速电机反馈信息
//电机反馈信息
typedef struct
{
    uint16_t angle;
    int16_t speed_rpm;
    int16_t real_current;
    uint8_t temperate;
    int16_t last_angle;
    int32_t round_cnt;
		int32_t	total_angle;
		float   real_total_angle;
}gear_moto_measure_t;

extern gear_moto_measure_t gear_motor_data[12];
extern int16_t Motor_Output[12];
extern uint8_t Motor_Output_State[12];
extern bool Game_Start;
extern bool Base_Shield;
extern bool Outpost_Alive;
extern uint32_t Can_Error;
extern bool Shootable;
extern uint8_t Inspect_Position;
extern bool Hit_Gimbal;

void get_gear_motor_measure(gear_moto_measure_t *ptr, uint8_t rxd[]);
void get_motor_measure(gear_moto_measure_t *ptr, uint8_t rxd[]);
void CAN_Send_CMD(CAN_HandleTypeDef *hcan, int StdID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_Motor_Ctrl(CAN_HandleTypeDef *hcan, int16_t Motor_Data[12]);

#endif
