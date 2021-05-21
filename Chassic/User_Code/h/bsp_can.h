#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h"
#include <stdbool.h>

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
extern bool Aim;

void CAN_Filter_Init(void);
void CAN_Motor_Ctrl(CAN_HandleTypeDef *hcan, int16_t Motor_Data[12]);
void CAN_Send_Gimbal(CAN_HandleTypeDef *hcan, uint8_t Data[], uint8_t Len);
void CAN_Send_Gimbal2(CAN_HandleTypeDef *hcan, uint8_t Data[], uint8_t Len);

#endif
