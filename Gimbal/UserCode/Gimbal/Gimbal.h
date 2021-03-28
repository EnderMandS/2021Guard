/*
 * @Descripttion: 云台
 * @version: V1.0
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-10-26 20:37:01
 */
#ifndef _GIMBAL_H
#define _GIMBAL_H
#define Motor_Ecd_to_Ang 0.0439506775729459f // 360度/8192
#define yaw_center 280
#define pitch_center 107
#include "main.h"
extern float yaw_angle;
extern float pitch_angle;
extern int sotf_start;
extern int remote_control_allow;
extern int read_allow;
extern int32_t pitch ,yaw ;
extern float yaw_nowangle;
extern float pit_nowangle;

void Gimbal_Sotf_Start(void);
void Gimbal_Remote_Control(void);
void Gimbal_Automatic_control(void);
void Gimbal_Inspect(void);
float Yaw_Motor_Angle_Change(void);
float Limit_Zero_To_360(float Input);

#endif
