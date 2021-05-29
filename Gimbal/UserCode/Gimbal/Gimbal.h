#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "main.h"
#include "stdbool.h"

#define Pitch_Motor_Zero (121.52448f)
#define Motor_Ecd_to_Ang 0.0439506775729459f // 360度/8191
#define Soft_Start_Up_Error 0.5f
#define pitch_center ((Pitch_Motor_Zero+Pitch_Limit_Bottom)/2.f)

#define Position_Inspect_Time 5
#define Front_Back_Time 2

enum Gimbal_Inspect_SPEED
{
    Gimbal_Inspect_SPEED_SLOW,
    Gimbal_Inspect_SPEED_FAST
};

extern float Pitch_Limit_Top;
extern float Pitch_Limit_Bottom;
extern int sotf_start;
extern int control_allow;
extern int read_allow;
extern int32_t pitch ,yaw ;
extern float yaw_nowangle;
extern float pit_nowangle;
extern bool Pitch_USE_Gyro;
extern bool Limit_Yaw;
extern uint8_t Position_Inspect_cnt;

void Gimbal_Sotf_Start(void);
void Gimbal_Remote_Control(void);
void Gimbal_Automatic_control(void);
void Gimbal_Inspect(void);
float Yaw_Motor_Angle_Change(void);
float Limit_Zero_To_360(float Input);
bool Set_Pitch_Zero_Point(void);
bool Yaw_At_Border(void);
float loop_fp32_constrain(float Input, float minValue, float maxValue);
float Zero_Offset_Cal(void);
bool Gimbal_Keep_Middle(void);
void Gimbal_Position_Inspect(void);

extern void Gimbal_Automatic_target(float _pitch,float _yaw);
extern void Gimbal_Automatic_target_lost(void);
extern void Gimbal_Inspect_setSpeed(int speed);

#endif
