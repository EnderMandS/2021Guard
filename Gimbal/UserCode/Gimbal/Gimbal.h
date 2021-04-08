#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "main.h"
#include "stdbool.h"

#define Motor_Ecd_to_Ang 0.0439506775729459f // 360度/8191
#define yaw_center 301.5f
#define pitch_center ((Pitch_Limit_Top+Pitch_Limit_Bottom)/2.f)

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

void Gimbal_Sotf_Start(void);
void Gimbal_Remote_Control(void);
void Gimbal_Automatic_control(void);
void Gimbal_Inspect(void);
float Yaw_Motor_Angle_Change(void);
float Limit_Zero_To_360(float Input);
bool Set_Pitch_Zero_Point(void);
void Avoid_Wall(void);
bool Yaw_At_Border(void);

extern void Gimbal_Automatic_target(float _pitch,float _yaw);
extern void Gimbal_Automatic_target_lost(void);
extern void Gimbal_Inspect_setSpeed(int speed);

#endif
