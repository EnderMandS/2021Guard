/*
 * @Descripttion: 包含云台的各种模式
 * @version: V1.0
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-10-26 20:36:48
 */
#include "Gimbal.h"
#include "caninfo.h"
#include "Remote_control.h"
#include "Filter.h"
#include "pid.h"
#include <math.h>
#include "usartinfo.h"

#define Pitch_Limit_Top			131.f
#define Pitch_Limit_Bottom	72.5f

#define Inspect_Empty				3
#define Pitch_Inspect_Speed	0.3f
#define Yaw_Inspect_Speed		0.3f

int32_t pitch, yaw;
float yaw_angle;
float pitch_angle;
int sotf_start = 1;
int remote_control_allow = 0;
int read_allow = 0;
int Pitch_dir=1;

//循环限幅函数
float	loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

/**
 * @brief: 云台缓启动
 * @param {*}
 * @retval: 
 * @attention: 
 */
void Gimbal_Sotf_Start(void)
{
    if (sotf_start == 1)
    {
        if (read_allow == 0)
        {
            pitch_angle = gear_motor_data[Gimbal_P].angle * Motor_Ecd_to_Ang;
            yaw_angle = Yaw_Motor_Angle_Change();
            read_allow = 1;
        }
        if (pitch_angle < pitch_center)
        {
            pitch_angle += 0.05f;
        }
        else if (pitch_angle > pitch_center)
        {
            pitch_angle -= 0.05f;
        }
        if ((pitch_angle < pitch_center + 1 && pitch_angle > pitch_center - 1) )
        {
            remote_control_allow = 1;
            sotf_start = 0;
        }
    }
}

/**
 * @brief: 云台PID控制
 * @param {*}
 * @retval: 
 * @attention: 
 */
void Gimbal_Remote_Control(void)
{
    if (remote_control_allow == 1)
    {
        pitch_angle += 0.0005f * first_order_filter_Y_cali(remote_control.ch4);
        yaw_angle += 0.001f * first_order_filter_X_cali(remote_control.ch3);
    }
		yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
    yaw = Control_YawPID(yaw_angle);
    Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top);
    pitch = Control_PitchPID(pitch_angle);
}

void Gimbal_Automatic_control(void)
{
	if(view_send_state==3)	//1:None	2:Send angle	3:View
	{
    yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
    yaw = Control_YawPID(yaw_angle);
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top);
    pitch = Control_PitchPID(pitch_angle);
	}
	else
		Gimbal_Inspect();
}

void Gimbal_Inspect(void)	//巡检
{
	if(Pitch_dir==1)
		if(pitch_angle>=Pitch_Limit_Top-Inspect_Empty)
			Pitch_dir=-1;
		
	if(Pitch_dir==-1)
		if(pitch_angle<=Pitch_Limit_Bottom+Inspect_Empty)
			Pitch_dir=1;
		
	pitch_angle += Pitch_dir*Pitch_Inspect_Speed;
	yaw_angle += Yaw_Inspect_Speed;
	yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
	yaw = Control_YawPID(yaw_angle);
	pitch = Control_PitchPID(pitch_angle);
}

float Yaw_Motor_Angle_Change(void)
{
	float Back=0;
	int Round=gear_motor_data[Gimbal_Y].round_cnt%2;
	if( Round==0 )
		Back=gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang /2.f;
	else if( Round==1 || Round==-1 )
		Back=gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang /2.f + 180.f;
	return Back;
}
