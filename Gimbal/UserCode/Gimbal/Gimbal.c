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
int32_t pitch, yaw;
float yaw_angle;
float pitch_angle;
int sotf_start = 1;
int remote_control_allow = 0;
int read_allow = 0;

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
            yaw_angle = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang;
            read_allow = 1;
        }
        if (pitch_angle < pitch_center)
        {
            pitch_angle += 0.01f;
        }
        else if (pitch_angle > pitch_center)
        {
            pitch_angle -= 0.01f;
        }
//        if (yaw_angle < yaw_center)
//        {
//            yaw_angle += 0.01f;
//        }
//        else if (yaw_angle > yaw_center)
//        {
//            yaw_angle -= 0.01f;
//        }
        if ((pitch_angle < pitch_center + 1 && pitch_angle > pitch_center - 1) ) //&& (yaw_angle < yaw_center + 1 && yaw_angle > yaw_center - 1))
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
        yaw_angle += -0.001f * first_order_filter_X_cali(remote_control.ch3);
    }
    yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
    yaw = Control_YawPID(yaw_angle);
    Limit(pitch_angle, 48, 131);		//48.5	131.5
    pitch = Control_PitchPID(pitch_angle);
}

/**
 * @brief: 自瞄控制
 * @param {*}
 * @retval: 
 * @attention: 
 */
void Gimbal_Automatic_control(void)
{
    yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
    yaw = Control_YawPID(yaw_angle);
		Limit(pitch_angle, 48, 131);		//1100 3000
    pitch = Control_PitchPID(pitch_angle);
}
