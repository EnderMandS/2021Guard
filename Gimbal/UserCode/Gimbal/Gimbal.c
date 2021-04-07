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

#define Inspect_Empty				1
#define Pitch_Inspect_Speed	0.08f
#define Yaw_Inspect_Speed		0.175f

float Pitch_Limit_Top = 121.52448f; //2788	121.52448f
float Pitch_Limit_Bottom = 75.85f;  //61.53f	75.85f

int32_t pitch, yaw;
float yaw_angle;
float pitch_angle;
int sotf_start = 1;
int control_allow = 0;
int read_allow = 0;
int Pitch_dir=1;
uint8_t Set_Zero=0;
bool Set_Zero_Complete=false;

// 自瞄时实际执行的pitch、yaw，云台坐标系
float vision_target_pitch,vision_target_yaw;
// 预测相关
#define PREVIOUS_DET_LENGTH 5
float last_time_target_yaw;
float last_time_target_pitch;
float previous_target_yaw_det[PREVIOUS_DET_LENGTH];
float previous_target_pitch_det[PREVIOUS_DET_LENGTH];
int previous_target_yaw_det_index = 0;
int previous_target_pitch_det_index = 0;
int n;
float yaw_det_average;
float pitch_det_average;

//循环限幅函数
float loop_fp32_constrain(float Input, float minValue, float maxValue)
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
		if(Set_Zero==0)
		{
			if (pitch_angle < Pitch_Limit_Top)
			{
				pitch_angle += 0.05f;
			}
			else if (pitch_angle > Pitch_Limit_Top)
			{
				pitch_angle -= 0.05f;
			}
		}
		if ((pitch_angle < Pitch_Limit_Top + 0.5f && pitch_angle > Pitch_Limit_Top - 0.5f) )
		{
			if(Set_Zero_Complete==false)
			{
				++Set_Zero;
				if(Set_Zero>50)
				{
					if(Set_Pitch_Zero_Point()==true)
					{
						Set_Zero_Complete=true;
						Set_Zero=0;
						control_allow = 1;
						sotf_start = 0;
					}
				}
			}
			else
			{
				control_allow = 1;
				sotf_start = 0;
			}
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
	if (control_allow == 1)
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
	if(control_allow==1)
	{
		if(Aimming==1)
		{
            float pre_yaw;
            vision_target_yaw += yaw_det_average * 50.0f * 0.0025f;
            pre_yaw = vision_target_yaw + (yaw_det_average * 50.0f * 0.12f);
			yaw_angle = loop_fp32_constrain(pre_yaw,0,360);
			yaw = Control_YawPID(yaw_angle);
            pitch_angle = vision_target_pitch;
			Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top);
			pitch = Control_PitchPID(pitch_angle);
		}
		else
			Gimbal_Inspect();
	}
}

/**
 * @brief: 更改自瞄目标，当串口接收到新目标时运行这个函数
 * @param {*}
 * @retval: 
 * @attention: 输入为视觉坐标系（-180°~180°）
 */
void Gimbal_Automatic_target(float _pitch,float _yaw){
    float det_yaw = _yaw - last_time_target_yaw;

    if ( det_yaw > 180.0f){
        det_yaw = 360.0f - det_yaw;
    } else if (det_yaw < -180.0f){
        det_yaw = -360.0f - det_yaw;
    }

    if( fabs(det_yaw) > 1.5){
        det_yaw = 0;
    }

    previous_target_yaw_det[previous_target_yaw_det_index] = det_yaw;
    if (previous_target_yaw_det_index < PREVIOUS_DET_LENGTH)
        previous_target_yaw_det_index++;
    else
        previous_target_yaw_det_index = 0;

    if(++n > PREVIOUS_DET_LENGTH){
        n = PREVIOUS_DET_LENGTH;
    }

    float sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += previous_target_yaw_det[i];
    }

    yaw_det_average = sum / n;

    last_time_target_yaw = _yaw;
    // 视觉坐标系转云台坐标系
    vision_target_pitch = _pitch + Pitch_Limit_Top;
    vision_target_yaw = _yaw;
}

void Gimbal_Automatic_target_lost(){
    last_time_target_yaw = 0.0;
    previous_target_yaw_det_index = 0;
    n = 0;
    yaw_det_average = 0.0;
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

#define Sampling_Times 5	//采样次数
#define Max_Error 0.5f	//最大允许误差，超过误差重新采样
float Gyto_Average=0;
float Motor_Average=0;
int cnt=Sampling_Times;
float Pitch_Gyro_Buf[Sampling_Times]={0};
float Pitch_Motor_Buf[Sampling_Times]={0};
float Pitch_Gyro_Max=-1;
float Pitch_Gyro_Min=361;
bool Set_Pitch_Zero_Point(void)	//采样取平均确定Pitch零点
{
	if(Hi229_Update==1)
	{
		Hi229_Update=0;
		//Update data
		Pitch_Gyro_Buf[cnt]=eular[0];
		Pitch_Motor_Buf[cnt]=gear_motor_data[Gimbal_P].angle;
		//Calculate Max&Min
		if(Pitch_Gyro_Buf[cnt]>Pitch_Gyro_Max)
			Pitch_Gyro_Max=Pitch_Gyro_Buf[cnt];
		if(Pitch_Gyro_Buf[cnt]<Pitch_Gyro_Min)
			Pitch_Gyro_Min=Pitch_Gyro_Buf[cnt];
		
		--cnt;
		
		if(cnt<0)
		{
			if(Pitch_Gyro_Max-Pitch_Gyro_Min>Max_Error)	//超过最大允许误差
			{
				Pitch_Gyro_Max=-1;
				Pitch_Gyro_Min=361;
				for(; cnt<Sampling_Times; ++cnt)
					Pitch_Gyro_Buf[cnt]=Pitch_Motor_Buf[cnt]=0;
				return false;
			}
			
			for(cnt=0; cnt<Sampling_Times; ++cnt)
			{
				Gyto_Average+=Pitch_Gyro_Buf[cnt];
				Motor_Average+=Pitch_Motor_Buf[cnt];
			}
			Gyto_Average/=(Sampling_Times*1.f);		//向下递增
			Motor_Average/=(Sampling_Times*1.f);	//向下递减
			Motor_Average*=Motor_Ecd_to_Ang;
			Pitch_Limit_Top+=Gyto_Average+Motor_Average-Pitch_Limit_Top;	//+偏移
			return true;
		}
	}
	return false;
}
