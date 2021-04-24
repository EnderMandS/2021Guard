/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"
#include "bsp_can.h"
#include "usart.h"
#include <math.h>
#include "caninfo.h"
#include "Gimbal.h"
#include "usartinfo.h"

#define ABS(x) ((x > 0) ? x : -x)
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
														PID_TypeDef *pid,
														PID_ID id,
														uint16_t maxout,
														uint16_t intergral_limit,
														float deadband,
														uint16_t period,
														int16_t max_err,
														int16_t target,

														float kp,
														float ki,
														float kd)
{
	pid->id = id;

	pid->ControlPeriod = period; //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->output = 0;
}

/*中途更改参数设定--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid计算-----------------------------------------------------------------------*/

static float pid_calculate(PID_TypeDef *pid, float measure) //, int16_t target)
{
	//	uint32_t time,lasttime;

	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime - pid->lasttime;
	pid->measure = measure;
	//	pid->target = target;

	pid->last_err = pid->err;
	pid->last_output = pid->output;

	pid->err = pid->target - pid->measure;

	//是否进入死区
	if ((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);

		pid->dout = pid->kd * (pid->err - pid->last_err);

		//积分是否超出限制
		if (pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if (pid->iout < -pid->IntegralLimit)
			pid->iout = -pid->IntegralLimit;

		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
		if (pid->output > pid->MaxOutput)
		{
			pid->output = pid->MaxOutput;
		}
		if (pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	}

	return pid->output;
}

/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PID_TypeDef *pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

//imu.wz是Yaw的速度，imu.wx是Pitch的速度，imu.wy是rol的速度

//#define USE_Gryo_PID
#ifdef USE_Gryo_PID
	float PitchOPIDP = 2.5;
	float PitchOPIDI = 0.03;
	float PitchOPIDD = 5;

	float PitchOPIDCurrentError = 0;
	float PitchOPIDLastError = 0;
	float PitchOPIDIMax = 30;
	float PitchOPIDPIDMax = 30000;
	float PitchOPIDPout;
	float PitchOPIDIout;
	float PitchOPIDDout;
	float PitchOPIDPIDout;

	float PitchIPIDP = 250;
	float PitchIPIDI = 0;
	float PitchIPIDD = 10;

	float PitchIPIDCurrentError = 0;
	float PitchIPIDLastError = 0;
	float PitchIPIDIMax = 0;
	float PitchIPIDPIDMax = 30000;
	float PitchIPIDPout, PitchIPIDIout, PitchIPIDDout;
	float PitchIPIDPIDout;
	float PitchCurrentTick, PitchLastTick;
#else
	float PitchOPIDP = 5;
	float PitchOPIDI = 0.03;
	float PitchOPIDD = 10;

	float PitchOPIDCurrentError = 0;
	float PitchOPIDLastError = 0;
	float PitchOPIDIMax = 30;
	float PitchOPIDPIDMax = 30000;
	float PitchOPIDPout;
	float PitchOPIDIout;
	float PitchOPIDDout;
	float PitchOPIDPIDout;

	float PitchIPIDP = 350;
	float PitchIPIDI = 0;
	float PitchIPIDD = 10;

	float PitchIPIDCurrentError = 0;
	float PitchIPIDLastError = 0;
	float PitchIPIDIMax = 0;
	float PitchIPIDPIDMax = 30000;
	float PitchIPIDPout, PitchIPIDIout, PitchIPIDDout;
	float PitchIPIDPIDout;
	float PitchCurrentTick, PitchLastTick;
#endif

float YawOPIDP = 200;
float YawOPIDI = 0;
float YawOPIDD = 0;
float YawOPIDCurrentError = 0;
float YawOPIDLastError = 0;
float YawOPIDLastTick = 0;
float YawOPIDIMax = 0;
float YawOPIDPIDMax = 500;
float YawOPIDPout, YawOPIDIout, YawOPIDDout;
float YawOPIDPIDout;

float YawIPIDP = 500;
float YawIPIDI = 0;
float YawIPIDD = 0;
float YawIPIDCurrentError = 0;
float YawIPIDLastError = 0;
float YawIPIDLastTick = 0;
float YawIPIDIMax = 0;
float YawIPIDPIDMax = 30000;
float YawIPIDPout, YawIPIDIout, YawIPIDDout;
float YawIPIDPIDout;
float YawCurrentTick, YawLastTick;
float Control_YawPID(float Target)
{
	/***************************************	外环角度环	******************************************/
	float Angle_now = Yaw_Motor_Angle_Change();

	YawOPIDCurrentError = (Target - Angle_now) ;
	if(fabs(YawOPIDCurrentError)>180)
	{
		if(YawOPIDCurrentError>0)
		{
			YawOPIDCurrentError = -(360-fabs(YawOPIDCurrentError));
		}
		else
		{
			YawOPIDCurrentError = (360-fabs(YawOPIDCurrentError));
		}
	}
	YawOPIDCurrentError= YawOPIDCurrentError* Motor_Ang_to_Rad;
	YawOPIDPout = YawOPIDP * YawOPIDCurrentError;
	YawOPIDIout += YawOPIDI * YawOPIDCurrentError;
	LimitMax(YawOPIDIout, YawOPIDIMax)
		YawOPIDDout = -YawOPIDD * (gear_motor_data[Gimbal_Y].speed_rpm * 2 * PI / 60);
	YawOPIDPIDout = YawOPIDPout + YawOPIDIout + YawOPIDDout;
	LimitMax(YawOPIDPIDout, YawOPIDPIDMax)
		YawOPIDLastError = YawOPIDCurrentError;
	/***************************************	内环速度环	******************************************/
	YawIPIDCurrentError = YawOPIDPIDout - (gear_motor_data[Gimbal_Y].speed_rpm * 2 * PI / 60); //YawOPIDPIDout
	YawIPIDPout = YawIPIDP * YawIPIDCurrentError;
	YawIPIDIout += YawIPIDI * YawIPIDCurrentError;
	LimitMax(YawIPIDIout, YawIPIDIMax)
		YawIPIDDout = YawIPIDD * (YawIPIDLastError - YawIPIDCurrentError);
	YawIPIDPIDout = (YawIPIDPout + YawIPIDIout + YawIPIDDout);
	LimitMax(YawIPIDPIDout, YawIPIDPIDMax)
		YawIPIDLastError = YawIPIDCurrentError;
	return YawIPIDPIDout;
}

float Control_PitchPID(float Target)
{
	/***************************************	外环角度环	******************************************/
	if(Pitch_USE_Gyro==true)
	{
		PitchOPIDCurrentError = Target - eular[0];
	}
	else
	{
		PitchOPIDCurrentError = Target - pit_nowangle;
	}
	if(fabs(PitchOPIDCurrentError)>180)
	{
		if(PitchOPIDCurrentError>0)
		{
			PitchOPIDCurrentError = -(360-fabs(PitchOPIDCurrentError));
		}
		else
		{
			PitchOPIDCurrentError = (360-fabs(PitchOPIDCurrentError));
		}
	}
	PitchOPIDPout = PitchOPIDP * PitchOPIDCurrentError;
	PitchOPIDIout += PitchOPIDI * PitchOPIDCurrentError;
	LimitMax(PitchOPIDIout, PitchOPIDIMax);
	PitchOPIDDout = -PitchOPIDD * (gear_motor_data[Gimbal_P].speed_rpm * 2 * PI / 60);
	PitchOPIDPIDout = PitchOPIDPout + PitchOPIDIout + PitchOPIDDout;
	LimitMax(PitchOPIDPIDout, PitchOPIDPIDMax)
		PitchOPIDLastError = PitchOPIDCurrentError;

	/***************************************	内环速度环	******************************************/
	PitchIPIDCurrentError = PitchOPIDPIDout - (gear_motor_data[Gimbal_P].speed_rpm * 2 * PI / 60);
	PitchIPIDPout = PitchIPIDP * PitchIPIDCurrentError;
	PitchIPIDIout += PitchIPIDI * PitchIPIDCurrentError;
	LimitMax(PitchIPIDIout, PitchIPIDIMax)

		PitchIPIDDout = PitchIPIDD * (PitchIPIDLastError - PitchIPIDCurrentError);
	PitchIPIDPIDout = (PitchIPIDPout + PitchIPIDIout + PitchIPIDDout);
	LimitMax(PitchIPIDPIDout, PitchIPIDPIDMax)

		PitchIPIDLastError = PitchIPIDCurrentError;
	PitchLastTick = PitchCurrentTick;
	return PitchIPIDPIDout;
}
