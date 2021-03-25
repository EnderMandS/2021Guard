/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
#define ABS(x) ((x > 0) ? x : -x)
/*������ʼ��--------------------------------------------------------------*/
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

	pid->ControlPeriod = period; //û�õ�
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

/*��;���Ĳ����趨--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid����-----------------------------------------------------------------------*/

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

	//�Ƿ��������
	if ((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);

		pid->dout = pid->kd * (pid->err - pid->last_err);

		//�����Ƿ񳬳�����
		if (pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if (pid->iout < -pid->IntegralLimit)
			pid->iout = -pid->IntegralLimit;

		//pid�����
		pid->output = pid->pout + pid->iout + pid->dout;

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
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

/*pid�ṹ���ʼ����ÿһ��pid������Ҫ����һ��-----------------------------------------------------*/
void pid_init(PID_TypeDef *pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

//imu.wz��Yaw���ٶȣ�imu.wx��Pitch���ٶȣ�imu.wy��rol���ٶ�

float PitchOPIDP = 45;	  //55
float PitchOPIDI = 0;
float PitchOPIDD = 65;	  //45
float PitchOPIDCurrentError = 0;
float PitchOPIDLastError = 0;
float PitchOPIDIMax = 0;
float PitchOPIDPIDMax = 15000; //500
float PitchOPIDPout, PitchOPIDIout, PitchOPIDDout;
float PitchOPIDPIDout;

float PitchIPIDP = 35; //40
float PitchIPIDI = 0;
float PitchIPIDD = 5; //5
float PitchIPIDCurrentError = 0;
float PitchIPIDLastError = 0;
float PitchIPIDIMax = 0;
float PitchIPIDPIDMax = 30000; //3000
float PitchIPIDPout, PitchIPIDIout, PitchIPIDDout;
float PitchIPIDPIDout;
float PitchCurrentTick, PitchLastTick;

float YawOPIDP = 30; //15
float YawOPIDI = 0;
float YawOPIDD = 0;
float YawOPIDCurrentError = 0;
float YawOPIDLastError = 0;
float YawOPIDLastTick = 0;
float YawOPIDIMax = 0;
float YawOPIDPIDMax = 500; //300
float YawOPIDPout, YawOPIDIout, YawOPIDDout;
float YawOPIDPIDout;

float YawIPIDP = 1800;
float YawIPIDI = 2;
float YawIPIDD = -300;
float YawIPIDCurrentError = 0;
float YawIPIDLastError = 0;
float YawIPIDLastTick = 0;
float YawIPIDIMax = 0;
float YawIPIDPIDMax = 24650; //3000
float YawIPIDPout, YawIPIDIout, YawIPIDDout;
float YawIPIDPIDout;
float YawCurrentTick, YawLastTick;
float Control_YawPID(float Target)
{
	/***************************************	�⻷�ǶȻ�	******************************************/
	float Angle_now = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang;

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
	/***************************************	�ڻ��ٶȻ�	******************************************/
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
	/***************************************	�⻷�ǶȻ�	******************************************/
	PitchOPIDCurrentError = Target - gear_motor_data[Gimbal_P].angle * Motor_Ecd_to_Ang;
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
	LimitMax(PitchOPIDIout, PitchOPIDIMax)
		PitchOPIDDout = -PitchOPIDD * (gear_motor_data[Gimbal_P].speed_rpm * 2 * PI / 60);
	PitchOPIDPIDout = PitchOPIDPout + PitchOPIDIout + PitchOPIDDout;
	LimitMax(PitchOPIDPIDout, PitchOPIDPIDMax)
		PitchOPIDLastError = PitchOPIDCurrentError;

	/***************************************	�ڻ��ٶȻ�	******************************************/
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
