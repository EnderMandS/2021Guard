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
#ifndef PID_H
#define PID_H
#include "main.h"
/***************底盘+摩擦轮+拨弹轮速度pid******************/
typedef enum
{
	PID_Position,
	PID_Speed
} PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;

	float target; //目标值
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;

	float measure;	//测量值
	float err;		//误差
	float last_err; //上次误差

	float pout;
	float iout;
	float dout;

	float output;	   //本次输出
	float last_output; //上次输出

	float MaxOutput;	 //输出限幅
	float IntegralLimit; //积分限幅
	float DeadBand;		 //死区（绝对值）
	float ControlPeriod; //控制周期
	float Max_Err;		 //最大误差

	uint32_t thistime;
	uint32_t lasttime;
	uint8_t dtime;

	void (*f_param_init)(struct _PID_TypeDef *pid, //PID参数初始化
						 PID_ID id,
						 uint16_t maxOutput,
						 uint16_t integralLimit,
						 float deadband,
						 uint16_t controlPeriod,
						 int16_t max_err,
						 int16_t target,
						 float kp,
						 float ki,
						 float kd);

	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp, float ki, float kd); //pid三个参数修改
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);				 //pid计算
} PID_TypeDef;

void pid_init(PID_TypeDef *pid);
/********************************************************/
// 串级PID
typedef struct _CascadePID_TypeDef
{
	float p;
	float i;
	float d;
	float CurrentError;
	float LastError;
	float iMax;
	float pidMax;
	float pout;
	float iout;
	float dout;
	float pidout;
} CascadePID_TypeDef;

/***************云台位置pid********************/
double Yaw_PID_cal(double angle_target, double max);

double Pitch_PID_cal(double angle_target, double max);

float Control_PitchPID(float Target);
float Control_YawPID(float Target);
float Control_follow_YawPID(float Angle_now, float Target, float YawOPIDP_follow, float YawOPIDI_follow, float YawOPIDD_follow, float YawIPIDP_follow, float YawIPIDI_follow, float YawIPIDD_follow);
/************************************************/
/****************拨弹轮位置pid**************************


********************************************/

#define PI 3.1415926535897932f
#define Motor_Ang_to_Rad 0.017453f			 //弧度
#define Motor_Ecd_to_Ang 0.0439506775729459f // 360度/8192

#define LimitMax(input, max)   \
	{                          \
		if (input > max)       \
		{                      \
			input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
			input = -max;      \
		}                      \
	}

#define Limit(input, min, max) \
	{                          \
		if (input > max)       \
		{                      \
			input = max;       \
		}                      \
		else if (input < min)  \
		{                      \
			input = min;       \
		}                      \
	}
extern PID_TypeDef Cartridge_wheel;
extern PID_TypeDef Fric_wheel[2];
extern PID_TypeDef Classis_wheel[4];
extern PID_TypeDef Chassis_wheel[4];
extern int32_t set_spd_to_Classis_wheel;
extern int32_t set_spd_to_Fric_wheel[2];

#endif
