#ifndef __PID_H__
#define __PID_H__

#include "main.h"

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

#endif
