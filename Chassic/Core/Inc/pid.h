#ifndef __PID_H__
#define __PID_H__

#include "main.h"

#define PI 3.1415926535897932f
#define Motor_Ang_to_Rad 0.017453f			 //����
#define Motor_Ecd_to_Ang 0.0439506775729459f // 360��/8192

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

	float target; //Ŀ��ֵ
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;

	float measure;	//����ֵ
	float err;		//���
	float last_err; //�ϴ����

	float pout;
	float iout;
	float dout;

	float output;	   //�������
	float last_output; //�ϴ����

	float MaxOutput;	 //����޷�
	float IntegralLimit; //�����޷�
	float DeadBand;		 //����������ֵ��
	float ControlPeriod; //��������
	float Max_Err;		 //������

	uint32_t thistime;
	uint32_t lasttime;
	uint8_t dtime;

	void (*f_param_init)(struct _PID_TypeDef *pid, //PID������ʼ��
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

	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp, float ki, float kd); //pid���������޸�
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);				 //pid����
} PID_TypeDef;

void pid_init(PID_TypeDef *pid);

#endif
