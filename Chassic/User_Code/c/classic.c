#include "classic.h"
#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "bsp_can.h"
#include "stm32f4xx_it.h"

int Buff_Time=100;	//���������Ļ���ʱ��
uint16_t Time_Cnt=0;	//������������ʱ�����
int Last_Dir=1;	//ǰһ�η���

PID_TypeDef motor_pid[2];	//���̵��PID�ṹ��
uint8_t Moto_ID[2]={Chassic_L,Chassic_R};	//���̵��ID
uint8_t Move_Allow=0;	//�ƶ�״̬��, 0ֹͣ 1�ƶ� 2�쳤����
int direction=1;	//�����ƶ�����
int eliminate_dithering_left = 0;	//���⿪����������
int eliminate_dithering_right = 0;
int Classic_Move_Speed=Classic_Slow;	//�����ƶ��ٶ�,��ʼ��Ϊ����
uint8_t Aimming=0;	//��׼״̬
uint8_t Changing_Speed_Flag=0;	//�ı��ٶȷ���ʱ��1���ɱ��ٺ�����0		��翪�غ����ܼ����1

void Chassis_init(void)	//���̳�ʼ������
{
	for(int j=0;j<2;j++)
	{
		pid_init(&motor_pid[j]);	//PID��ʼ��
		motor_pid[j].f_param_init(&motor_pid[j],PID_Speed,5000,2500,10,0,6000,0,1.5,0.1,0);  //0.004
/*				   PID_ID id,uint16_t maxOutput,uint16_t integralLimit,float deadband,uint16_t controlPeriod,int16_t max_err,     
			int16_t  target,
			 float kp,
			 float ki,
			 float kd);*/
	}
}

float Slow_Change_Speed(int dir, uint16_t Speed)	//�����ı��ٶ�,���Լ�С�������
{
	Buff_Time=100;	//����ʱ��
	if(dir!=Last_Dir)	//����ı�
	{
		++Time_Cnt;	//��������
		if(Time_Cnt<Buff_Time)
		{
			if(Last_Dir==0)
				return dir*sin( (Time_Cnt*1.0)/Buff_Time*PI*0.5 )*Speed;	//ԭ�������
			else
				return Last_Dir*cos( (Time_Cnt*1.0)/Buff_Time*PI )*Speed;	//���������
		}
		else
		{
			Last_Dir=dir;	//�������
			Changing_Speed_Flag=Time_Cnt=0;	//��ռ���
		}
	}
	return dir*Speed;
}

#define Wait_Cnt 400		//��⵽�����ת֮��ȴ�ʱ�� ʵ��ʱ��=Wait_Cnt/400Hz
bool Dir_Change_Wait=false;
int16_t Max_Speed=0;	//���ɵ��������ͷŵ��̵�����ٶ�
uint32_t Dir_Change_Wait_Cnt=0;	//��⵽�����ת֮��ȴ�����
void Spring(int dir,uint16_t Speed)
{
	if(dir!=Last_Dir)	//�ٶȸı�
	{
		if(Last_Dir==1)	//�жϷ���
		{
			if(gear_motor_data[Moto_ID[0]].speed_rpm>0)	//�����0,�ȴ����ɷ���
			{
				for (uint8_t i=0; i<2; i++)
				{
					motor_pid[i].target=dir*Speed;
					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
			else
			{
				Dir_Change_Wait=true;
				Last_Dir=dir;
			}
			return;
		}
		else if(Last_Dir==-1)
		{
			if(gear_motor_data[Moto_ID[0]].speed_rpm<0)	//�����0,�ȴ����ɷ���
			{
				for(uint8_t i=0; i<2; i++)
				{
					motor_pid[i].target=dir*Speed;
					motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
					Motor_Output[ Moto_ID[i] ]=0;
				}
			}
			else
			{
				Dir_Change_Wait=true;
				Last_Dir=dir;
			}
			return;
		}
	}
	else
	{
		if(Dir_Change_Wait==true)	//�ȴ����ɵ��������ͷ�
		{
			++Dir_Change_Wait_Cnt;
			if(Dir_Change_Wait_Cnt>=Wait_Cnt)	//���ȴ�1s, ��ʱ�����ƶ�
			{
				Dir_Change_Wait_Cnt=0;
				Dir_Change_Wait=false;
			}
			if( abs(gear_motor_data[Moto_ID[0]].speed_rpm) > abs(Max_Speed) )	//��¼��������ٶ�
				Max_Speed=gear_motor_data[Moto_ID[0]].speed_rpm;
			else if( abs(gear_motor_data[Moto_ID[0]].speed_rpm) < abs(Max_Speed)-200 )	//����������ȫ�ͷ�,�ٶȿ�ʼ�½�,�������
			{
				Dir_Change_Wait_Cnt=0;
				Dir_Change_Wait=false;
			}
		}
		else	//�����ƶ�PID����
		{
			for (uint8_t i=0; i<2; i++)
			{
				motor_pid[i].target=dir*Speed;
				motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
				Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
			}
		}
	}
}

#define Sample_Times 6	//��������,ʵ��ʹ��5������,��һ�����ݲ����ڹ����Ե��ʼ��������
uint8_t Measuer_State=Ready_Measure;	//����״̬
uint32_t Rail_Len=0;	//�������
uint32_t Rail_Len_Buff[Sample_Times]={0};	//������Ȼ�����
uint16_t Rail_Len_Buff_cnt=0;	//������������ֵ
void Measuer_Rail_Len(void)
{
	if(Measuer_State==End_Measure)	//��ɲ���,�˳�����
		return;
	if(direction!=Last_Dir)	//�ٶȷ���,�Ѿ�������һ��
	{
		if(Measuer_State==Measuring)	//������
		{
			Rail_Len_Buff[Rail_Len_Buff_cnt]=abs(gear_motor_data[Moto_ID[0]].round_cnt);	//д�뻺����
			++Rail_Len_Buff_cnt;
			if(Rail_Len_Buff_cnt!=Sample_Times)	//û�в������
			{
				Measuer_State=Ready_Measure;	//׼���´μ���
			}
			else
			{
				uint32_t sum=0;	//�������,���ȡƽ��
				for(uint8_t i=1; i<Sample_Times; ++i)	//��һ����������
					sum += Rail_Len_Buff[i];
				Rail_Len = sum/(Sample_Times-1);
				Measuer_State=End_Measure;	//�ò�����ɱ�־λ
			}
		}
	}
	else	//û�д�������,�����ƶ�
	{
		if(Measuer_State==Ready_Measure)	//׼����һ�β���,Ȧ������
		{
			gear_motor_data[Moto_ID[0]].round_cnt=0;
			gear_motor_data[Moto_ID[1]].round_cnt=0;
			Measuer_State=Measuring;	//�ò����б�־λ
		}
	}
}
void Go_To_Middle(uint16_t Speed)	//�ص�����м�	0.4-0.6
{
	int dir=0;
	if(Rail_Position<0.4f)	//�ж�λ��,ȷ���ƶ�����
		dir=-1;
	else if(Rail_Position>0.6f)
		dir=1;
	for (uint8_t i=0; i<2; i++)	//pid����
	{
		motor_pid[i].target=dir*Speed;
		motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
	}
	if(dir!=0)	//����Ϊ0,�������
	{
		for (uint8_t i=0; i<2; i++)
			Motor_Output[ Moto_ID[i] ]=motor_pid[i].output;
	}
	else
	{
		for (uint8_t i=0; i<2; i++)
			Motor_Output[ Moto_ID[i] ]=0;
	}
}
