# include "Filter.h"
#include "stdint.h"
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.1666666667f
#define CHASSIS_CONTROL_TIME 0.015f


/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */

float first_order_filter_X_cali(float input)
{
    static float first_order_filter_X_out ;
	
		first_order_filter_X_out = (CHASSIS_ACCEL_X_NUM / (CHASSIS_ACCEL_X_NUM + CHASSIS_CONTROL_TIME) * first_order_filter_X_out + CHASSIS_CONTROL_TIME / (CHASSIS_ACCEL_X_NUM + CHASSIS_CONTROL_TIME) * input);
		
		return first_order_filter_X_out;
}
float first_order_filter_Y_cali(float input)
{
    static float first_order_filter_Y_out ;
		first_order_filter_Y_out = (CHASSIS_ACCEL_Y_NUM / (CHASSIS_ACCEL_Y_NUM + CHASSIS_CONTROL_TIME) * first_order_filter_Y_out + CHASSIS_CONTROL_TIME / (CHASSIS_ACCEL_Y_NUM + CHASSIS_CONTROL_TIME) * input);
		return first_order_filter_Y_out;
}


/*
#define max_len 30
int huadpnglvbo(int newdata){
	static int data[max_len]={0};
	static int sum=0;
	sum-=data[0];
	for(char i=0;i<28;i++){
		data[i]=data[i+1];
	}
	data[29]=newdata;
	sum+=data[29];
	return sum;
}
*/

void Data_Common(int* Data_Tab,int p,int *Idx,int *Sum,int MagBARO_TAB_SIZE)                 //移窗平均值算法
{
 *Idx = *Idx % MagBARO_TAB_SIZE; 
 (*Sum) -= Data_Tab[*Idx];
 Data_Tab[*Idx] = p;
 (*Sum) += Data_Tab[*Idx];
 (*Idx) ++;
}

void Data_Float_Common(float* Data_Tab,float p,int *Idx,float *Sum,int MagBARO_TAB_SIZE)                 //移窗平均值算法
{
 *Idx = *Idx % MagBARO_TAB_SIZE; 
 (*Sum) -= Data_Tab[*Idx];
 Data_Tab[*Idx] = p;
 (*Sum) += Data_Tab[*Idx];
 (*Idx) ++;
}
