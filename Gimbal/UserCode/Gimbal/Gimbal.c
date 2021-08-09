/*
 * @Author: M
 * @Date: 2021-04-11 09:56:15
 * @LastEditTime: 2021-08-09 15:20:42
 * @LastEditors: Please set LastEditors
 * @Description: 云台控制相关
 * @FilePath: \MDK-ARMe:\RM\Guard\NewGuard\Gimbal\UserCode\Gimbal\Gimbal.c
 */
#include "Gimbal.h"
#include "caninfo.h"
#include "Remote_control.h"
#include "Filter.h"
#include "pid.h"
#include <math.h>
#include "usartinfo.h"
#include "buzzer.h"

#define Outpost_Mul_Inspect	//前哨站重复巡检
#define Outpost_Mul_Inspect_Times 3	//前哨站重复巡检次数

#define Inspect_Empty 1	//巡检死区容许范围
float Pitch_Inspect_Speed = 0.25f;	//0.08
float Yaw_Inspect_Speed = 0.4f;	//it will be changed in function Gimbal_Inspect_setSpeed
#define Yaw_Inspect_Speed_Offset 0.025f

bool Limit_Yaw=false;
bool Pitch_USE_Gyro=false;		//陀螺仪太抖、弃用，仅保留代码
//#define Zero_Sample_Once		//零点采样，注释掉采样四次

/*
柱子
左边  191   231			40°
右边  18.6  44.6		26°
*/

//231 <--> 18.6
#define Yaw_Limit_Min 235.f	//257.6		231
#define Yaw_Limit_Max	18.6f	//				18.6

#define Yaw_Front_Middle (301.5f)	//云台正前方
#define Yaw_Back_Middle (Yaw_Front_Middle-180.f)	//正后方
float yaw_center=301.5f;
float Yaw_Soft_Start_Speed_Ratio=1.f;

#define Pitch_Gyro_Top 0.f
#define Pitch_Gyro_Bottom (-46.52f)

float Pitch_Limit_Top = 129.f; //2788	121.52448f	127.52448
float Pitch_Limit_Bottom = 75.f;	//61.53f	75.85f
float Pitch_Middle=98.f;

int32_t pitch, yaw;
float yaw_angle;
float pitch_angle;
int sotf_start = 1;	//云台缓起标志位
int control_allow = 0;	//缓起完成后允许控制标志位
int read_allow = 0;	//读电机数据标志位
int Pitch_dir=1;	//Pitch巡检方向
int Yaw_dir=1;		//Yaw巡检方向

float Zero_Offset[4]={0};	//电机绝对零度校准数据区
uint32_t Set_Zero=0;	//零点校准标志位
bool Set_Zero_Complete=false;	//零点校准完成标志
int Set_State=0;	//校零阶段标志位

// 自瞄时实际执行的pitch、yaw，云台坐标系
float vision_target_pitch, vision_target_yaw;
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

/**
 * @description: 循环限幅函数
 * @param {float} Input
 * @param {float} minValue 限幅最小值
 * @param {float} maxValue 限幅最大值
 * @return {float} 限幅输出
 */
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
 * @description: 云台缓起
 * @param {*}
 * @return {*}
 */
void Gimbal_Sotf_Start(void)
{
	if (sotf_start == 1)
	{
		if (read_allow == 0)
		{
			if(Pitch_USE_Gyro==true)	//使用陀螺仪已弃用, 仅保留代码
			{
				pitch_angle = eular[0];
			}
			else
			{
				pitch_angle = gear_motor_data[Gimbal_P].angle * Motor_Ecd_to_Ang;
			}
			yaw_angle = Yaw_Motor_Angle_Change();
			read_allow = 1;	//缓起完成, Pitch和Yaw角度转换
		}
		if(Pitch_USE_Gyro==true)
		{
			if (pitch_angle < Pitch_Gyro_Top)
			{
				pitch_angle += 0.05f;
			}
			else if (pitch_angle > Pitch_Gyro_Top)
			{
				pitch_angle -= 0.05f;
			}
		}
		else
		{
			if(Set_State>3)	//每次进入中断让Pitch期望角度自增到缓起中心
			{
				if (pitch_angle < Pitch_Middle)
					pitch_angle += 0.075f;
				else if (pitch_angle > Pitch_Middle)
					pitch_angle -= 0.075f;
			}
			else
			{
				if (pitch_angle < Pitch_Motor_Zero)
					pitch_angle += 0.075f;
				else if (pitch_angle > Pitch_Motor_Zero)
					pitch_angle -= 0.075f;
			}
		}
		
		float dif=yaw_center-yaw_angle;	//取劣弧,为了解决从0-360的循环限幅问题
		int Direction=0;
		if(dif<=-180)
			Direction=1;
		else if(dif>-180 && dif <=0)
			Direction=-1;
		else if(dif>0 && dif<=180)
			Direction=1;
		else if(dif>180)
			Direction=-1;
		yaw_angle += 0.075f*Yaw_Soft_Start_Speed_Ratio*Direction;
		
		float Soft_Start_Angle_Pitch=pitch_angle;	//调试用的,方便切换
		float Soft_Start_Angle_Yaw=yaw_angle;
		
		if(Pitch_USE_Gyro==true)
		{
			if ((Soft_Start_Angle_Pitch < Pitch_Gyro_Top + Soft_Start_Up_Error && Soft_Start_Angle_Pitch > Pitch_Gyro_Top - Soft_Start_Up_Error)&&(yaw_nowangle < yaw_angle + Soft_Start_Up_Error && yaw_nowangle > yaw_angle - Soft_Start_Up_Error))
			{
				control_allow = 1;
				sotf_start = 0;
			}
		}
		else
		{
			if(((Soft_Start_Angle_Pitch<(Pitch_Motor_Zero+Soft_Start_Up_Error) && Soft_Start_Angle_Pitch>(Pitch_Motor_Zero-Soft_Start_Up_Error)) && (Soft_Start_Angle_Yaw<yaw_center+Soft_Start_Up_Error && Soft_Start_Angle_Yaw>yaw_center-Soft_Start_Up_Error)) ||	//零点校准缓起,没有完成零点校准
				 (((Soft_Start_Angle_Pitch<(Pitch_Middle+Soft_Start_Up_Error) && Soft_Start_Angle_Pitch>(Pitch_Middle-Soft_Start_Up_Error)) && (Soft_Start_Angle_Yaw<(yaw_center+Soft_Start_Up_Error) && Soft_Start_Angle_Yaw>(yaw_center-Soft_Start_Up_Error))) && (Set_State>=4)))	//完成零点校准,正常缓起
			{
				if(Set_Zero_Complete == false)	//没有完成零点校准
				{
					if(Set_State==4)	//采样四次
						Set_Zero=801;	//采样完成
					else
						++Set_Zero;	//没有采样完成,让计数值自增, 云台缓起到指定角度之后,整辆车可能在晃动,陀螺仪也没完全稳定,需要等待一段时间
					if(Set_Zero > 800)
					{
						if(Set_Pitch_Zero_Point() == true)	//零点采样函数,在这里调用,当Set_Zero>800,就相当于在400Hz定时器中一直调用这个零点采样函数
						{
							#ifdef Zero_Sample_Once	//单次零点采样已弃用, 精度太低
								Set_Zero_Complete = true;
								Set_Zero = 0;
								control_allow = 1;
								sotf_start = 0;
							#else
								++Set_State;	//完成一次采样之后，让采样计时值自增
								Yaw_Soft_Start_Speed_Ratio=3.f;	//更改Yaw的移动速度，原本缓起的速度太慢
								Set_Zero = 0;	//等待云台稳定计数值清零
								if(Set_State<4)	//采样次数小于4，让下一个采样点的Yaw角度值减90
									yaw_center-=90;
								else if(Set_State==4)	//最后一次回到原本的缓起中心，不能继续减90是0-360的限制
									yaw_center=301.5f;
								else if(Set_State==5)	//第五次，即完成零点采样
								{
									Buzzer_Short(2);
									Yaw_Soft_Start_Speed_Ratio=1;	//Yaw缓起速度改为原本的速度
									Set_Zero_Complete = true;	//置位零点采样完成标志
									control_allow = 1;	//置位允许控制
									sotf_start = 0;	//置位缓起
								}
							#endif
						}
					}
				}
				else	//零点采样完成之后的普通缓起
				{
					Buzzer_Short(2);
					control_allow = 1;
					sotf_start = 0;
				}
			}
		}
	}
}

/**
 * @description: 保持云台在中间，轨道长度采样的时候用到，防止云台撞柱子
 * @param {*}
 * @return {bool} 云台到达中间之后返回true, 不在中间则返回false
 */
bool Gimbal_Keep_Middle(void)
{
	static bool Return_State=false;	//防止因为云台抖动停止读取轨道长度
	if(yaw_angle < yaw_center + 0.5f && yaw_angle > yaw_center - 0.5f)	//在中间，返回true
		Return_State=true;
	else
	{	//不在中间
		if(yaw_nowangle<Yaw_Limit_Min)
			Return_State=false;
		float dif=yaw_center-yaw_nowangle;
		int Direction=0;
		if(dif<=-180)	//根据差值，确定回到中间的方向，取劣弧回到中间
			Direction=1;
		else if(dif>-180 && dif <=0)
			Direction=-1;
		else if(dif>0 && dif<=180)
			Direction=1;
		else if(dif>180)
			Direction=-1;
		yaw_angle += 0.05f*3.f*Direction;
	}
	yaw_angle = loop_fp32_constrain(yaw_angle, 0, 360);	//循环限幅
	yaw = Control_YawPID(yaw_angle);	//PID计算
	return Return_State;
}

/**
 * @description: 云台遥控器控制
 * @param {*}
 * @return {*}
 */
void Gimbal_Remote_Control(void)
{
	if (control_allow == 1)	//缓起完成之后
	{
		pitch_angle += 0.0005f * first_order_filter_Y_cali(remote_control.ch4);	//遥控器的值滤波之后乘上比例
		yaw_angle += 0.001f * first_order_filter_X_cali(remote_control.ch3);
	}
	yaw_angle = loop_fp32_constrain(yaw_angle, 0, 360);	//限幅
	if(Limit_Yaw==true && Set_Zero_Complete==true)	//限制Yaw
	{
		float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;	//死区中心
		if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )	//当云台在死区里，取劣弧返回非死区
			yaw_angle=Yaw_Limit_Max;
		else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
			yaw_angle=Yaw_Limit_Min;
	}
	else	//不限制Yaw的时候要考虑云台撞柱的问题
	{	//Hit_Gimbal由底盘发过来
		if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )	//取最小角度避开撞柱
		{
			if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
				yaw_angle=195.f;
			else
				yaw_angle=Yaw_Limit_Min;
		}
	}
	yaw = Control_YawPID(yaw_angle);
	if(Pitch_USE_Gyro==true)	//Pitch限幅
	{
		Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top);
	}
	else
	{
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top);
	}
	pitch = Control_PitchPID(pitch_angle);
}

/**
 * @description: 云台自动控制
 * @param {*}
 * @return {*}
 */
#define Gimbal_Force_Time 400	//云台手强制控制时间 400Hz
uint16_t Gimbal_Force_Time_cnt=Gimbal_Force_Time;	//云台手强制控制计数值
void Gimbal_Automatic_control(void)
{
	if(control_allow==1)	//缓起完成，允许控制
	{
		if(Aimming==1 && Gimbal_Force_Time_cnt>=Gimbal_Force_Time)	//瞄准到目标，强制控制时间已结束
		{
			float pre_yaw;	//叶哥的预测
			vision_target_yaw += yaw_det_average * 50.0f * 0.005f;
			pre_yaw = vision_target_yaw + (yaw_det_average * 50.0f * 0.2f);
			pre_yaw = loop_fp32_constrain(pre_yaw, 0, 360);
			if (pre_yaw - yaw_angle > 0.1f)
			{
				pre_yaw = yaw_angle + 0.1f;
			}
			else if (pre_yaw - yaw_angle < -0.1f)
			{
				pre_yaw = yaw_angle - 0.1f;
			}
			yaw_angle = loop_fp32_constrain(pre_yaw, 0, 360);
			if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )
			{
				if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
					yaw_angle=195.f;
				else
					yaw_angle=Yaw_Limit_Min;
			}
			yaw = Control_YawPID(yaw_angle);

			if (vision_target_pitch - pitch_angle > 0.5f)
			{
				pitch_angle += 0.5f;
			}
			else if (vision_target_pitch - pitch_angle < -0.5f)
			{
				pitch_angle -= 0.5f;
			}
			else
			{
				pitch_angle = vision_target_pitch;
			}

			if(Pitch_USE_Gyro==true)
			{
				Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top);
			}
			else
			{
				Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top);
			}
			pitch = Control_PitchPID(pitch_angle);
			
			Position_Inspect_cnt=Inspect_Position=Angle_Stay=0;	//clear flag
		}
		else
		{
			if(Gimbal_Force_Time_cnt<Gimbal_Force_Time)	//云台手强制控制，一定时间内无视瞄准到的目标
				++Gimbal_Force_Time_cnt;
			Gimbal_Position_Inspect();	//云台巡检
		}
	}
}

/**
 * @brief: 更改自瞄目标，当串口接收到新目标时运行这个函数
 * @param {*}
 * @retval: 
 * @attention: 输入为视觉坐标系（-180°~180°）
 */
void Gimbal_Automatic_target(float _pitch, float _yaw)
{
	float det_yaw = _yaw - last_time_target_yaw;

	if (det_yaw > 180.0f)
	{
		det_yaw = 360.0f - det_yaw;
	}
	else if (det_yaw < -180.0f)
	{
		det_yaw = -360.0f - det_yaw;
	}

	if (fabs(det_yaw) > 1.5)
	{
		det_yaw = 0;
	}

	previous_target_yaw_det[previous_target_yaw_det_index] = det_yaw;
	if (previous_target_yaw_det_index < PREVIOUS_DET_LENGTH)
		previous_target_yaw_det_index++;
	else
		previous_target_yaw_det_index = 0;

	if (++n > PREVIOUS_DET_LENGTH)
	{
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
	if(Pitch_USE_Gyro==true)
	{
		vision_target_pitch = _pitch;
	}
	else
	{
		vision_target_pitch = _pitch + Pitch_Motor_Zero;
	}
	vision_target_yaw = _yaw;
}

void Gimbal_Automatic_target_lost()
{
	last_time_target_yaw = 0.0;
	previous_target_yaw_det_index = 0;
	n = 0;
	vision_target_yaw = yaw_angle;
	vision_target_pitch = pitch_angle;
	yaw_det_average = 0.0;
}

/**
 * @brief: 巡检速度设置
 * @param {*}
 * @retval: 
 * @attention: 在串口接收NUC数据时调用
 */
void Gimbal_Inspect_setSpeed(int speed)
{
	switch (speed)
	{
		case Gimbal_Inspect_SPEED_SLOW:
				Yaw_Inspect_Speed = 0.05f;
				Pitch_Inspect_Speed = 0.2f;		//0.05
				break; 
		
		case Gimbal_Inspect_SPEED_FAST:
		default:
				Yaw_Inspect_Speed = 0.5f;
				Pitch_Inspect_Speed = 0.25f;		//0.08
				break;
	}
}

/**
 * @description: 改变Pitch上限位，云台面向前方时，需要看到更高的地方，面向后面时，不需要看得高
 * @param {*}
 * @return {float} 改变限位的偏置值
 */
#define Inspect_Front_Offset (0.f)
#define Inspect_Back_Offset (0.f)
float Inspect_Offset(void)
{
	if(yaw_angle<=Yaw_Limit_Max || yaw_angle>=Yaw_Limit_Min)	//根据Yaw的角度，判断在云台在前面还是在后面
		return Inspect_Front_Offset;
	else if(yaw_angle<=191.f && yaw_angle>=44.6f)	//返回不同的偏置值
		return -Inspect_Back_Offset;
	return 0;
}

/**
 * @description: 云台普通巡检，云台手不能控制，已弃用，仅保留代码
 * @param {*}
 * @return {*}
 */
void Gimbal_Inspect(void) //巡检
{
	if(Pitch_USE_Gyro==true)	//陀螺仪方案已弃用
	{
		if(Pitch_dir==1 && pitch_angle>=Pitch_Gyro_Top-16.f-Inspect_Empty+Inspect_Offset())
			Pitch_dir=-1;
		if(Pitch_dir==-1 && pitch_angle<=Pitch_Gyro_Bottom+12.5f+Inspect_Empty)
			Pitch_dir=1;
	}
	else
	{
		if(Pitch_dir==1 && pitch_angle>=Pitch_Motor_Zero-16.f-Inspect_Empty+Inspect_Offset())	//根据当前Pitch角度确定巡检方向
			Pitch_dir=-1;
		if(Pitch_dir==-1 && pitch_angle<=Pitch_Limit_Bottom+12.5f+Inspect_Empty)
			Pitch_dir=1;
	}
	pitch_angle += Pitch_dir*Pitch_Inspect_Speed;	//Pitch角度自增或自减
	if(Pitch_USE_Gyro==true)	//限幅
		Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top)
	else
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top)
	pitch = Control_PitchPID(pitch_angle);	//PID计算

	if(Limit_Yaw==false)	//360巡检
	{
		if( ((yaw_angle>191.f)&&(yaw_angle<231.f)) || ((yaw_angle>18.6f)&&(yaw_angle<44.6f)) )	//在柱子处加快巡检速度
			yaw_angle += Yaw_Inspect_Speed*3.f*Yaw_dir;
		else
			yaw_angle += Yaw_Inspect_Speed*Yaw_dir;
		if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )	//云台撞柱规避
		{
			if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )	//取最小角度方向规避
				yaw_angle=195.f;
			else
				yaw_angle=Yaw_Limit_Min;
		}
	}
	else
	{
		float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;	//死区中心
		#ifdef Outpost_Mul_Inspect	//前哨站方向重复巡检
			if(Outpost_Alive==true)	//前哨站存活才进行重复巡检
			{
				static uint8_t Inspect_cnt=0;	//前哨站方向重复巡检计数
				static bool Clear_Flag=false;	//完成一次重复巡检的标志位
				if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )	//我自己也看不懂了 麻了
				{
					Yaw_dir=-1;
					if(Clear_Flag==true)
						Clear_Flag=false;
				}
				else if( yaw_angle<301.5f && yaw_angle>Dead_Zone_Middle )
				{
					if(Inspect_cnt<Outpost_Mul_Inspect_Times)
					{
						Yaw_dir=1;
						if(Clear_Flag==false)
							++Inspect_cnt;
					}
					else if(yaw_angle<Yaw_Limit_Min)
					{
						Yaw_dir=1;
						Clear_Flag=true;
						Inspect_cnt=0;
					}
				}
			}
			else
			{	//前哨站灭了，正常巡检
				if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )
					Yaw_dir=-1;
				else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
					Yaw_dir=1;
			}
		#else
			if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )
				Yaw_dir=-1;
			else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
				Yaw_dir=1;
		#endif
		yaw_angle += Yaw_dir*Yaw_Inspect_Speed;
	}
	yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
	yaw = Control_YawPID(yaw_angle);
}

/**
 * @description: 云台巡检+云台手控制
 * @param {*}
 * @return {*}
 */
uint8_t Position_Inspect_cnt=Position_Inspect_Time;	//位置巡检计数值
uint32_t Angle_Stay=0;	//云台手标点角度停留时间计数值
int Last_Yaw_Dir=0;	//上一次Yaw巡检方向
float Inspect_Angle=0;	//云台手标点角度
void Gimbal_Position_Inspect(void)
{
	if(Pitch_USE_Gyro==true)
	{
		if(Pitch_dir==1 && pitch_angle>=Pitch_Gyro_Top-16.f-Inspect_Empty+Inspect_Offset())
			Pitch_dir=-1;
		if(Pitch_dir==-1 && pitch_angle<=Pitch_Gyro_Bottom+12.5f+Inspect_Empty)
			Pitch_dir=1;
	}
	else
	{
		if(Pitch_dir==1 && pitch_angle>=Pitch_Motor_Zero-16.f-Inspect_Empty+Inspect_Offset())	//Pitch巡检方向
			Pitch_dir=-1;
		if(Pitch_dir==-1 && pitch_angle<=Pitch_Limit_Bottom+12.5f+Inspect_Empty)
			Pitch_dir=1;
	}
	pitch_angle += Pitch_dir*Pitch_Inspect_Speed;	//角度
	if(Pitch_USE_Gyro==true)
		Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top)
	else
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top)	//限幅
	pitch = Control_PitchPID(pitch_angle);	//PID计算
	
	//收到云台手的控制指令之后，先进行指定角度的巡检，再在云台手标点位置的象限巡检，最后回到普通巡检
	if(Angle_Stay==0)	//指定角度巡检完成
	{
		if(Position_Inspect_cnt==0)	//象限巡检完成
			Inspect_Position=0;	//巡检象限清零
		switch(Inspect_Position)
		{
			case 0:		//Normal正常巡检
			{
				if(Limit_Yaw==false)	//基地护盾被破
				{
					if( ((yaw_angle>191.f)&&(yaw_angle<231.f)) || ((yaw_angle>18.6f)&&(yaw_angle<44.6f)) )	//在柱子看到不到目标，处加快巡检速度
						yaw_angle += Yaw_Inspect_Speed*2.f*Yaw_dir;
					if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )	//撞柱规避
					{
						if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
							yaw_angle=195.f;
						else
							yaw_angle=Yaw_Limit_Min;
					}
				}
				else
				{
					float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;	//死区中心，取劣弧用
					#ifdef Outpost_Mul_Inspect
						if(Outpost_Alive==true)	//前哨站重复巡检
						{
							static uint8_t Inspect_cnt=0;	//自己也看不懂了
							static bool Clear_Flag=false;
							if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )
							{
								Yaw_dir=-1;
								if(Clear_Flag==true)
									Clear_Flag=false;
							}
							else if( yaw_angle<301.5f && yaw_angle>Dead_Zone_Middle )
							{
								if(Inspect_cnt<Outpost_Mul_Inspect_Times)
								{
									Yaw_dir=1;
									if(Clear_Flag==false)
										++Inspect_cnt;
								}
								else if(yaw_angle<Yaw_Limit_Min)
								{
									Yaw_dir=1;
									Clear_Flag=true;
									Inspect_cnt=0;
								}
							}
						}
						else
						{
							if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )	//正常巡检
								Yaw_dir=-1;
							else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
								Yaw_dir=1;
						}
					#else
						if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )
							Yaw_dir=-1;
						else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
							Yaw_dir=1;
					#endif
				}
			}
			break;
			
			case 1:		// 1 Quadrant
			{
				float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;	//每个象限的死区中心不同
				if(yaw_angle>=18.6f && yaw_angle<Dead_Zone_Middle)	//取劣弧，确定方向
				{
					yaw_angle=18.6f;
					Yaw_dir=-1;
				}
				else if(yaw_angle<=Yaw_Front_Middle && yaw_angle>=Dead_Zone_Middle)
				{
					yaw_angle=Yaw_Front_Middle;
					Yaw_dir=1;
				}
			}
			break;
				
			case 2:		// 2 Quadrant
			{
				float Dead_Zone_Middle=(Yaw_Front_Middle+Yaw_Limit_Min)/2.f-180.f;
				if(yaw_angle>=Yaw_Front_Middle || yaw_angle<Dead_Zone_Middle)
				{
					yaw_angle=Yaw_Front_Middle;
					Yaw_dir=-1;
				}
				else if(yaw_angle<=Yaw_Limit_Min && yaw_angle>=Dead_Zone_Middle)
				{
					yaw_angle=Yaw_Limit_Min;
					Yaw_dir=1;
				}
			}
			break;
			
			case 3:		// 3 Quadrant
			{
				if(yaw_angle>=191.f)
				{
					yaw_angle=191.f;
					Yaw_dir=-1;
				}
				else if(yaw_angle<Yaw_Back_Middle)
				{
					yaw_angle=Yaw_Back_Middle;
					Yaw_dir=1;
				}
			}
			break;
			
			case 4:		// 4 Quadrant
			{
				float Dead_Zone_Middle=(Yaw_Back_Middle+44.6f)/2.f+180.f;
				if(yaw_angle>=Yaw_Back_Middle && yaw_angle<Dead_Zone_Middle)
				{
					yaw_angle=Yaw_Back_Middle;
					Yaw_dir=-1;
				}
				else if(yaw_angle<=44.6f || yaw_angle>=Dead_Zone_Middle)
				{
					yaw_angle=44.6f;
					Yaw_dir=1;
				}
			}
			break;
			
			case 5:		// front正前方巡检，前方装甲板被击打
			{
				if(yaw_angle<Yaw_Limit_Min && yaw_angle>Yaw_Back_Middle)
				{
					yaw_angle=Yaw_Limit_Min;
					Yaw_dir=1;
				}
				else if(yaw_angle>Yaw_Limit_Max && yaw_angle<=Yaw_Back_Middle)
				{
					yaw_angle=Yaw_Limit_Max;
					Yaw_dir=-1;
				}
			}
			break;
			
			case 6:		// back后方巡检，后方装甲板被击打
			{
				if(yaw_angle>191.f && yaw_angle<Yaw_Front_Middle)
				{
					yaw_angle=191.f;
					Yaw_dir=-1;
				}
				else if(yaw_angle<44.6f || yaw_angle>=Yaw_Front_Middle)
				{
					yaw_angle=44.6f;
					Yaw_dir=1;
				}
			}
			break;
			
			default:
				Yaw_dir=0;
			break;
		}
		
		yaw_angle += Yaw_dir*Yaw_Inspect_Speed;
	}
	else
	{
		--Angle_Stay;	//指定角度巡检
		Gimbal_Force_Control();	//云台手控制角度转云台坐标系
	}
	yaw_angle = loop_fp32_constrain(yaw_angle,0,360);	//限幅
	yaw = Control_YawPID(yaw_angle);	//PID计算
	
	if(Yaw_dir!=Last_Yaw_Dir && Position_Inspect_cnt>0)	//巡检方向改变之后计数值自减，减到零回到正常巡检
		--Position_Inspect_cnt;	//这个变量在接收到云台手控制指令后置数
	Last_Yaw_Dir=Yaw_dir;
}

/**
 * @description: 云台手控制角度转云台坐标系
 * @param {*}
 * @return {*}
 */
void Gimbal_Force_Control(void)	//云台手控制,最高优先级
{
	if(Inspect_Angle>=0 && Inspect_Angle<90)	//云台坐标系转换
	{
		if(Inspect_Angle<12.9f)
			Inspect_Angle=12.9f;
		else
			Inspect_Angle=Yaw_Front_Middle+90-Inspect_Angle;
	}
	else if(Inspect_Angle>=90 && Inspect_Angle<180)
	{
		Inspect_Angle=Yaw_Front_Middle-(Inspect_Angle-90);
		if(Inspect_Angle<Yaw_Limit_Min)
			Inspect_Angle=Yaw_Limit_Min;
	}
	else if(Inspect_Angle>=180 && Inspect_Angle<270)
	{
		Inspect_Angle=270-Inspect_Angle+Yaw_Back_Middle;
		if(Inspect_Angle>191)
			Inspect_Angle=191;
	}
	else
	{
		Inspect_Angle=Yaw_Back_Middle-(Inspect_Angle-270);
		if(Inspect_Angle<44.6f)
			Inspect_Angle=44.6f;
	}
	yaw_angle=Inspect_Angle;
	yaw_angle=loop_fp32_constrain(yaw_angle,0,360);
}

/**
 * @description: Yaw电机坐标系转云台坐标系
 * @param {*}
 * @return {float} 返回转换之后的角度
 */
float Yaw_Motor_Angle_Change(void)
{
	float Back = 0;	//Yaw电机传动比2：1，通过电机圈数判断云台在哪个方向
	int Round = gear_motor_data[Gimbal_Y].round_cnt % 2;
	if (Round == 0)
		Back = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang / 2.f;
	else if (Round == 1 || Round == -1)
		Back = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang / 2.f + 180.f;
	return Back;
}

/**
 * @description: 零点采样函数,在云台缓起中调用，共4个样点，每90°一个
 * @param {*}
 * @return {bool} 采样一个位置的零点之后返回true, 正在采样或采样失败返回false
 */
#define Sampling_Times 5 //采样次数
#define Max_Error 0.5f	 //最大允许误差，超过误差重新采样
float Gyro_Average = 0;	//陀螺仪均值
float Motor_Average = 0;	//电机数据均值
uint8_t cnt=0;	//采样次数计数值,采样Sampling_Times次取平均值
float Pitch_Gyro_Buf[Sampling_Times] = {0};	//陀螺仪采样值存放数组
float Pitch_Motor_Buf[Sampling_Times] = {0};	//电机采样值存放数组
float Pitch_Gyro_Max = -181;	//陀螺仪采样数据最大值	陀螺仪返回数据-180~180
float Pitch_Gyro_Min = 181;		//陀螺仪采样数据最小值
bool Set_Pitch_Zero_Point(void)
{
	static uint32_t Time_Div=0;
	if(Set_State>3)		//Sample complete, return true
		return true;
	if(++Time_Div%2==0)		//二分频	200Hz
	{
		if(Hi229_Update==1)	//陀螺仪数据更新之后再进行采样,防止对没有更新的陀螺仪数据重复采样
		{
			Hi229_Update = 0;	//清空陀螺仪数据更新标志位
			//Update data
			Pitch_Gyro_Buf[cnt] = eular[0];
			Pitch_Motor_Buf[cnt] = gear_motor_data[Gimbal_P].angle;
			//Calculate Max&Min
			if (Pitch_Gyro_Buf[cnt] > Pitch_Gyro_Max)
				Pitch_Gyro_Max = Pitch_Gyro_Buf[cnt];
			if (Pitch_Gyro_Buf[cnt] < Pitch_Gyro_Min)
				Pitch_Gyro_Min = Pitch_Gyro_Buf[cnt];

			++cnt;	//取得一次数据之后,计数值自增
			if(cnt>=Sampling_Times)	//采样够指定次数
			{
				cnt=0;	//清空计数值,准备下一次采样
				if(Pitch_Gyro_Max-Pitch_Gyro_Min > Max_Error) //超过最大允许误差
				{
					Pitch_Gyro_Max = -181;	//重置最大最小值
					Pitch_Gyro_Min = 181;
					for(uint8_t i=0; i<Sampling_Times; ++i)		//清空数据暂存区
						Pitch_Gyro_Buf[i]=Pitch_Motor_Buf[i]=0;
					return false;
				}

				for(uint8_t i=0; i<Sampling_Times; ++i)	//计算平均值
				{
					Gyro_Average += Pitch_Gyro_Buf[i];
					Motor_Average += Pitch_Motor_Buf[i];
				}
				Gyro_Average/=Sampling_Times;
				Motor_Average/=Sampling_Times;
				Zero_Offset[Set_State]=-(Gyro_Average+Motor_Average*Motor_Ecd_to_Ang-Pitch_Motor_Zero);	//计算误差，转换成零点偏置
				
				Pitch_Gyro_Max = -181;	//重置最大最小值
				Pitch_Gyro_Min = 181;
				Gyro_Average=Motor_Average=0;	
				for(uint8_t i=0; i<Sampling_Times; ++i)
					Pitch_Gyro_Buf[i]=Pitch_Motor_Buf[i]=0;	//清空数据暂存区

				Buzzer_Short(1);
				return true;	//完成一次零点采样之后返回true
			}
		}
	}
	return false;
}

/**
 * @description: 零点偏置计算，根据当前的Yaw角度，返回偏置值
 * @param {*}
 * @return {float} Pitch偏置值
 */
float Zero_Offset_Cal(void)
{
	if(Set_Zero_Complete==true)	//已完成零点采样
	{
		float dif=yaw_center-yaw_nowangle;	//以云台正前方为中心判断偏置值
		
		//用采样点拟合出线性的方程
		if(Limit_Yaw==true)	//限制Yaw和不限制Yaw有两种不同的方法
		{
			if(dif>180)
				dif=360-dif;
			else if(dif<0)
				dif=-dif;	//计算差值
		
			if(dif==0 || dif==90)
				return 0;
			else if(dif<90)
				return Zero_Offset[0]-(dif/90.f)*Zero_Offset[0];
			else if(dif<=180)
				return -((dif-90.f)/90.f)*Zero_Offset[0];
		}
		else
		{
			if(dif>180)
				dif=dif-360;
			
			if(dif==0)	//从-180~180遍历所有情况，分母为0的单独列出
				return Zero_Offset[0];
			else if(dif>0 && dif <90)
				return ((90-dif)/90.f)*Zero_Offset[0]+(dif/90.f)*Zero_Offset[1];
			else if(dif==90)
				return Zero_Offset[1];
			else if(dif>90 && dif<180)
				return ((180-dif)/90.f)*Zero_Offset[1]+((dif-90)/90.f)*Zero_Offset[2];
			else if(dif==180)
				return Zero_Offset[2];
			else if(dif<0 && dif>-90)
				return ((dif+90)/90.f)*Zero_Offset[0]+(-dif/90.f)*Zero_Offset[3];
			else if(dif==-90)
				return Zero_Offset[3];
			else if(dif<-90 && dif>-180)
				return ((dif+180)/90.f)*Zero_Offset[3]+((-dif-90)/90.f)*Zero_Offset[2];
		}
	}
	return 0;
}

/**
 * @description: 在Limit_Yaw的条件下，判断云台是不是在限位的边界
 * @param {*}
 * @return {bool} 
 */
#define Yaw_Dead_Zone 0.25f
bool Yaw_At_Border(void)
{
	if( (yaw_nowangle>Yaw_Limit_Min-Yaw_Dead_Zone && yaw_nowangle<Yaw_Limit_Min+Yaw_Dead_Zone) || 
			(yaw_nowangle>Yaw_Limit_Max-Yaw_Dead_Zone && yaw_nowangle<Yaw_Limit_Max+Yaw_Dead_Zone) )
		return true;
	return false;
}
