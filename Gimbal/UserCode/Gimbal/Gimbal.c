#include "Gimbal.h"
#include "caninfo.h"
#include "Remote_control.h"
#include "Filter.h"
#include "pid.h"
#include <math.h>
#include "usartinfo.h"
#include "buzzer.h"

#define Outpost_Mul_Inspect
#define Outpost_Mul_Inspect_Times 3

#define Inspect_Empty 1
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

#define Yaw_Front_Middle (301.5f)
#define Yaw_Back_Middle (Yaw_Front_Middle-180.f)
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
int sotf_start = 1;
int control_allow = 0;
int read_allow = 0;
int Pitch_dir=1;
int Yaw_dir=1;

float Zero_Offset[4]={0};
uint32_t Set_Zero=0;
bool Set_Zero_Complete=false;
float Yaw_Zero_Set_At=0;
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

void Gimbal_Sotf_Start(void)
{
	if (sotf_start == 1)
	{
		if (read_allow == 0)
		{
			if(Pitch_USE_Gyro==true)
			{
				pitch_angle = eular[0];
			}
			else
			{
				pitch_angle = gear_motor_data[Gimbal_P].angle * Motor_Ecd_to_Ang;
			}
			yaw_angle = Yaw_Motor_Angle_Change();
			read_allow = 1;
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
			if(Set_State>3)
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
		
		float dif=yaw_center-yaw_angle;	//劣弧
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
		
		float Soft_Start_Angle_Pitch=pitch_angle;
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
			if(((Soft_Start_Angle_Pitch<(Pitch_Motor_Zero+Soft_Start_Up_Error) && Soft_Start_Angle_Pitch>(Pitch_Motor_Zero-Soft_Start_Up_Error)) && (Soft_Start_Angle_Yaw<yaw_center+Soft_Start_Up_Error && Soft_Start_Angle_Yaw>yaw_center-Soft_Start_Up_Error)) ||
				 (((Soft_Start_Angle_Pitch<(Pitch_Middle+Soft_Start_Up_Error) && Soft_Start_Angle_Pitch>(Pitch_Middle-Soft_Start_Up_Error)) && (Soft_Start_Angle_Yaw<(yaw_center+Soft_Start_Up_Error) && Soft_Start_Angle_Yaw>(yaw_center-Soft_Start_Up_Error))) && (Set_State>=4)))
			{
				if(Set_Zero_Complete == false)
				{
					if(Set_State==4)
						Set_Zero=801;
					else
						++Set_Zero;
					if(Set_Zero > 800)
					{
						if(Set_Pitch_Zero_Point() == true)
						{
							#ifdef Zero_Sample_Once
								Set_Zero_Complete = true;
								Set_Zero = 0;
								control_allow = 1;
								sotf_start = 0;
							#else
								++Set_State;
								Yaw_Soft_Start_Speed_Ratio=3.f;
								Set_Zero = 0;
								if(Set_State<4)
									yaw_center-=90;
								else if(Set_State==4)
									yaw_center=301.5f;
								else if(Set_State==5)
								{
									Buzzer_Short(2);
									Yaw_Soft_Start_Speed_Ratio=1;
									Set_Zero_Complete = true;
									control_allow = 1;
									sotf_start = 0;
								}
							#endif
						}
					}
				}
				else
				{
					Buzzer_Short(2);
					control_allow = 1;
					sotf_start = 0;
				}
			}
		}
	}
}

bool Gimbal_Keep_Middle(void)
{
	static bool Return_State=false;	//防止因为云台抖动停止读取轨道长度
	if(yaw_angle < yaw_center + 0.5f && yaw_angle > yaw_center - 0.5f)
		Return_State=true;
	else
	{
		if(yaw_nowangle<Yaw_Limit_Min)
			Return_State=false;
		float dif=yaw_center-yaw_nowangle;
		int Direction=0;
		if(dif<=-180)
			Direction=1;
		else if(dif>-180 && dif <=0)
			Direction=-1;
		else if(dif>0 && dif<=180)
			Direction=1;
		else if(dif>180)
			Direction=-1;
		yaw_angle += 0.05f*3.f*Direction;
	}
	yaw_angle = loop_fp32_constrain(yaw_angle, 0, 360);
	yaw = Control_YawPID(yaw_angle);
	return Return_State;
}

void Gimbal_Remote_Control(void)
{
	if (control_allow == 1)
	{
		pitch_angle += 0.0005f * first_order_filter_Y_cali(remote_control.ch4);
		yaw_angle += 0.001f * first_order_filter_X_cali(remote_control.ch3);
	}
	yaw_angle = loop_fp32_constrain(yaw_angle, 0, 360);
	if(Limit_Yaw==true && Set_Zero_Complete==true)
	{
		float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;
		if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )
			yaw_angle=Yaw_Limit_Max;
		else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
			yaw_angle=Yaw_Limit_Min;
	}
	else
	{
		if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )
		{
			if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
				yaw_angle=195.f;
			else
				yaw_angle=Yaw_Limit_Min;
		}
	}
	yaw = Control_YawPID(yaw_angle);
	if(Pitch_USE_Gyro==true)
	{
		Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top);
	}
	else
	{
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top);
	}
	pitch = Control_PitchPID(pitch_angle);
}

void Gimbal_Automatic_control(void)
{
	if(control_allow == 1)
	{
		if(Aimming == 1)
		{
			float pre_yaw;
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
			if(Limit_Yaw==true)
			{
				float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;
				if( yaw_angle>Yaw_Limit_Max && yaw_angle<=Dead_Zone_Middle )
					yaw_angle=Yaw_Limit_Max;
				else if( yaw_angle<Yaw_Limit_Min && yaw_angle>Dead_Zone_Middle )
					yaw_angle=Yaw_Limit_Min;
			}
			else
			{
				if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )
				{
					if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
						yaw_angle=195.f;
					else
						yaw_angle=Yaw_Limit_Min;
				}
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
//			if(Inspect_Position==0)
//				Gimbal_Inspect();
//			else
				Gimbal_Position_Inspect();
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
 * @attention: 
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

#define Inspect_Front_Offset (0.f)
#define Inspect_Back_Offset (0.f)
float Inspect_Offset(void)
{
	if(yaw_angle<=Yaw_Limit_Max || yaw_angle>=Yaw_Limit_Min)
		return Inspect_Front_Offset;
	else if(yaw_angle<=191.f && yaw_angle>=44.6f)
		return -Inspect_Back_Offset;
	return 0;
}

void Gimbal_Inspect(void) //巡检
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
		if(Pitch_dir==1 && pitch_angle>=Pitch_Motor_Zero-16.f-Inspect_Empty+Inspect_Offset())
			Pitch_dir=-1;
		if(Pitch_dir==-1 && pitch_angle<=Pitch_Limit_Bottom+12.5f+Inspect_Empty)
			Pitch_dir=1;
	}
	pitch_angle += Pitch_dir*Pitch_Inspect_Speed;
	if(Pitch_USE_Gyro==true)
		Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top)
	else
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top)
	pitch = Control_PitchPID(pitch_angle);

	if(Limit_Yaw==false)
	{
		if( ((yaw_angle>191.f)&&(yaw_angle<231.f)) || ((yaw_angle>18.6f)&&(yaw_angle<44.6f)) )	//在柱子处加快巡检速度
			yaw_angle += Yaw_Inspect_Speed*3.f*Yaw_dir;
		else
			yaw_angle += Yaw_Inspect_Speed*Yaw_dir;
		if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )
		{
			if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
				yaw_angle=195.f;
			else
				yaw_angle=Yaw_Limit_Min;
		}
	}
	else
	{
		float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;
		#ifdef Outpost_Mul_Inspect
			if(Outpost_Alive==true)
			{
				static uint8_t Inspect_cnt=0;
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

uint8_t Position_Inspect_cnt=Position_Inspect_Time;
uint32_t Angle_Stay=0;
int Last_Yaw_Dir=0;
float Inspect_Angle=0;
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
		if(Pitch_dir==1 && pitch_angle>=Pitch_Motor_Zero-16.f-Inspect_Empty+Inspect_Offset())
			Pitch_dir=-1;
		if(Pitch_dir==-1 && pitch_angle<=Pitch_Limit_Bottom+12.5f+Inspect_Empty)
			Pitch_dir=1;
	}
	pitch_angle += Pitch_dir*Pitch_Inspect_Speed;
	if(Pitch_USE_Gyro==true)
		Limit(pitch_angle, Pitch_Gyro_Bottom, Pitch_Gyro_Top)
	else
		Limit(pitch_angle, Pitch_Limit_Bottom, Pitch_Limit_Top)
	pitch = Control_PitchPID(pitch_angle);
	
	if(Angle_Stay==0)
	{
		if(Position_Inspect_cnt==0)
			Inspect_Position=0;
		switch(Inspect_Position)
		{
			case 0:		//Normal
			{
				if(Limit_Yaw==false)
				{
					if( ((yaw_angle>191.f)&&(yaw_angle<231.f)) || ((yaw_angle>18.6f)&&(yaw_angle<44.6f)) )	//在柱子处加快巡检速度
						yaw_angle += Yaw_Inspect_Speed*2.f*Yaw_dir;
					if( Hit_Gimbal==true && yaw_angle>195.f && yaw_angle<Yaw_Limit_Min )
					{
						if( yaw_angle<(195.f+Yaw_Limit_Min)/2.f )
							yaw_angle=195.f;
						else
							yaw_angle=Yaw_Limit_Min;
					}
				}
				else
				{
					float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;
					#ifdef Outpost_Mul_Inspect
						if(Outpost_Alive==true)
						{
							static uint8_t Inspect_cnt=0;
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
				}
			}
			break;
			
			case 1:		// 1 Quadrant
			{
				float Dead_Zone_Middle=(Yaw_Limit_Min+Yaw_Limit_Max)/2.f;
				if(yaw_angle>=18.6f && yaw_angle<Dead_Zone_Middle)
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
			
			case 5:		// front
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
			
			case 6:		// back
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
		--Angle_Stay;
		if(Inspect_Angle>=0 && Inspect_Angle<90)
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
	}
	yaw_angle = loop_fp32_constrain(yaw_angle,0,360);
	yaw = Control_YawPID(yaw_angle);
	
	if(Yaw_dir!=Last_Yaw_Dir && Position_Inspect_cnt>0)
		--Position_Inspect_cnt;
	Last_Yaw_Dir=Yaw_dir;
}

float Yaw_Motor_Angle_Change(void)
{
	float Back = 0;
	int Round = gear_motor_data[Gimbal_Y].round_cnt % 2;
	if (Round == 0)
		Back = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang / 2.f;
	else if (Round == 1 || Round == -1)
		Back = gear_motor_data[Gimbal_Y].angle * Motor_Ecd_to_Ang / 2.f + 180.f;
	return Back;
}

#define Sampling_Times 5 //采样次数
#define Max_Error 0.5f	 //最大允许误差，超过误差重新采样
float Gyro_Average = 0;
float Motor_Average = 0;
uint8_t cnt=0;
float Pitch_Gyro_Buf[Sampling_Times] = {0};
float Pitch_Motor_Buf[Sampling_Times] = {0};
float Pitch_Gyro_Max = -181;
float Pitch_Gyro_Min = 181;
bool Set_Pitch_Zero_Point(void) //采样取平均确定Pitch零点
{
	static uint32_t Time_Div=0;
	if(Set_State>3)		//Sample complete, return true
		return true;
	if(++Time_Div%2==0)		//二分频	200Hz
	{
		if(Hi229_Update==1)
		{
			Hi229_Update = 0;
			//Update data
			Pitch_Gyro_Buf[cnt] = eular[0];
			Pitch_Motor_Buf[cnt] = gear_motor_data[Gimbal_P].angle;
			//Calculate Max&Min
			if (Pitch_Gyro_Buf[cnt] > Pitch_Gyro_Max)
				Pitch_Gyro_Max = Pitch_Gyro_Buf[cnt];
			if (Pitch_Gyro_Buf[cnt] < Pitch_Gyro_Min)
				Pitch_Gyro_Min = Pitch_Gyro_Buf[cnt];

			++cnt;
			if(cnt>=5)
			{
				cnt=0;
				if(Pitch_Gyro_Max-Pitch_Gyro_Min > Max_Error) //超过最大允许误差
				{
					Pitch_Gyro_Max = -181;
					Pitch_Gyro_Min = 181;
					for(uint8_t i=0; i<Sampling_Times; ++i)
						Pitch_Gyro_Buf[i]=Pitch_Motor_Buf[i]=0;
					return false;
				}

				for(uint8_t i=0; i<Sampling_Times; ++i)
				{
					Gyro_Average += Pitch_Gyro_Buf[i];
					Motor_Average += Pitch_Motor_Buf[i];
				}
				Gyro_Average/=Sampling_Times;
				Motor_Average/=Sampling_Times;
				Zero_Offset[Set_State]=-(Gyro_Average+Motor_Average*Motor_Ecd_to_Ang-Pitch_Motor_Zero);
				
				Pitch_Gyro_Max = -181;
				Pitch_Gyro_Min = 181;
				Gyro_Average=Motor_Average=0;
				for(uint8_t i=0; i<Sampling_Times; ++i)
					Pitch_Gyro_Buf[i]=Pitch_Motor_Buf[i]=0;

				Buzzer_Short(1);
				return true;
			}
		}
	}
	return false;
}

float Zero_Offset_Cal(void)
{
	if(Set_Zero_Complete==true)
	{
		float dif=yaw_center-yaw_nowangle;
		
		if(Limit_Yaw==true)
		{
			if(dif>180)
				dif=360-dif;
			else if(dif<0)
				dif=-dif;
		
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
			
			if(dif==0)
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

#define Yaw_Dead_Zone 0.25f
bool Yaw_At_Border(void)
{
	if( (yaw_nowangle>Yaw_Limit_Min-Yaw_Dead_Zone && yaw_nowangle<Yaw_Limit_Min+Yaw_Dead_Zone) || 
			(yaw_nowangle>Yaw_Limit_Max-Yaw_Dead_Zone && yaw_nowangle<Yaw_Limit_Max+Yaw_Dead_Zone) )
		return true;
	return false;
}
