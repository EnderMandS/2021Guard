#ifndef __SHOOT__
#define __SHOOT__
#include "stdint.h"
/***************减速电机角度环******************************/
#define OUT  0
#define IN 1
//extern float	GEAR_MOTO_PID[2];
typedef struct
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
}gear_moto_position_pid;

extern gear_moto_position_pid Cartridge_Position_Pid[2];
extern uint8_t Shoot_Ctrl;

void gear_moto_position_pid_init(void);
float gear_moto_position_pid_calc(gear_moto_position_pid *pid_out,gear_moto_position_pid *pid_in,float target,float now_angle,int16_t feeback_rpm);
void Shoot_Speed_Pid_Init(void);
void Shoot_Speed_Pid_Calc(float Fric_Speed_target);
void Cartridge_wheel_PID_Calc(int16_t Cartridge_wheel_Speed);
#endif

