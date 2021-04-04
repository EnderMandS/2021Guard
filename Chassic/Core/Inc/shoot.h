#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "main.h"
#include "pid.h"

#define OUT	0
#define IN 	1

#define Cartridge_Ultra	2500
#define Cartridge_Fast	2000
#define Cartridge_Slow	1500

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

extern uint8_t Shoot_State;
extern uint16_t Cartridge_angle;
extern gear_moto_position_pid Cartridge_Position_Pid[2];
extern PID_TypeDef Cartridge_wheel;

void gear_moto_position_pid_init(void);
float gear_moto_position_pid_calc(gear_moto_position_pid *pid_out,gear_moto_position_pid *pid_in,float target,float now_angle,int16_t feeback_rpm);
void Cartridge_wheel_PID_Calc(int16_t Cartridge_wheel_Speed);

#endif
