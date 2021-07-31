# include "shoot.h"
# include "pid.h"
#include "bsp_can.h"
#include "usart.h"
#include "math.h"
#include "caninfo.h"
gear_moto_position_pid Cartridge_Position_Pid[2];
PID_TypeDef Cartridge_wheel;
PID_TypeDef Fric_wheel[2];
int32_t set_spd_to_Fric_wheel[2];
uint8_t Shoot_Ctrl=0;

void gear_moto_position_pid_init()
{
	Cartridge_Position_Pid[OUT].p =  0.15;
	Cartridge_Position_Pid[OUT].i =  0;
	Cartridge_Position_Pid[OUT].d = 0.25;
	Cartridge_Position_Pid[OUT].CurrentError = 0;
	Cartridge_Position_Pid[OUT].LastError = 0;
	Cartridge_Position_Pid[OUT].iMax = 0;
	Cartridge_Position_Pid[OUT].pidMax = 50;
	
	Cartridge_Position_Pid[IN].p = 1000;
	Cartridge_Position_Pid[IN].i = -0.1;
	Cartridge_Position_Pid[IN].d = 0;
	Cartridge_Position_Pid[IN].LastError =0;
	Cartridge_Position_Pid[IN].CurrentError = 0;
	Cartridge_Position_Pid[IN].iMax = 5000;
	Cartridge_Position_Pid[IN].pidMax = 30000;
}
float gear_moto_position_pid_calc(gear_moto_position_pid *pid_out,gear_moto_position_pid *pid_in,float target,float now_angle,int16_t feeback_rpm)
{
	
	pid_out->CurrentError =  target - now_angle;
		if(fabs(pid_out->CurrentError)>180)
	{
		if (pid_out->CurrentError >0)
		pid_out->CurrentError = -(360 - fabs(pid_out->CurrentError));
		else pid_out->CurrentError = (360 - fabs(pid_out->CurrentError));
	}
	pid_out->pout =  (pid_out->p)*(pid_out->CurrentError);
	pid_out->iout += (pid_out->i) *  (pid_out->CurrentError);
	LimitMax(pid_out->iout,pid_out->iMax)
	pid_out->dout = -(pid_out->d) * (feeback_rpm*2*PI/60/36);
	pid_out->pidout = (pid_out->pout)+(pid_out->iout)+(pid_out->dout);
	LimitMax((pid_out->pidout),(pid_out->pidMax))
	pid_out->LastError = pid_out->CurrentError;
	//
	//pid_out->pidout =0;
	//
	pid_in->CurrentError = pid_out->pidout-(feeback_rpm*2*PI/60/36);
	pid_in->pout =  (pid_in->p)*(pid_in->CurrentError);
	pid_in->iout += pid_in->i *  (pid_in->CurrentError);
	LimitMax(pid_in->iout,pid_in->iMax)
	pid_in->dout = pid_in->d*(pid_in->LastError - pid_in->CurrentError);
	pid_in->pidout = pid_in->pout+pid_in->iout+pid_in->dout;
	LimitMax(pid_in->pidout,pid_in->pidMax)
	pid_in->LastError = pid_in->CurrentError;
	return pid_in->pidout;
}
void Shoot_Speed_Pid_Init()
{
	for(uint8_t i=0; i<2; i++)
	{
		pid_init(&Fric_wheel[i]);
		Fric_wheel[i].f_param_init(&Fric_wheel[i],PID_Speed,13926,5000,10,0,8000,0,1.5,0.1,0);
	}
}

void Shoot_Speed_Pid_Calc(float Fric_Speed_target)
{
		set_spd_to_Fric_wheel[0] = Fric_Speed_target;
		set_spd_to_Fric_wheel[1] = -Fric_Speed_target;
		Fric_wheel[0].target = set_spd_to_Fric_wheel[0];
		Fric_wheel[0].f_cal_pid(&Fric_wheel[0],gear_motor_data[Fric_1].speed_rpm);		
		Fric_wheel[1].target = set_spd_to_Fric_wheel[1];
		Fric_wheel[1].f_cal_pid(&Fric_wheel[1],gear_motor_data[Fric_2].speed_rpm);
}
