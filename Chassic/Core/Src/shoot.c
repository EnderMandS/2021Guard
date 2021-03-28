#include "shoot.h"
#include "math.h"
#include "bsp_can.h"
#include "classic.h"

uint8_t Shoot_State=0;
gear_moto_position_pid Cartridge_Position_Pid[2];
PID_TypeDef Cartridge_wheel;
uint16_t Cartridge_angle=0;

void gear_moto_position_pid_init(void)
{
	//位置环Init
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
	
	//速度环Init
	pid_init(&Cartridge_wheel);
  Cartridge_wheel.f_param_init(&Cartridge_wheel,PID_Speed,10000,10000,10,0,2700,0,1.3,0.05,0);
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
void Cartridge_wheel_PID_Calc(int16_t Cartridge_wheel_Speed)
{
	Cartridge_wheel.target = Cartridge_wheel_Speed;
	Cartridge_wheel.f_cal_pid(&Cartridge_wheel,gear_motor_data[Cartridge].speed_rpm);
}
