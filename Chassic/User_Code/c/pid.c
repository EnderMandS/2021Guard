/*
 * @Author: your name
 * @Date: 2021-05-21 10:31:10
 * @LastEditTime: 2021-08-09 15:42:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMe:\RM\Guard\NewGuard\Chassic\User_Code\c\pid.c
 */
#include "pid.h"

#define ABS(x) ((x > 0) ? x : -x)
static void pid_param_init(
							PID_TypeDef *pid,
							PID_ID id,
							uint16_t maxout,
							uint16_t intergral_limit,
							float deadband,
							uint16_t period,
							int16_t max_err,
							int16_t target,

							float kp,
							float ki,
							float kd)
{
	pid->id = id;

	pid->ControlPeriod = period; //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->output = 0;
}
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}
static float pid_calculate(PID_TypeDef *pid, float measure) //, int16_t target)
{
	//	uint32_t time,lasttime;

	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime - pid->lasttime;
	pid->measure = measure;
	//	pid->target = target;

	pid->last_err = pid->err;
	pid->last_output = pid->output;

	pid->err = pid->target - pid->measure;

	//是否进入死区
	if ((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);

		pid->dout = pid->kd * (pid->err - pid->last_err);

		//积分是否超出限制
		if (pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if (pid->iout < -pid->IntegralLimit)
			pid->iout = -pid->IntegralLimit;

		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
		if (pid->output > pid->MaxOutput)
		{
			pid->output = pid->MaxOutput;
		}
		if (pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	}

	return pid->output;
}
void pid_init(PID_TypeDef *pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
