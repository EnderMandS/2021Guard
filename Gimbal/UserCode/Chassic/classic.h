/*
 * @Author: M
 * @Date: 2021-04-11 09:56:15
 * @LastEditTime: 2021-08-09 15:23:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMe:\RM\Guard\NewGuard\Gimbal\UserCode\Chassic\classic.h
 */
#ifndef _CHASSIC_H
#define _CHASSIC_H
#include "pid.h"

#define Classic_StdID 			0x101

#define Classic_Allow 			0x11
#define Classic_Stop 				0x22

#define Classic_Speed_Up		0xA0
#define Classic_Speed_Down	0xB0
#define Classic_Speed_0			0x00
#define Classic_Speed_1			0x01
#define Classic_Speed_2			0x02
#define Classic_Speed_3			0x03
#define Classic_Speed_4			0x04
#define Classic_Speed_5			0x05
#define Classic_Speed_6			0x06
#define Classic_Speed_7			0x07
#define Classic_Speed_8			0x08
#define Classic_Speed_9			0x09
#define Classic_Speed_A			0x0A
#define Classic_Speed_B			0x0B
#define Classic_Speed_C			0x0C
#define Classic_Speed_D			0x0D
#define Classic_Speed_E			0x0E
#define Classic_Speed_F			0x0F

#define Left_Switch		0
#define Right_Switch	1

//²¦µ¯
#define Cartridge_Stop			0x11
#define Cartridge_Low				0x22	//500
#define Cartridge_Middle		0x33	//750
#define Cartridge_High			0x44	//1000

#define Cartridge_Low_Speed			500
#define Cartridge_Middle_Speed	750
#define Cartridge_High_Speed		2000

void Chassis_init(void);
void Chassis_Control(void);
void Chassis_Control_test(void);
float Slow_Change_Speed(int dir, uint16_t Speed);
void Chassic_Ctrl(uint8_t *Data, uint8_t Len);

extern PID_TypeDef motor_pid[2];
extern uint8_t Classic_Ctrl[8];
extern uint8_t Classic_First_Start;
extern uint8_t Chassic_State;

#endif
