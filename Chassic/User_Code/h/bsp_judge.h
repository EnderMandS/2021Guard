#ifndef __BSP_JUDGE_H__
#define __BSP_JUDGE_H__

#include "main.h"
#include <stdbool.h>

typedef struct
{
	bool Supply_Health_Occupy_1;	//��������վ 1�Ų�Ѫ��ռ��״̬
	bool Supply_Health_Occupy_2;	//��������վ 2�Ų�Ѫ��ռ��״̬
	bool Supply_Health_Occupy_3;	//��������վ 3�Ų�Ѫ��ռ��״̬
	bool Hit_Area_Occupy;	//�����ռ��״
	bool Small_Buff_Active;	//С�������ؼ���״̬
	bool Big_Buff_Active;	//���������ؼ���״̬
	bool R2_Occupy;	//����R2���θߵ�ռ��״̬
	bool R3_Occupy;	//����R3���θߵ�ռ��״̬
	bool R4_Occupy;	//����R4���θߵ�ռ��״̬
	bool Base_Shield_Existence;	//�������ػ���״̬
	bool Outpost_Alive;	//����ǰ��ս���״̬
}Field_Event;

extern bool No_Bullet;
extern bool Rand_Change_Flag;
extern Field_Event Field_Event_Data;
extern uint8_t Inspect_Position;

void Check_Being_Hit(void);
void Rand_Dir_Change(void);
void Event_Decode(uint32_t Event);
void Receive_Robot_Interactive(void);

#endif