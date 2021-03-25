#ifndef __BULLET_H__
#define __BULLET_H__

#include "main.h"

#define Bullet_Full		1
#define Bullet_Empty	0

#define Shoot_Switch_Up_Pin 		GPIOE
#define Shoot_Switch_Down_Pin 	GPIOE
#define Shoot_Switch_Up_Port		GPIO_PIN_13
#define Shoot_Switch_Down_Port	GPIO_PIN_14

extern uint8_t Switch_State[2];

void Updata_Switch_State(void);
uint8_t Enough_Heat(void);

#endif
