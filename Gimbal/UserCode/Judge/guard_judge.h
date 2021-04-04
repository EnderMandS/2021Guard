#ifndef __GUARD_JUDGE_H__
#define __GUARD_JUDGE_H__

#include "main.h"

#define Classic_Power_Max 30
#define Shoot_Speed_Max 30
#define Shoot_Heat_Max 320
#define Shoot_Heat_Down 50

extern uint8_t Shoot_Heat_Allow;

void Classic_Speed_Check(void);
void Shoot_Speed_Check(void);

#endif
