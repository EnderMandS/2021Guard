#ifndef __BSP_JUDGE_H__
#define __BSP_JUDGE_H__

#include <stdbool.h>

extern int Cartridge_Speed_Offset;
extern bool No_Bullet;

void Check_Being_Hit(void);
void Power_Heat_Cheak(void);
void Empty_Bullet(void);

#endif
